#include <fstream>
#include <iomanip>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

// Functionality from GDAL for projections
#include <gdal/ogr_spatialref.h>

struct bathy_interface_t
{

	double interval_s; // interval at which to check if bathy needs to be updated.

	// Timing
	double last_update;

	// Current tile
	int grid_priority;
	bool initialized;
	int row, col;
};

struct bathy_grid_t
{

	std::string prefix;
	int priority;

	int epsg;
	OGRSpatialReference srs; // spatial reference system
	OGRCoordinateTransformation *poCT;

	// lower-left-hand-corner.
	double anchor_lat;
	double anchor_lon;

	double spacing_lat;
	double spacing_lon;

	int colmax;
	int rowmax;
};

namespace gazebo
{
	class BathyPlugin : public WorldPlugin
	{
	public:
		void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
		{

			this->world = _parent;
			this->sdf = _sdf;

			// Load the spatial reference system (simulation coordinates)
			this->srs.importFromEPSG(this->sdf->GetElement("projection")->Get<int>("epsg"));
			OGRSpatialReference tsrs;
			tsrs.importFromEPSG(4326); // lat/lon
			this->poCT = OGRCreateCoordinateTransformation(&this->srs, &tsrs);

			// populate the list of available bathymetry.
			if (_sdf->HasElement("bathymetry"))
			{
				sdf::ElementPtr bathySDF = this->sdf->GetElement("bathymetry");
				if (!bathySDF->HasElement("grid"))
				{
					gzerr << "No grids specified for bathy_plugin!" << std::endl;
					return;
				}
				sdf::ElementPtr gridSDF = bathySDF->GetElement("grid");
				while (gridSDF)
				{

					gzdbg << "Found bathymetry: " << gridSDF->Get<std::string>("prefix") << std::endl;

					bathy_grid_t *bg = new bathy_grid_t;

					bg->prefix = gridSDF->Get<std::string>("prefix");
					bg->priority = gridSDF->Get<int>("priority");

					bg->epsg = gridSDF->Get<int>("epsg");
					bg->srs.importFromEPSG(bg->epsg);
					OGRSpatialReference tsrs;
					tsrs.importFromEPSG(4326); // lat/lon
					bg->poCT = OGRCreateCoordinateTransformation(&this->srs, &tsrs);

					bg->anchor_lat = gridSDF->Get<double>("anchor_lat");
					bg->anchor_lon = gridSDF->Get<double>("anchor_lon");
					bg->spacing_lat = gridSDF->Get<double>("spacing_lat");
					bg->spacing_lon = gridSDF->Get<double>("spacing_lon");
					bg->rowmax = gridSDF->Get<double>("rowmax");
					bg->colmax = gridSDF->Get<double>("colmax");

					this->bathy_grids.push_back(bg);

					gridSDF = bathySDF->GetNextElement("grid");
				}
			}

			// Sort available bathymetry by priority
			std::sort(this->bathy_grids.begin(), this->bathy_grids.end(), [](const bathy_grid_t *first, const bathy_grid_t *second) { return (first->priority > second->priority); });

			gzdbg << "Loaded " << this->bathy_grids.size() << " bathymetry data sources." << std::endl;

			// Listen to the update event.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&BathyPlugin::OnUpdate, this, _1));
		}

	public:
		void OnUpdate(const common::UpdateInfo &_info)
		{

			// Walk though all active models and compare against existing list.
			// Add any new ones, delete any that have disappeared.
			for (auto &ii : this->world->Models())
			{
				bool isnew = true;
				for (auto &iii : this->models)
				{
					if (ii->GetId() == std::get<0>(iii)->GetId())
					{
						isnew = false;
					}
				}
				if (isnew) // in the list of active models but not in the internal list
				{
					gzmsg << "FLAGFLSG" << std::endl;

					this->models.push_back(std::make_tuple(ii, static_cast<bathy_interface_t *>(nullptr)));

					if (ii->GetSDF()->HasElement("plugin"))
					{
						gzmsg << "FLAGFLSG2" << std::endl;
						sdf::ElementPtr pluginSDF = ii->GetSDF()->GetElement("plugin");
						while (pluginSDF)
						{
							if (pluginSDF->HasElement("bathymetry"))
							{

								gzdbg << "Loading bathymetry plugin for " << ii->GetName() << "..." << std::endl;

								sdf::ElementPtr bathySDF = pluginSDF->GetElement("bathymetry");

								bathy_interface_t *bathy_interface = new bathy_interface_t;
								bathy_interface->interval_s = bathySDF->Get<double>("interval_s");
								bathy_interface->last_update = 0.0;

								bathy_interface->initialized = false;

								this->models.pop_back();
								std::tuple<gazebo::physics::ModelPtr, bathy_interface_t *> replacement(ii, bathy_interface);
								this->models.push_back(replacement);
							}
							pluginSDF = pluginSDF->GetNextElement();
						}
					}
				}
			}

			// Delete models that have disappeared.
			auto ii = this->models.begin();
			while (ii != this->models.end())
			{
				bool isgone = true;
				for (auto &iii : this->world->Models())
				{
					if (std::get<0>(*ii)->GetId() == iii->GetId())
					{
						isgone = false;
					}
				}
				if (isgone) // in the internal list but not in the list of active models
				{
					ii = this->models.erase(ii);
				}
				else
				{
					++ii;
				}
			}

			// Need to update tiles for all bathy sources.  Some robots may move between bathy sources.
			std::vector<std::list<std::tuple<int, int>>> indeces_to_add;
			std::vector<std::list<std::tuple<int, int>>> indeces_to_del;
			indeces_to_add.resize(this->bathy_grids.size());
			indeces_to_del.resize(this->bathy_grids.size());

			// Do whatever this plugin does for the models in the list.
			// cannot actually add and delete tiles here because multiple robots may share one, so
			// instead just populate the indeces_to_add/del lists.
			for (auto &mm : this->models)
			{
				gazebo::physics::ModelPtr ii = std::get<0>(mm);
				if (ii->GetSDF()->HasElement("plugin"))
				{
					sdf::ElementPtr pluginSDF = ii->GetSDF()->GetElement("plugin");
					while (pluginSDF)
					{
						if (pluginSDF->HasElement("bathymetry"))
						{
							bathy_interface_t *bathy = std::get<1>(mm);

							// Is it time to update?
							double now = _info.simTime.Double();
							if (now - bathy->last_update >= bathy->interval_s)
							{
								bathy->last_update = now;

								gzdbg << "Update bathy!" << std::endl;

								// Get the geographic coordinates.  Use our geographic transformation, not gazebo's since our
								// projection could be different.
								ignition::math::Pose3d pose = ii->WorldPose();

								double sNorthing = pose.Pos().Y(); // native projection (source coordinates)
								double sEasting = pose.Pos().X();
								double tLat = sNorthing; // This is both the input and output to Transform() below.
								double tLon = sEasting;

								if (this->poCT == NULL || !this->poCT->Transform(1, &tLon, &tLat))
								{
									gzerr << "Projected coordinate transformation failed." << std::endl;
								}
								else
								{
									// transform succeeded.  Determine the grid node indeces for each of available bathymetry
									// ignore out of range bathymetry - or just do nothing if the file doesn't exist?
									for (int nn = 0; nn < this->bathy_grids.size(); nn++)
									{
										bathy_grid_t *bg = this->bathy_grids[nn];

										int col = floor((tLon - bg->anchor_lon) / bg->spacing_lon);
										int row = floor((tLat - bg->anchor_lat) / bg->spacing_lat);

										// col = 1;
										// row = 1;
										gzmsg << col << std::endl;
										gzmsg << row << std::endl;
										gzmsg << tLon << std::endl;
										gzmsg << tLat << std::endl;

										// If bathymetry exists for this location it is the highest priority available
										// so use it and don't check lower priority bathy.
										if (col < 0 || col > bg->colmax || row < 0 || row > bg->rowmax)
										{

											gzdbg << "No bathymetry found in priority " << bg->priority << " data for robot " << ii->GetName();

											continue;
										}

										/// @@@ flaw here for if you fall off a map and then come back onto a different one.

										// Check if the model has moved into a new tile.
										if (!bathy->initialized)
										{
											bathy->grid_priority = bg->priority;
											bathy->row = row;
											bathy->col = col;
											indeces_to_add[nn].push_back(std::tuple<int, int>(row, col));
											bathy->initialized = true;
											continue;
										}
										if (bathy->row == row && bathy->col == col && bathy->grid_priority == bg->priority)
										{
											continue; // model has not moved off its tile and there is no better bathymetry available.
										}
										else
										{
											// model has moved tiles.  Delete the last one, add the new one.  These may be from different datasets.
											if (bathy->grid_priority == bg->priority)
											{
												indeces_to_del[nn].push_back(std::tuple<int, int>(bathy->row, bathy->col));
											}
											else
											{
												std::vector<bathy_grid_t *>::iterator oldb = std::find_if(this->bathy_grids.begin(),
																										  this->bathy_grids.end(),
																										  [bathy](const bathy_grid_t *b) { return (b->priority == bathy->grid_priority); });
												int index = std::distance(this->bathy_grids.begin(), oldb);
												indeces_to_del[index].push_back(std::tuple<int, int>(bathy->row, bathy->col));
											}

											bathy->row = row;
											bathy->col = col;
											bathy->grid_priority = bg->priority;
											indeces_to_add[nn].push_back(std::tuple<int, int>(bathy->row, bathy->col));
										}

										// If we got this far then bathymetry from this priority has been used and so do not overwrite that with any lower
										// priority bathy.  Exit the loop.
										break;

										//gzdbg << "lon/col: " << tLon << "/" << col << " lat/row: " << tLat << "/" << row << std::endl;
									}
								}
							}
						}
						pluginSDF = pluginSDF->GetNextElement("plugin");
					}
				}
			}

			for (int nn = 0; nn < indeces_to_add.size(); nn++)
			{
				// Anything to do?
				if (indeces_to_add[nn].size() > 0)
				{

					// Make the list of tiles requiring maintenance unique.
					indeces_to_add[nn].sort();
					indeces_to_add[nn].unique();

					// Loop over the indeces to add.
					for (auto tt : indeces_to_add[nn])
					{

						// Compute the lower-left corner of the tile.
						int row = std::get<0>(tt);
						int col = std::get<1>(tt);
						double lonc = this->bathy_grids[nn]->anchor_lon + col * this->bathy_grids[nn]->spacing_lon;
						double latc = this->bathy_grids[nn]->anchor_lat + row * this->bathy_grids[nn]->spacing_lat;

						gzdbg << "lonc/col: " << lonc << "/" << col << " latc/row: " << latc << "/" << row << std::endl;

						// Generate the bounding box string (assumed format).
						std::ostringstream bboxstr;
						bboxstr << "R_"
								<< std::fixed << std::setprecision(3)
								<< lonc << "_"
								<< lonc + this->bathy_grids[nn]->spacing_lon << "_"
								<< latc << "_"
								<< latc + this->bathy_grids[nn]->spacing_lat;
						std::string fnamestr = "model://" + this->bathy_grids[nn]->prefix + "." + bboxstr.str() + ".epsg" + std::to_string(this->bathy_grids[nn]->epsg);
						std::string modelnamestr = this->bathy_grids[nn]->prefix + "." + bboxstr.str();

						// fnamestr = "model://ncei19.R_-70.750_-70.738_41.520_41.530.epsg26987";

						this->world->InsertModelFile(fnamestr);
						gzdbg << "Inserted model: " << modelnamestr << std::endl;
						// Next call almost always returns NULL because apparently the one above isn't blocking and insertion takes some time.
						/*
		  if (this->world->ModelByName(modelnamestr) != NULL)
		  {
		  gzdbg << "Inserted model: " << fnamestr << std::endl;
		  }
		  else
		  {
		  gzerr << "Failed to insert model: " << fnamestr << std::endl;
		  }
		*/
					}
				}
			}

			// Any tiles to remove?
			for (int nn = 0; nn < indeces_to_add.size(); nn++)
			{
				if (indeces_to_del[nn].size() > 0)
				{

					// Make the list of tiles requiring maintenance unique.
					indeces_to_del[nn].sort();
					indeces_to_del[nn].unique();

					// Loop over the indeces to remove.
					for (auto tt : indeces_to_del[nn])
					{

						// Compute the lower-left corner of the tile.
						int row = std::get<0>(tt);
						int col = std::get<1>(tt);
						double lonc = this->bathy_grids[nn]->anchor_lon + col * this->bathy_grids[nn]->spacing_lon;
						double latc = this->bathy_grids[nn]->anchor_lat + row * this->bathy_grids[nn]->spacing_lat;

						gzdbg << "lonc/col: " << lonc << "/" << col << " latc/row: " << latc << "/" << row << std::endl;

						// Generate the bounding box string (assumed format).
						std::ostringstream bboxstr;
						bboxstr << "R_"
								<< std::fixed << std::setprecision(3)
								<< lonc << "_"
								<< lonc + this->bathy_grids[nn]->spacing_lon << "_"
								<< latc << "_"
								<< latc + this->bathy_grids[nn]->spacing_lat;
						std::string modelnamestr = this->bathy_grids[nn]->prefix + "." + bboxstr.str();

						// @@@@ right messages are coming out but model not being deleted.  it was in the original versino of this.
						this->world->RemoveModel(modelnamestr);
						gzdbg << "Removed model: " << modelnamestr << std::endl;
					}
				}
			}

		} // OnUpdate

	private:
		gazebo::physics::WorldPtr world;

	private:
		sdf::ElementPtr sdf;

	protected:
		gazebo::event::ConnectionPtr updateConnection;

	private:
		OGRSpatialReference srs; // spatial reference system
	private:
		OGRCoordinateTransformation *poCT;

	private:
		std::list<std::tuple<gazebo::physics::ModelPtr, bathy_interface_t *>> models;

	private:
		std::vector<bathy_grid_t *> bathy_grids;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_WORLD_PLUGIN(BathyPlugin)
} // namespace gazebo
