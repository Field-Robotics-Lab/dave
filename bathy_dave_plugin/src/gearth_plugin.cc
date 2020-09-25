#include <fstream>
#include <iomanip>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

// Functionality from GDAL for projections
#include <gdal/ogr_spatialref.h>

struct gearth_interface_t
{


  std::string kmlfile;  // kmlfile to write
  double interval_s; // interval at which to write it
  std::string name; // GE Point name

  // string icon;

  // Timing
  double last_update; 
  
};

namespace gazebo
{
class GEarthPlugin : public WorldPlugin
{
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {

    this->world = _parent;
    this->sdf = _sdf;
    
    // Load the spatial reference system (simulation coordinates)
    this->srs.importFromEPSG(this->sdf->GetElement("projection")->Get<int>("epsg"));
    OGRSpatialReference tsrs;
    tsrs.importFromEPSG(4326); // lat/lon
    this->poCT = OGRCreateCoordinateTransformation(&this->srs, &tsrs );

        
    // Listen to the update event.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GEarthPlugin::OnUpdate, this, _1));

  }
  public: void OnUpdate(const common::UpdateInfo& _info)
  {
    
    // Walk though all active models and compare against existing list.
    // Add any new ones, delete any that have disappeared.
    for (auto& ii : this->world->Models())
      {
	bool isnew =  true;
	for ( auto& iii : this->models )
	  {
	    if ( ii->GetId() == std::get<0>(iii)->GetId() )
	      {
		isnew = false;
	      }
	  }
	if ( isnew ) // in the list of active models but not in the internal list
	  {

	    this->models.push_back(std::make_tuple(ii,static_cast<gearth_interface_t*>(nullptr)));

	    if (ii->GetSDF()->HasElement("plugin"))
	      {
		sdf::ElementPtr pluginSDF = ii->GetSDF()->GetElement("plugin");
		while (pluginSDF) 
		  {
		    if (pluginSDF->HasElement("gearth_interface"))
		      {
			
			gzdbg << "Loading GE plugin for " << ii->GetName() << "..." << std::endl;
			
			sdf::ElementPtr gearthSDF = pluginSDF->GetElement("gearth_interface");
			
			gearth_interface_t* gearth_interface = new gearth_interface_t;
			gearth_interface->kmlfile = gearthSDF->Get<std::string>("kmlfile");
			gearth_interface->interval_s = gearthSDF->Get<double>("interval_s");
			gearth_interface->name = gearthSDF->Get<std::string>("name");
			gearth_interface->last_update = 0.0;

			gzdbg << "Will write lat/lon to " << gearth_interface->kmlfile << " every " << gearth_interface->interval_s << " s for entity " << gearth_interface->name << std::endl;
			
			this->models.pop_back();
			std::tuple<gazebo::physics::ModelPtr,gearth_interface_t*> replacement(ii,gearth_interface);
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
	for ( auto& iii : this->world->Models() )
	  {
	    if ( std::get<0>(*ii)->GetId() == iii->GetId() )
	      {
		isgone = false;
	      }
	  }
	if ( isgone ) // in the internal list but not in the list of active models
	  {
	    ii = this->models.erase(ii);
	  }
	else
	  {
	    ++ii;
	  }
      } 
    
    // Do whatever this plugin does for the models in the list.
    for ( auto& mm : this->models )
      {
    	gazebo::physics::ModelPtr ii = std::get<0>(mm);
	if (ii->GetSDF()->HasElement("plugin"))
	  {
	    sdf::ElementPtr pluginSDF = ii->GetSDF()->GetElement("plugin");
	    while (pluginSDF) 
	      {
		if (pluginSDF->HasElement("gearth_interface"))
		  {
		    gearth_interface_t* ge = std::get<1>(mm);

		    // Is it time to write a new file?
		    double now = _info.simTime.Double();
		    if ( now - ge->last_update >= ge->interval_s )
		    {
		      ge->last_update = now;
		      
		      // Get the geographic coordinates.  Use our geographic transformation, not gazebo's since our 
		      // projection could be different.
		      ignition::math::Pose3d pose = ii->WorldPose();

		      double sNorthing = pose.Pos().Y(); // native projection (source coordinates)
		      double sEasting = pose.Pos().X();
		      double tLat = sNorthing; // This is both the input and output to Transform() below.
		      double tLon = sEasting;

		      if( this->poCT == NULL
			  || !this->poCT->Transform( 1, &tLon, &tLat ) )
			{
			  gzerr << "Projected coordinate transformation failed." << std::endl;
			}
		      else
			{
			  
			  // // Write the file
			  std::ofstream kmlfile;
			  kmlfile.open(ge->kmlfile.c_str(), std::ios::out);
			  if ( kmlfile.is_open() )
			    {
			      kmlfile << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << std::endl;
			      kmlfile << "<kml xmlns=\"http://earth.google.com/kml/2.1\">"
				      << std::endl;
			      kmlfile << "  <Placemark>" << std::endl;
			      kmlfile << "    <Point>" << std::endl;
			      kmlfile << "      <coordinates>"
				      << std::setprecision(9)
				      << tLon << "," << tLat << ",0"
				      <<"</coordinates>"
				      << std::endl;
			      kmlfile << "    </Point>" << std::endl;
			      kmlfile << "  </Placemark>" << std::endl;
			      kmlfile << "</kml>" << std::endl;
			      kmlfile.close();
			    }
			  else
			    {
			      gzerr << "Failed to write kmlfile for " <<ii->GetName() << "!"
				    << std::endl;
			    }
			}
		    }
		  }
		pluginSDF = pluginSDF->GetNextElement("plugin");
	      }
	  }
      }

  } // OnUpdate
  
  private: gazebo::physics::WorldPtr world;
  private: sdf::ElementPtr sdf;
  private: OGRSpatialReference srs; // spatial reference system

  protected: gazebo::event::ConnectionPtr updateConnection;

  private: std::list< std::tuple<gazebo::physics::ModelPtr,gearth_interface_t*> > models;

  private: OGRCoordinateTransformation *poCT;
  
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GEarthPlugin)
}
