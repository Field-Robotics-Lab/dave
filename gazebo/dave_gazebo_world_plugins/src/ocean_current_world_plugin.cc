// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file ocean_current_world_plugin.cc

#include <math.h>
#include <dave_gazebo_world_plugins/ocean_current_world_plugin.h>
#include <StratifiedCurrentVelocity.pb.h>

#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <sdf/sdf.hh>

namespace gazebo {

/////////////////////////////////////////////////
UnderwaterCurrentPlugin::UnderwaterCurrentPlugin()
{
  // Doing nothing for now
}

/////////////////////////////////////////////////
UnderwaterCurrentPlugin::~UnderwaterCurrentPlugin()
{
#if GAZEBO_MAJOR_VERSION >= 8
  this->updateConnection.reset();
#else
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
#endif
}

/////////////////////////////////////////////////
void UnderwaterCurrentPlugin::
  Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world != NULL, "World pointer is invalid");
  GZ_ASSERT(_sdf != NULL, "SDF pointer is invalid");

  this->world = _world;
  this->sdf = _sdf;

  // Read the namespace for topics and services
  this->ns = _sdf->Get<std::string>("namespace");

  gzmsg << "Loading underwater world..." << std::endl;
  // Initializing the transport node
  this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION >= 8
  this->node->Init(this->world->Name());
#else
  this->node->Init(this->world->GetName());
#endif

  // Initialize the time update
#if GAZEBO_MAJOR_VERSION >= 8
  this->lastUpdate = this->world->SimTime();
#else
  this->lastUpdate = this->world->GetSimTime();
#endif

  this->LoadGlobalCurrentConfig();
  this->LoadStratifiedCurrentDatabase();
  if (this->sdf->HasElement("tidal_oscillation"))
  {
    this->LoadTidalOscillationDatabase();
  }

  // Connect the update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&UnderwaterCurrentPlugin::Update,
    this, _1));

  gzmsg << "Underwater current plugin loaded!" << std::endl
    << "\tWARNING: Current velocity calculated in the ENU frame"
    << std::endl;
}

/////////////////////////////////////////////////
void UnderwaterCurrentPlugin::Init()
{
  // Doing nothing for now
}

/////////////////////////////////////////////////
void UnderwaterCurrentPlugin::Update(const common::UpdateInfo & /** _info */)
{
#if GAZEBO_MAJOR_VERSION >= 8
  common::Time time = this->world->SimTime();
#else
  common::Time time = this->world->GetSimTime();
#endif

  // Calculate the flow velocity and the direction using the Gauss-Markov
  // model

  // Update current velocity
  double currentVelMag = this->currentVelModel.Update(time.Double());
  // Update current horizontal direction around z axis of flow frame
  double horzAngle = this->currentHorzAngleModel.Update(time.Double());

  // Update current horizontal direction around z axis of flow frame
  double vertAngle = this->currentVertAngleModel.Update(time.Double());

  // Generating the current velocity vector as in the NED frame
  this->currentVelocity = ignition::math::Vector3d(
      currentVelMag * cos(horzAngle) * cos(vertAngle),
      currentVelMag * sin(horzAngle) * cos(vertAngle),
      currentVelMag * sin(vertAngle));

  // Generate the depth-specific velocities
  this->currentStratifiedVelocity.clear();
  for (int i = 0; i < this->stratifiedDatabase.size(); i++)
  {
      double depth = this->stratifiedDatabase[i].Z();
      currentVelMag =
        this->stratifiedCurrentModels[i][0].Update(time.Double());
      horzAngle =
        this->stratifiedCurrentModels[i][1].Update(time.Double());
      vertAngle =
        this->stratifiedCurrentModels[i][2].Update(time.Double());
      ignition::math::Vector4d depthVel(
          currentVelMag * cos(horzAngle) * cos(vertAngle),
          currentVelMag * sin(horzAngle) * cos(vertAngle),
          currentVelMag * sin(vertAngle),
          depth);
      this->currentStratifiedVelocity.push_back(depthVel);
  }

  // Update time stamp
  this->lastUpdate = time;
  this->PublishCurrentVelocity();
  this->PublishStratifiedCurrentVelocity();
}

/////////////////////////////////////////////////
void UnderwaterCurrentPlugin::LoadGlobalCurrentConfig()
{
  // NOTE: The plugin currently requires stratified current, so the
  //       global current set up in this method is potentially
  //       inconsistent or redundant.
  //       Consider setting it up as one or the other, but not both?

  // Retrieve the velocity configuration, if existent
  GZ_ASSERT(this->sdf->HasElement("constant_current"),
    "Current configuration not available");
  sdf::ElementPtr currentVelocityParams = this->sdf->GetElement(
    "constant_current");

  // Read the topic names from the SDF file
  if (currentVelocityParams->HasElement("topic"))
    this->currentVelocityTopic =
      currentVelocityParams->Get<std::string>("topic");
  else
    this->currentVelocityTopic = "current_velocity";

  GZ_ASSERT(!this->currentVelocityTopic.empty(),
    "Empty ocean current velocity topic");

  if (currentVelocityParams->HasElement("velocity"))
  {
    sdf::ElementPtr elem = currentVelocityParams->GetElement("velocity");
    if (elem->HasElement("mean"))
        this->currentVelModel.mean = elem->Get<double>("mean");
    if (elem->HasElement("min"))
        this->currentVelModel.min = elem->Get<double>("min");
    if (elem->HasElement("max"))
        this->currentVelModel.max = elem->Get<double>("max");
    if (elem->HasElement("mu"))
        this->currentVelModel.mu = elem->Get<double>("mu");
    if (elem->HasElement("noiseAmp"))
        this->currentVelModel.noiseAmp = elem->Get<double>("noiseAmp");

    GZ_ASSERT(this->currentVelModel.min < this->currentVelModel.max,
      "Invalid current velocity limits");
    GZ_ASSERT(this->currentVelModel.mean >= this->currentVelModel.min,
      "Mean velocity must be greater than minimum");
    GZ_ASSERT(this->currentVelModel.mean <= this->currentVelModel.max,
      "Mean velocity must be smaller than maximum");
    GZ_ASSERT(this->currentVelModel.mu >= 0 && this->currentVelModel.mu < 1,
      "Invalid process constant");
    GZ_ASSERT(this->currentVelModel.noiseAmp < 1 &&
      this->currentVelModel.noiseAmp >= 0,
      "Noise amplitude has to be smaller than 1");
  }

  this->currentVelModel.var = this->currentVelModel.mean;
  gzmsg << "Current velocity [m/s] Gauss-Markov process model:" << std::endl;
  this->currentVelModel.Print();

  if (currentVelocityParams->HasElement("horizontal_angle"))
  {
    sdf::ElementPtr elem =
      currentVelocityParams->GetElement("horizontal_angle");

    if (elem->HasElement("mean"))
      this->currentHorzAngleModel.mean = elem->Get<double>("mean");
    if (elem->HasElement("min"))
      this->currentHorzAngleModel.min = elem->Get<double>("min");
    if (elem->HasElement("max"))
      this->currentHorzAngleModel.max = elem->Get<double>("max");
    if (elem->HasElement("mu"))
      this->currentHorzAngleModel.mu = elem->Get<double>("mu");
    if (elem->HasElement("noiseAmp"))
      this->currentHorzAngleModel.noiseAmp = elem->Get<double>("noiseAmp");

    GZ_ASSERT(this->currentHorzAngleModel.min <
      this->currentHorzAngleModel.max,
      "Invalid current horizontal angle limits");
    GZ_ASSERT(this->currentHorzAngleModel.mean >=
      this->currentHorzAngleModel.min,
      "Mean horizontal angle must be greater than minimum");
    GZ_ASSERT(this->currentHorzAngleModel.mean <=
      this->currentHorzAngleModel.max,
      "Mean horizontal angle must be smaller than maximum");
    GZ_ASSERT(this->currentHorzAngleModel.mu >= 0 &&
      this->currentHorzAngleModel.mu < 1,
      "Invalid process constant");
    GZ_ASSERT(this->currentHorzAngleModel.noiseAmp < 1 &&
      this->currentHorzAngleModel.noiseAmp >= 0,
      "Noise amplitude for horizontal angle has to be between 0 and 1");
  }

  this->currentHorzAngleModel.var = this->currentHorzAngleModel.mean;
  gzmsg <<
    "Current velocity horizontal angle [rad] Gauss-Markov process model:"
    << std::endl;
  this->currentHorzAngleModel.Print();

  if (currentVelocityParams->HasElement("vertical_angle"))
  {
    sdf::ElementPtr elem = currentVelocityParams->GetElement("vertical_angle");

    if (elem->HasElement("mean"))
      this->currentVertAngleModel.mean = elem->Get<double>("mean");
    if (elem->HasElement("min"))
      this->currentVertAngleModel.min = elem->Get<double>("min");
    if (elem->HasElement("max"))
      this->currentVertAngleModel.max = elem->Get<double>("max");
    if (elem->HasElement("mu"))
      this->currentVertAngleModel.mu = elem->Get<double>("mu");
    if (elem->HasElement("noiseAmp"))
      this->currentVertAngleModel.noiseAmp = elem->Get<double>("noiseAmp");

    GZ_ASSERT(this->currentVertAngleModel.min <
      this->currentVertAngleModel.max, "Invalid current vertical angle limits");
    GZ_ASSERT(this->currentVertAngleModel.mean >=
      this->currentVertAngleModel.min,
      "Mean vertical angle must be greater than minimum");
    GZ_ASSERT(this->currentVertAngleModel.mean <=
      this->currentVertAngleModel.max,
      "Mean vertical angle must be smaller than maximum");
    GZ_ASSERT(this->currentVertAngleModel.mu >= 0 &&
      this->currentVertAngleModel.mu < 1,
      "Invalid process constant");
    GZ_ASSERT(this->currentVertAngleModel.noiseAmp < 1 &&
      this->currentVertAngleModel.noiseAmp >= 0,
      "Noise amplitude for vertical angle has to be between 0 and 1");
  }

  this->currentVertAngleModel.var = this->currentVertAngleModel.mean;
  gzmsg <<
    "Current velocity vertical angle [rad] Gauss-Markov process model:"
    << std::endl;
  this->currentVertAngleModel.Print();

  this->currentVelModel.lastUpdate = this->lastUpdate.Double();
  this->currentHorzAngleModel.lastUpdate = this->lastUpdate.Double();
  this->currentVertAngleModel.lastUpdate = this->lastUpdate.Double();

  // Advertise the current velocity & stratified current velocity topics
  this->publishers[this->currentVelocityTopic] =
    this->node->Advertise<msgs::Vector3d>(
    this->ns + "/" + this->currentVelocityTopic);
  gzmsg << "Current velocity topic name: " <<
    this->ns + "/" + this->currentVelocityTopic << std::endl;
}

/////////////////////////////////////////////////
void UnderwaterCurrentPlugin::LoadStratifiedCurrentDatabase()
{
  GZ_ASSERT(this->sdf->HasElement("transient_current"),
    "Transient current configuration not available");
  sdf::ElementPtr transientCurrentParams = this->sdf->GetElement(
    "transient_current");

  if (transientCurrentParams->HasElement("topic_stratified"))
    this->stratifiedCurrentVelocityTopic =
      transientCurrentParams->Get<std::string>("topic_stratified");
  else
    this->stratifiedCurrentVelocityTopic = "stratified_current_velocity";

  GZ_ASSERT(!this->stratifiedCurrentVelocityTopic.empty(),
    "Empty stratified ocean current velocity topic");

  // Read the depth dependent ocean current file path from the SDF file
  if (transientCurrentParams->HasElement("databasefilePath"))
    this->databaseFilePath =
      transientCurrentParams->Get<std::string>("databasefilePath");
  else
  {
    this->databaseFilePath = "transientOceanCurrentDatabase.csv";
  }

  GZ_ASSERT(!this->databaseFilePath.empty(),
    "Empty stratified ocean current database file path");

  gzmsg << this->databaseFilePath << std::endl;

#if GAZEBO_MAJOR_VERSION >= 8
  this->lastUpdate = this->world->SimTime();
#else
  this->lastUpdate = this->world->GetSimTime();
#endif

  // Read database
  std::ifstream csvFile;
  std::string line;
  csvFile.open(this->databaseFilePath);
  if (!csvFile)
  {
    common::SystemPaths *paths = common::SystemPaths::Instance();
    this->databaseFilePath =
      paths->FindFile(this->databaseFilePath, true);
    csvFile.open(this->databaseFilePath);
  }
  GZ_ASSERT(csvFile, "Stratified Ocean database file does not exist");

  gzmsg << "Statified Ocean Current Database loaded : "
        << this->databaseFilePath << std::endl;

  // skip the 3 lines
  getline(csvFile, line);
  getline(csvFile, line);
  getline(csvFile, line);
  while (getline(csvFile, line))
  {
      if (line.empty())  // skip empty lines:
      {
          continue;
      }
      std::istringstream iss(line);
      std::string lineStream;
      std::string::size_type sz;
      std::vector <long double> row;
      while (getline(iss, lineStream, ','))
      {
          row.push_back(stold(lineStream, &sz));  // convert to double
      }
      ignition::math::Vector3d read;
      read.X() = row[0];
      read.Y() = row[1];
      read.Z() = row[2];
      this->stratifiedDatabase.push_back(read);

      // Create Gauss-Markov processes for the stratified currents
      // Means are the database-specified magnitudes & angles, and
      // the other values come from the constant current models
      // TODO: Vertical angle currently set to 0 (not in database)
      GaussMarkovProcess magnitudeModel;
      magnitudeModel.mean = hypot(row[1], row[0]);
      magnitudeModel.var = magnitudeModel.mean;
      magnitudeModel.max = this->currentVelModel.max;
      magnitudeModel.min = 0.0;
      magnitudeModel.mu = this->currentVelModel.mu;
      magnitudeModel.noiseAmp = this->currentVelModel.noiseAmp;
      magnitudeModel.lastUpdate = this->lastUpdate.Double();

      GaussMarkovProcess hAngleModel;
      hAngleModel.mean = atan2(row[1], row[0]);
      hAngleModel.var = hAngleModel.mean;
      hAngleModel.max = M_PI;
      hAngleModel.min = -M_PI;
      hAngleModel.mu = this->currentHorzAngleModel.mu;
      hAngleModel.noiseAmp = this->currentHorzAngleModel.noiseAmp;
      hAngleModel.lastUpdate = this->lastUpdate.Double();

      GaussMarkovProcess vAngleModel;
      vAngleModel.mean = 0.0;
      vAngleModel.var = vAngleModel.mean;
      vAngleModel.max = M_PI/2.0;
      vAngleModel.min = -M_PI/2.0;
      vAngleModel.mu = this->currentVertAngleModel.mu;
      vAngleModel.noiseAmp = this->currentVertAngleModel.noiseAmp;
      vAngleModel.lastUpdate = this->lastUpdate.Double();

      std::vector<GaussMarkovProcess> depthModels;
      depthModels.push_back(magnitudeModel);
      depthModels.push_back(hAngleModel);
      depthModels.push_back(vAngleModel);
      this->stratifiedCurrentModels.push_back(depthModels);
  }
  csvFile.close();

  this->publishers[this->stratifiedCurrentVelocityTopic] =
    this->node->Advertise<
      dave_gazebo_world_plugins_msgs::msgs::StratifiedCurrentVelocity>(
      this->ns + "/" + this->stratifiedCurrentVelocityTopic);
  gzmsg << "Stratified current velocity topic name: " <<
    this->ns + "/" + this->stratifiedCurrentVelocityTopic << std::endl;
}

/////////////////////////////////////////////////
void UnderwaterCurrentPlugin::LoadTidalOscillationDatabase()
{
  this->tideFlag = true;
  this->tidalHarmonicFlag = false;

  sdf::ElementPtr tidalOscillationParams =
    this->sdf->GetElement("tidal_oscillation");
  sdf::ElementPtr tidalHarmonicParams;

  // Read the tidal oscillation parameter from the SDF file
  if (tidalOscillationParams->HasElement("databasefilePath"))
  {
    this->tidalFilePath =
      tidalOscillationParams->Get<std::string>("databasefilePath");
    gzmsg << "Tidal current database configuration found" << std::endl;
  }
  else
  {
    if (tidalOscillationParams->HasElement("harmonic_constituents"))
    {
      tidalHarmonicParams =
        tidalOscillationParams->GetElement("harmonic_constituents");
      gzmsg << "Tidal harmonic constituents "
            << "configuration found" << std::endl;
      tidalHarmonicFlag = true;
    }
    else
      this->tidalFilePath = ros::package::getPath("dave_worlds") +
        "/worlds/ACT1951_predictionMaxSlack_2021-02-24.csv";
  }

  // Read the tidal oscillation direction from the SDF file
  GZ_ASSERT(tidalOscillationParams->HasElement("mean_direction"),
    "Tidal mean direction not defined");
  if (tidalOscillationParams->HasElement("mean_direction"))
  {
    sdf::ElementPtr elem =
      tidalOscillationParams->GetElement("mean_direction");
    GZ_ASSERT(elem->HasElement("ebb"),
      "Tidal mean ebb direction not defined");
    this->ebbDirection = elem->Get<double>("ebb");
    this->floodDirection = elem->Get<double>("flood");
    GZ_ASSERT(elem->HasElement("flood"),
      "Tidal mean flood direction not defined");
  }

  // Read the world start time (GMT) from the SDF file
  GZ_ASSERT(tidalOscillationParams->HasElement("world_start_time_GMT"),
    "World start time (GMT) not defined");
  if (tidalOscillationParams->HasElement("world_start_time_GMT"))
  {
    sdf::ElementPtr elem =
      tidalOscillationParams->GetElement("world_start_time_GMT");
    GZ_ASSERT(elem->HasElement("day"),
      "World start time (day) not defined");
    this->world_start_time_day = elem->Get<double>("day");
    GZ_ASSERT(elem->HasElement("month"),
      "World start time (month) not defined");
    this->world_start_time_month = elem->Get<double>("month");
    GZ_ASSERT(elem->HasElement("year"),
      "World start time (year) not defined");
    this->world_start_time_year = elem->Get<double>("year");
    GZ_ASSERT(elem->HasElement("hour"),
      "World start time (hour) not defined");
    this->world_start_time_hour = elem->Get<double>("hour");
    if (elem->HasElement("minute"))
      this->world_start_time_minute = elem->Get<double>("minute");
    else
      this->world_start_time_minute = 0;
  }

  if (tidalHarmonicFlag)
  {
    // Read harmonic constituents
    GZ_ASSERT(tidalHarmonicParams->HasElement("M2"),
      "Harcomnic constituents M2 not found");
    sdf::ElementPtr M2Params = tidalHarmonicParams->GetElement("M2");
    this->M2_amp = M2Params->Get<double>("amp");
    this->M2_phase = M2Params->Get<double>("phase");
    this->M2_speed = M2Params->Get<double>("speed");
    GZ_ASSERT(tidalHarmonicParams->HasElement("S2"),
      "Harcomnic constituents S2 not found");
    sdf::ElementPtr S2Params = tidalHarmonicParams->GetElement("S2");
    this->S2_amp = S2Params->Get<double>("amp");
    this->S2_phase = S2Params->Get<double>("phase");
    this->S2_speed = S2Params->Get<double>("speed");
    GZ_ASSERT(tidalHarmonicParams->HasElement("N2"),
      "Harcomnic constituents N2 not found");
    sdf::ElementPtr N2Params = tidalHarmonicParams->GetElement("N2");
    this->N2_amp = N2Params->Get<double>("amp");
    this->N2_phase = N2Params->Get<double>("phase");
    this->N2_speed = N2Params->Get<double>("speed");
    gzmsg << "Tidal harmonic constituents loaded : " << std::endl;
    gzmsg << "M2 amp: " << this->M2_amp << " phase: " << this->M2_phase
          << " speed: " << this->M2_speed << std::endl;
    gzmsg << "S2 amp: " << this->S2_amp << " phase: " << this->S2_phase
          << " speed: " << this->S2_speed << std::endl;
    gzmsg << "N2 amp: " << this->N2_amp << " phase: " << this->N2_phase
          << " speed: " << this->N2_speed << std::endl;
  }
  else
  {
    // Read database
    std::ifstream csvFile;
    std::string line;
    csvFile.open(this->tidalFilePath);
    if (!csvFile)
    {
      common::SystemPaths *paths = common::SystemPaths::Instance();
      this->tidalFilePath =
        paths->FindFile(this->tidalFilePath, true);
      csvFile.open(this->tidalFilePath);
    }
    GZ_ASSERT(csvFile, "Tidal Oscillation database file does not exist");

    gzmsg << "Tidal Oscillation  Database loaded : "
          << this->tidalFilePath << std::endl;

    // skip the first line
    getline(csvFile, line);
    while (getline(csvFile, line))
    {
        if (line.empty())  // skip empty lines:
        {
            continue;
        }
        std::istringstream iss(line);
        std::string lineStream;
        std::string::size_type sz;
        std::vector<std::string> row;
        std::array<int, 5> tmpDateArray;
        while (getline(iss, lineStream, ','))
        {
          row.push_back(lineStream);
        }
        if (strcmp(row[1].c_str(), " slack"))  // skip 'slack' category
        {
          tmpDateArray[0] = std::stoi(row[0].substr(0, 4));
          tmpDateArray[1] = std::stoi(row[0].substr(5, 7));
          tmpDateArray[2] = std::stoi(row[0].substr(8, 10));
          tmpDateArray[3] = std::stoi(row[0].substr(11, 13));
          tmpDateArray[4] = std::stoi(row[0].substr(14, 16));
          this->dateGMT.push_back(tmpDateArray);

          this->speedcmsec.push_back(stold(row[2], &sz));
        }
    }
    csvFile.close();

    // Eliminate data with same consecutive type
    std::vector<int> duplicated;
    for (int i = 0; i  <this->dateGMT.size() - 1; i++)
    {
      // delete latter if same sign
      if (((this->speedcmsec[i] > 0) - (this->speedcmsec[i] < 0))
          == ((this->speedcmsec[i+1] > 0) - (this->speedcmsec[i+1] < 0)))
      {
        duplicated.push_back(i+1);
      }
    }
    int eraseCount = 0;
    for (int i = 0; i < duplicated.size(); i++)
    {
      this->dateGMT.erase(
        this->dateGMT.begin()+duplicated[i]-eraseCount);
      this->speedcmsec.erase(
        this->speedcmsec.begin()+duplicated[i]-eraseCount);
      eraseCount++;
    }
  }
}

/////////////////////////////////////////////////
void UnderwaterCurrentPlugin::PublishCurrentVelocity()
{
  msgs::Vector3d currentVel;
  msgs::Set(&currentVel, ignition::math::Vector3d(this->currentVelocity.X(),
                                                  this->currentVelocity.Y(),
                                                  this->currentVelocity.Z()));
  this->publishers[this->currentVelocityTopic]->Publish(currentVel);
}

/////////////////////////////////////////////////
void UnderwaterCurrentPlugin::PublishStratifiedCurrentVelocity()
{
  dave_gazebo_world_plugins_msgs::msgs::StratifiedCurrentVelocity currentVel;
  for (std::vector<ignition::math::Vector4d>::iterator it =
         this->currentStratifiedVelocity.begin();
       it != this->currentStratifiedVelocity.end(); ++it)
  {
    msgs::Set(currentVel.add_velocity(),
              ignition::math::Vector3d(it->X(), it->Y(), it->Z()));
    currentVel.add_depth(it->W());
  }
  if (currentVel.velocity_size() == 0)
    return;
  this->publishers[this->stratifiedCurrentVelocityTopic]->Publish(currentVel);
}

GZ_REGISTER_WORLD_PLUGIN(UnderwaterCurrentPlugin)
}
