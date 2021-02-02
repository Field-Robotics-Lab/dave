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

/// \file UnderwaterCurrentPlugin.cc

#include <math.h>

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
#include <dave_world_plugins/UnderwaterCurrentPlugin.hh>

#include "ros/package.h"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(UnderwaterCurrentPlugin)

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
  // Retrieve the velocity configuration, if existent
  GZ_ASSERT(this->sdf->HasElement("depth_dependent_current"),
    "Current configuration not available");
  sdf::ElementPtr currentVelocityParams = this->sdf->GetElement(
    "depth_dependent_current");

  // Read the topic names from the SDF file
  if (currentVelocityParams->HasElement("topic"))
    this->currentVelocityTopic =
      currentVelocityParams->Get<std::string>("topic");
  else
    this->currentVelocityTopic = "current_velocity";

  GZ_ASSERT(!this->currentVelocityTopic.empty(),
    "Empty ocean current velocity topic");

  if (currentVelocityParams->HasElement("topic_stratified"))
    this->stratifiedCurrentVelocityTopic =
      currentVelocityParams->Get<std::string>("topic_stratified");
  else
    this->stratifiedCurrentVelocityTopic = "stratified_current_velocity";

  GZ_ASSERT(!this->stratifiedCurrentVelocityTopic.empty(),
    "Empty stratified ocean current velocity topic");

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

  // Read the depth dependent ocean current file path from the SDF file
  if (currentVelocityParams->HasElement("database"))
    this->databaseFilePath =
      currentVelocityParams->Get<std::string>("database");
  else
  {
    // To hardcode path:
    // this->databaseFilePath =
    //   "/home/jessica/uuv_ws/src/dave/uuv_dave/worlds/
    //   transientOceanCurrentDatabase.csv";
    //
    // Using boost:
    // boost::filesystem::path
    //    concatPath(boost::filesystem::initial_path().parent_path());
    // concatPath +=
    //    "/uuv_ws/src/dave/uuv_dave/worlds/transientOceanCurrentDatabase.csv";
    // this->databaseFilePath = concatPath.generic_string();
    //
    // Use ros package path:
    this->databaseFilePath = ros::package::getPath("uuv_dave") +
      "/worlds/transientOceanCurrentDatabase.csv";
  }
  GZ_ASSERT(!this->databaseFilePath.empty(),
    "Empty database file path");

  // Read database
  std::ifstream csvFile; std::string line;
  csvFile.open(this->databaseFilePath);
  // skip the 3 lines
  getline(csvFile, line); getline(csvFile, line); getline(csvFile, line);
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
      read.X() = row[0]; read.Y() = row[1]; read.Z() = row[2];
      this->database.push_back(read);
  }

  // Initialize the time update
#if GAZEBO_MAJOR_VERSION >= 8
  this->lastUpdate = this->world->SimTime();
#else
  this->lastUpdate = this->world->GetSimTime();
#endif
  this->currentVelModel.lastUpdate = this->lastUpdate.Double();
  this->currentHorzAngleModel.lastUpdate = this->lastUpdate.Double();
  this->currentVertAngleModel.lastUpdate = this->lastUpdate.Double();

  // Advertise the current velocity topic
  this->publishers[this->currentVelocityTopic] =
    this->node->Advertise<msgs::Vector3d>(
    this->ns + "/" + this->currentVelocityTopic);

  gzmsg << "Current velocity topic name: " <<
    this->ns + "/" + this->currentVelocityTopic << std::endl;

  // Stratified Database only published at ROS plugin. Not here (gazebo)
  // this->publishers[this->stratifiedCurrentVelocityTopic] =
  //   this->node->Advertise<
  //   stratified_current_velocity_msgs::msgs::StratifiedCurrentVelocity>(
  //   this->ns + "/" + this->stratifiedCurrentVelocityTopic);

  // gzmsg << "Stratified current velocity topic name: " <<
  //   this->ns + "/" + this->stratifiedCurrentVelocityTopic << std::endl;

  // Subscribe vehicle depth topic
  this->vehicleDepthTopic = "vehicle_depth";
  this->subscriber = this->node->Subscribe<msgs::Any>(
    this->vehicleDepthTopic, &UnderwaterCurrentPlugin::SubscribeVehicleDepth,
    this);

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

  // Apply Depth dependent ocean current database
  this->ApplyDatabase();

  // Update time stamp
  this->lastUpdate = time;
  this->PublishCurrentVelocity();

  // Stratified Database only published at ROS plugin. Not here (gazebo)
  // this->PublishStratifiedCurrentVelocity();
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
// Stratified Database only published at ROS plugin. Not here (gazebo)
void UnderwaterCurrentPlugin::PublishStratifiedCurrentVelocity()
{
  // stratified_current_velocity_msgs::msgs::StratifiedCurrentVelocity msg;
  // msgs::Vector3d* _velocity = msg.add_velocity();
  // msgs::Any* _depth = msg.add_depth();
  // for (int i = 0; i < this->database.size(); i++) {
  //   msgs::Set(&_velocity[i],
  //         ignition::math::Vector3d(this->database[i].X(),  // northCurrent
  //                                  this->database[i].Y(),  // eastCurrent
  //                                  0.0));
  //   _depth[i] = msgs::ConvertAny(this->database[i].Z());
  // }
  // this->publishers[this->stratifiedCurrentVelocityTopic]->Publish(msg);
}

/////////////////////////////////////////////////
void UnderwaterCurrentPlugin::SubscribeVehicleDepth(AnyPtr &_msg)
{
  this->vehicleDepth = _msg->double_value();
  // gzmsg << "WorldSubscribe : " << this->vehicleDepth << std::endl;
}

/////////////////////////////////////////////////
void UnderwaterCurrentPlugin::ApplyDatabase()
{
  double northCurrent, eastCurrent;
  //--- Interpolate velocity from database ---//
  // find current depth index from database
  // (X: north-direction, Y: east-direction, Z: depth)
  int depthIndex = 0;
  for (int i = 1; i <= this->database.size(); i++) {
    if (this->database[i].Z() > this->vehicleDepth) {
      depthIndex = i; break;
    }
  }
  // interpolate
  if (depthIndex == 0) {  // Deeper than database use deepest value
    northCurrent = this->database.back().X();
    eastCurrent = this->database.back().Y();
  }
  else
  {
      double rate =
        (this->vehicleDepth-this->database[depthIndex-1].Z())
        /(this->database[depthIndex].Z()-this->database[depthIndex-1].Z());
      northCurrent =
        (this->database[depthIndex].X()-this->database[depthIndex-1].X())*rate
        + this->database[depthIndex-1].X();
      eastCurrent =
        (this->database[depthIndex].Y()-this->database[depthIndex-1].Y())*rate
        + this->database[depthIndex-1].Y();
  }
  // apply
  this->currentVelocity = this->currentVelocity
    + ignition::math::Vector3d(northCurrent, eastCurrent, 0.0);
}

