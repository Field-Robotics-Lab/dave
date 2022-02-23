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

/// \file ocean_current_model_plugin.cc

#include <dave_gazebo_model_plugins/ocean_current_model_plugin.h>

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

#include "ros/package.h"

namespace gazebo {

/////////////////////////////////////////////////
TransientCurrentPlugin::TransientCurrentPlugin()
{
  this->rosPublishPeriod = gazebo::common::Time(0.05);
  this->lastRosPublishTime = gazebo::common::Time(0.0);
}

/////////////////////////////////////////////////
TransientCurrentPlugin::~TransientCurrentPlugin()
{
  #if GAZEBO_MAJOR_VERSION >= 8
    this->updateConnection.reset();
  #else
    gazebo::event::Events::DisconnectWorldUpdateBegin(
      this->rosPublishConnection);
  #endif
    this->rosNode->shutdown();
}

/////////////////////////////////////////////////
void TransientCurrentPlugin::Load(
  physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model != NULL, "Model pointer is invalid");
  GZ_ASSERT(_sdf != NULL, "SDF pointer is invalid");

  this->model = _model;
  this->world = _model->GetWorld();
  this->sdf = _sdf;

  // Read the namespace for topics and services
  this->ns = _sdf->Get<std::string>("namespace");

  gzmsg << "Loading transient ocean current model plugin..." << std::endl;

  // Read the vehicle flow velocity topic name from the SDF file
  if (_sdf->HasElement("flow_velocity_topic"))
    this->currentVelocityTopic = _sdf->Get<std::string>("flow_velocity_topic");
  else
  {
    this->currentVelocityTopic =
      "hydrodynamics/current_velocity/" + this->model->GetName();
    gzerr << "Empty flow_velocity_topic for transient_current model plugin. "
       << "Default topicName definition is used" << std::endl;
  }
  gzmsg << "Transient velocity topic name for " << this->model->GetName()
        << " : " << this->currentVelocityTopic << std::endl;

  // Initializing the ROS node
  this->rosNode.reset(new ros::NodeHandle(this->ns));

  // Advertise the ROS flow velocity as a stamped twist message
  this->flowVelocityPub = this->rosNode->advertise<geometry_msgs::TwistStamped>(
    this->currentVelocityTopic, 10);

  // Initializing the Gazebo transport node
  this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION >= 8
  this->node->Init(this->world->Name());
#else
  this->node->Init(this->world->GetName());
#endif

  // Advertise the Gazebo current velocity topic
  this->publishers[this->currentVelocityTopic] =
    this->node->Advertise<msgs::Vector3d>(this->currentVelocityTopic);

  // Read topic name of stratified ocean current from SDF
  if (_sdf->HasElement("transient_current"))
  {
    sdf::ElementPtr currentVelocityParams = _sdf->GetElement(
      "transient_current");
    if (currentVelocityParams->HasElement("topic_stratified"))
      this->transientCurrentVelocityTopic =
        "/hydrodynamics/" +
        currentVelocityParams->Get<std::string>("topic_stratified") +
        "_database";
    else
    {
      this->transientCurrentVelocityTopic =
        "/hydrodynamics/stratified_current_velocity_database";
    }

    // Read Gauss-Markov parameters
    if (currentVelocityParams->HasElement("velocity_north"))
    {
      sdf::ElementPtr elem =
        currentVelocityParams->GetElement("velocity_north");
      if (elem->HasElement("mean"))
          this->currentVelNorthModel.mean = 0.0;
      if (elem->HasElement("mu"))
          this->currentVelNorthModel.mu = 0.0;
      if (elem->HasElement("noiseAmp"))
          this->noiseAmp_North = elem->Get<double>("noiseAmp");
      this->currentVelNorthModel.min =
        this->currentVelNorthModel.mean - this->noiseAmp_North;
      this->currentVelNorthModel.max =
        this->currentVelNorthModel.mean + this->noiseAmp_North;
      if (elem->HasElement("noiseFreq"))
          this->noiseFreq_North = elem->Get<double>("noiseFreq");
      this->currentVelNorthModel.noiseAmp = this->noiseFreq_North;
    }
    this->currentVelNorthModel.var = this->currentVelNorthModel.mean;
    gzmsg << "For vehicle " << this->model->GetName()
          << " -> Current north-direction velocity [m/s] "
          << "Gauss-Markov process model:" << std::endl;
    this->currentVelNorthModel.Print();

    if (currentVelocityParams->HasElement("velocity_east"))
    {
      sdf::ElementPtr elem = currentVelocityParams->GetElement("velocity_east");
      if (elem->HasElement("mean"))
          this->currentVelEastModel.mean = 0.0;
      if (elem->HasElement("mu"))
          this->currentVelEastModel.mu = 0.0;
      if (elem->HasElement("noiseAmp"))
          this->noiseAmp_East = elem->Get<double>("noiseAmp");
      this->currentVelEastModel.min =
        this->currentVelEastModel.mean - this->noiseAmp_East;
      this->currentVelEastModel.max =
        this->currentVelEastModel.mean + this->noiseAmp_East;
      if (elem->HasElement("noiseFreq"))
          this->noiseFreq_East = elem->Get<double>("noiseFreq");
      this->currentVelEastModel.noiseAmp = this->noiseFreq_East;
    }
    this->currentVelEastModel.var = this->currentVelEastModel.mean;
    gzmsg << "For vehicle " << this->model->GetName()
          << " -> Current east-direction velocity [m/s] "
          << "Gauss-Markov process model:" << std::endl;
    this->currentVelEastModel.Print();

    if (currentVelocityParams->HasElement("velocity_down"))
    {
      sdf::ElementPtr elem = currentVelocityParams->GetElement("velocity_down");
      if (elem->HasElement("mean"))
          this->currentVelDownModel.mean = 0.0;
      if (elem->HasElement("mu"))
          this->currentVelDownModel.mu = 0.0;
      if (elem->HasElement("noiseAmp"))
          this->noiseAmp_Down = elem->Get<double>("noiseAmp");
      this->currentVelDownModel.min =
        this->currentVelDownModel.mean - this->noiseAmp_Down;
      this->currentVelDownModel.max =
        this->currentVelDownModel.mean + this->noiseAmp_Down;
      if (elem->HasElement("noiseFreq"))
          this->noiseFreq_Down = elem->Get<double>("noiseFreq");
      this->currentVelDownModel.noiseAmp = this->noiseFreq_Down;
    }
    this->currentVelDownModel.var = this->currentVelDownModel.mean;
    gzmsg << "For vehicle " << this->model->GetName()
          << " -> Current down-direction velocity [m/s]"
          << "Gauss-Markov process model:" << std::endl;
    this->currentVelDownModel.Print();

    // Initialize the time update
    #if GAZEBO_MAJOR_VERSION >= 8
      this->lastUpdate = this->world->SimTime();
    #else
      this->lastUpdate = this->world->GetSimTime();
    #endif
    this->currentVelNorthModel.lastUpdate = this->lastUpdate.Double();
    this->currentVelEastModel.lastUpdate = this->lastUpdate.Double();
    this->currentVelDownModel.lastUpdate = this->lastUpdate.Double();
  }

  // Tidal Oscillation
  if (this->sdf->HasElement("tide_oscillation")
    && this->sdf->Get<bool>("tide_oscillation") == true)
    this->tideFlag = true;
  else
    this->tideFlag = false;

  // Subscribe stratified ocean current database
  this->databaseSub = this->rosNode->subscribe
    <dave_gazebo_ros_plugins::StratifiedCurrentDatabase>
    (this->transientCurrentVelocityTopic, 10,
    boost::bind(&TransientCurrentPlugin::UpdateDatabase, this, _1));

  // Connect the update event callback for ROS and ocean current calculation
  this->Connect();

  gzmsg << "Transient current model plugin loaded!" << std::endl;

  this->lastDepthIndex = 0;
}

/////////////////////////////////////////////////
void TransientCurrentPlugin::Init()
{
  // Doing nothing for now
}

/////////////////////////////////////////////////
void TransientCurrentPlugin::Connect()
{
  // Connect the update event
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&TransientCurrentPlugin::Update,
                  this, _1));
}

/////////////////////////////////////////////////
void TransientCurrentPlugin::Update(const gazebo::common::UpdateInfo &)
{
  // Update time
  this->lastUpdate = this->model->GetWorld()->SimTime();

  this->CalculateOceanCurrent();

  this->PublishCurrentVelocity();
}

/////////////////////////////////////////////////
void TransientCurrentPlugin::UpdateDatabase(
  const dave_gazebo_ros_plugins::StratifiedCurrentDatabase::ConstPtr &_msg)
{
    this->lock_.lock();

    this->database.clear();
    for (int i = 0; i < _msg->depths.size(); i++)
    {
      ignition::math::Vector3d data(_msg->velocities[i].x,
                                    _msg->velocities[i].y,
                                    _msg->depths[i]);
      this->database.push_back(data);
    }
    if (this->tideFlag)
    {
      this->timeGMT.clear();
      this->tideVelocities.clear();
      if (_msg->tideConstituents == true)
      {
        this->M2_amp = _msg->M2amp;
        this->M2_phase = _msg->M2phase;
        this->M2_speed = _msg->M2speed;
        this->S2_amp = _msg->S2amp;
        this->S2_phase = _msg->S2phase;
        this->S2_speed = _msg->S2speed;
        this->N2_amp = _msg->N2amp;
        this->N2_phase = _msg->N2phase;
        this->N2_speed = _msg->N2speed;
        this->tide_Constituents = true;
      }
      else
      {
        std::array<int, 5> tmpDateVals;
        for (int i = 0; i < _msg->timeGMTYear.size(); i++)
        {
          tmpDateVals[0] = _msg->timeGMTYear[i];
          tmpDateVals[1] = _msg->timeGMTMonth[i];
          tmpDateVals[2] = _msg->timeGMTDay[i];
          tmpDateVals[3] = _msg->timeGMTHour[i];
          tmpDateVals[4] = _msg->timeGMTMinute[i];

          this->timeGMT.push_back(tmpDateVals);
          this->tideVelocities.push_back(_msg->tideVelocities[i]);
        }
        this->tide_Constituents = false;
      }
      this->ebbDirection = _msg->ebbDirection;
      this->floodDirection = _msg->floodDirection;
      this->world_start_time[0] = _msg->worldStartTimeYear;
      this->world_start_time[1] = _msg->worldStartTimeMonth;
      this->world_start_time[2] = _msg->worldStartTimeDay;
      this->world_start_time[3] = _msg->worldStartTimeHour;
      this->world_start_time[4] = _msg->worldStartTimeMinute;
    }

    this->lock_.unlock();
}

/////////////////////////////////////////////////
void TransientCurrentPlugin::CalculateOceanCurrent()
{
  this->lock_.lock();

  // Update vehicle position
  double vehicleDepth = - this->model->WorldPose().Pos().Z();

  if (this->database.size() == 0)
  {
    // skip for next time (wating for valid database subscrition)
  }
  else
  {
    double northCurrent = 0.0;
    double eastCurrent = 0.0;

    //--- Interpolate velocity from database ---//
    // find current depth index from database
    // (X: north-direction, Y: east-direction, Z: depth)
    int depthIndex = 0;
    for (int i = 1; i < this->database.size(); i++)
    {
      if (this->database[i].Z() > vehicleDepth)
      {
        depthIndex = i;
        break;
      }
    }

    // If sudden change found, use the one before
    if (this->lastDepthIndex == 0)
      this->lastDepthIndex = depthIndex;
    else
    {
      if (abs(depthIndex - this->lastDepthIndex) > 2)
        depthIndex = this->lastDepthIndex;
      this->lastDepthIndex = depthIndex;
    }

    // interpolate
    if (depthIndex == 0)
    {  // Deeper than database use deepest value
      northCurrent =
        this->database[this->database.size()-1].X();
      eastCurrent =
        this->database[this->database.size()-1].Y();
    }
    else
    {
      double rate =
        (vehicleDepth-this->database[depthIndex-1].Z())
        /(this->database[depthIndex].Z()-this->database[depthIndex-1].Z());
      northCurrent =
        (this->database[depthIndex].X()-this->database[depthIndex-1].X())*rate
        + this->database[depthIndex-1].X();
      eastCurrent =
        (this->database[depthIndex].Y()-this->database[depthIndex-1].Y())*rate
        + this->database[depthIndex-1].Y();
    }
    this->currentVelNorthModel.mean = northCurrent;
    this->currentVelEastModel.mean = eastCurrent;
    this->currentVelDownModel.mean = 0.0;

    // Tidal oscillation
    if (this->tideFlag)
    {
      // Update tide oscillation
    #if GAZEBO_MAJOR_VERSION >= 8
      common::Time time = this->world->SimTime();
    #else
      common::Time time = this->world->GetSimTime();
    #endif
      if (this->tide_Constituents)
      {
        this->tide.M2_amp = this->M2_amp;
        this->tide.M2_phase = this->M2_phase;
        this->tide.M2_speed = this->M2_speed;
        this->tide.S2_amp = this->S2_amp;
        this->tide.S2_phase = this->S2_phase;
        this->tide.S2_speed = this->S2_speed;
        this->tide.N2_amp = this->N2_amp;
        this->tide.N2_phase = this->N2_phase;
        this->tide.N2_speed = this->N2_speed;
      }
      else
      {
        this->tide.dateGMT = this->timeGMT;
        this->tide.speedcmsec = this->tideVelocities;
      }
      this->tide.ebbDirection = this->ebbDirection;
      this->tide.floodDirection = this->floodDirection;
      this->tide.worldStartTime = this->world_start_time;
      this->tide.Initiate(this->tide_Constituents);
      std::pair<double, double> currents =
        this->tide.Update(time.Double(), northCurrent);
      this->currentVelNorthModel.mean = currents.first;
      this->currentVelEastModel.mean = currents.second;
      this->currentVelDownModel.mean = 0.0;
    }
    else
    {
      this->currentVelNorthModel.mean = northCurrent;
      this->currentVelEastModel.mean = eastCurrent;
      this->currentVelDownModel.mean = 0.0;
    }

    // Change min max accordingly
    currentVelNorthModel.max = currentVelNorthModel.mean + this->noiseAmp_North;
    currentVelNorthModel.min = currentVelNorthModel.mean - this->noiseAmp_North;
    currentVelEastModel.max = currentVelEastModel.mean + this->noiseAmp_East;
    currentVelEastModel.min = currentVelEastModel.mean - this->noiseAmp_East;
    currentVelDownModel.max = currentVelDownModel.mean + this->noiseAmp_Down;
    currentVelDownModel.min = currentVelDownModel.mean - this->noiseAmp_Down;

    // Assing values to the model
    this->currentVelNorthModel.var = this->currentVelNorthModel.mean;
    this->currentVelEastModel.var = this->currentVelEastModel.mean;
    this->currentVelDownModel.var = this->currentVelDownModel.mean;

    // Update time
    #if GAZEBO_MAJOR_VERSION >= 8
      common::Time time = this->world->SimTime();
    #else
      common::Time time = this->world->GetSimTime();
    #endif

    // Update current velocity
    double velocityNorth = this->currentVelNorthModel.Update(time.Double());

    // Update current horizontal direction around z axis of flow frame
    double velocityEast = this->currentVelEastModel.Update(time.Double());

    // Update current horizontal direction around z axis of flow frame
    double velocityDown = this->currentVelDownModel.Update(time.Double());

    // Update current Velocity
    this->currentVelocity =
      ignition::math::Vector3d(velocityNorth, velocityEast, velocityDown);

    // Update time stamp
    this->lastUpdate = time;
  }

  this->lock_.unlock();
}

/////////////////////////////////////////////////
void TransientCurrentPlugin::PublishCurrentVelocity()
{
  // Generate and publish ROS topic according to the vehicle depth
  if (this->lastUpdate - this->lastRosPublishTime >= this->rosPublishPeriod)
  {
    this->lastRosPublishTime = this->lastUpdate;
    geometry_msgs::TwistStamped flowVelMsg;
    flowVelMsg.header.stamp = ros::Time().now();
    flowVelMsg.header.frame_id = "/world";
    flowVelMsg.twist.linear.x = this->currentVelocity.X();
    flowVelMsg.twist.linear.y = this->currentVelocity.Y();
    flowVelMsg.twist.linear.z = this->currentVelocity.Z();
    this->flowVelocityPub.publish(flowVelMsg);
  }

  // Generate and publish Gazebo topic according to the vehicle depth
  msgs::Vector3d currentVel;
  msgs::Set(&currentVel, ignition::math::Vector3d(this->currentVelocity.X(),
                                                  this->currentVelocity.Y(),
                                                  this->currentVelocity.Z()));
  this->publishers[this->currentVelocityTopic]->Publish(currentVel);
}

GZ_REGISTER_MODEL_PLUGIN(TransientCurrentPlugin)
}
