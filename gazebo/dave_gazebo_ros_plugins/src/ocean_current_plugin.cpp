// Copyright (c) 2016 The dave Simulator Authors.
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

/// \file ocean_current_plugin.cpp

#include <dave_gazebo_ros_plugins/ocean_current_plugin.h>
#include <dave_gazebo_world_plugins/gauss_markov_process.h>

#include <algorithm>
#include <string>

namespace dave_simulator_ros
{
/////////////////////////////////////////////////
UnderwaterCurrentROSPlugin::UnderwaterCurrentROSPlugin()
{
  this->rosPublishPeriod = gazebo::common::Time(0.05);
  this->lastRosPublishTime = gazebo::common::Time(0.0);
  this->db_path = ros::package::getPath("dave_worlds");
}

/////////////////////////////////////////////////
UnderwaterCurrentROSPlugin::~UnderwaterCurrentROSPlugin()
{
#if GAZEBO_MAJOR_VERSION >= 8
  this->rosPublishConnection.reset();
#else
  gazebo::event::Events::DisconnectWorldUpdateBegin(
    this->rosPublishConnection);
#endif
  this->rosNode->shutdown();
}

/////////////////////////////////////////////////
void UnderwaterCurrentROSPlugin::Load(gazebo::physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  try
  {
    UnderwaterCurrentPlugin::Load(_world, _sdf);
    this->stratifiedCurrentVelocityDatabaseTopic =
        this->stratifiedCurrentVelocityTopic + "_database";
  } catch(gazebo::common::Exception &_e)
  {
    gzerr << "Error loading plugin."
          << "Please ensure that your model is correct."
          << '\n';
    return;
  }

  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS has not been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  this->ns = "";
  if (_sdf->HasElement("namespace"))
    this->ns = _sdf->Get<std::string>("namespace");

  gzmsg << "UnderwaterCurrentROSPlugin::namespace=" << this->ns << std::endl;

  this->rosNode.reset(new ros::NodeHandle(this->ns));

  // Advertise the flow velocity as a stamped twist message
  this->flowVelocityPub = this->rosNode->advertise<geometry_msgs::TwistStamped>(
    this->currentVelocityTopic, 10);

  // Advertise the stratified ocean current msg
  this->stratifiedCurrentVelocityPub = this->rosNode->advertise<
    dave_gazebo_ros_plugins::StratifiedCurrentVelocity>(
    this->stratifiedCurrentVelocityTopic, 10);

  // Advertise the stratified ocean current msg
  this->stratifiedCurrentDatabasePub = this->rosNode->advertise<
    dave_gazebo_ros_plugins::StratifiedCurrentDatabase>(
    this->stratifiedCurrentVelocityDatabaseTopic, 10);

  // Advertise the service to update the current velocity model
  this->worldServices["set_current_velocity_model"] =
    this->rosNode->advertiseService(
      "set_current_velocity_model",
      &UnderwaterCurrentROSPlugin::UpdateCurrentVelocityModel, this);

  // Advertise the service to update the current velocity model
  this->worldServices["get_current_velocity_model"] =
    this->rosNode->advertiseService(
      "get_current_velocity_model",
      &UnderwaterCurrentROSPlugin::GetCurrentVelocityModel, this);

  // Advertise the service to update the current velocity model
  this->worldServices["set_current_horz_angle_model"] =
    this->rosNode->advertiseService(
      "set_current_horz_angle_model",
      &UnderwaterCurrentROSPlugin::UpdateCurrentHorzAngleModel, this);

  // Advertise the service to update the current velocity model
  this->worldServices["get_current_horz_angle_model"] =
    this->rosNode->advertiseService(
      "get_current_horz_angle_model",
      &UnderwaterCurrentROSPlugin::GetCurrentHorzAngleModel, this);

  // Advertise the service to update the current velocity model
  this->worldServices["set_current_vert_angle_model"] =
    this->rosNode->advertiseService(
      "set_current_vert_angle_model",
      &UnderwaterCurrentROSPlugin::UpdateCurrentVertAngleModel, this);

  // Advertise the service to update the current velocity model
  this->worldServices["get_current_vert_angle_model"] =
    this->rosNode->advertiseService(
      "get_current_vert_angle_model",
      &UnderwaterCurrentROSPlugin::GetCurrentHorzAngleModel, this);

  // Advertise the service to update the current velocity mean value
  this->worldServices["set_current_velocity"] =
    this->rosNode->advertiseService(
      "set_current_velocity",
      &UnderwaterCurrentROSPlugin::UpdateCurrentVelocity, this);

  // Advertise the service to update the current velocity mean value
  this->worldServices["set_stratified_current_velocity"] =
    this->rosNode->advertiseService(
      "set_stratified_current_velocity",
      &UnderwaterCurrentROSPlugin::UpdateStratCurrentVelocity, this);

  // Advertise the service to update the current velocity mean value
  this->worldServices["set_current_horz_angle"] =
    this->rosNode->advertiseService(
      "set_current_horz_angle",
      &UnderwaterCurrentROSPlugin::UpdateHorzAngle, this);

  // Advertise the service to update the current velocity mean value
  this->worldServices["set_stratified_current_horz_angle"] =
    this->rosNode->advertiseService(
      "set_stratified_current_horz_angle",
      &UnderwaterCurrentROSPlugin::UpdateStratHorzAngle, this);

  // Advertise the service to update the current velocity mean value
  this->worldServices["set_current_vert_angle"] =
    this->rosNode->advertiseService(
      "set_current_vert_angle",
      &UnderwaterCurrentROSPlugin::UpdateVertAngle, this);

  // Advertise the service to update the current velocity mean value
  this->worldServices["set_stratified_current_vert_angle"] =
    this->rosNode->advertiseService(
      "set_stratified_current_vert_angle",
      &UnderwaterCurrentROSPlugin::UpdateStratVertAngle, this);

  this->rosPublishConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(&UnderwaterCurrentROSPlugin::OnUpdateCurrentVel, this));
}

/////////////////////////////////////////////////
void UnderwaterCurrentROSPlugin::OnUpdateCurrentVel()
{
  if (this->lastUpdate - this->lastRosPublishTime >= this->rosPublishPeriod)
  {
    // Generate and publish current_velocity according to the vehicle depth
    this->lastRosPublishTime = this->lastUpdate;
    geometry_msgs::TwistStamped flowVelMsg;
    flowVelMsg.header.stamp = ros::Time().now();
    flowVelMsg.header.frame_id = "/world";

    flowVelMsg.twist.linear.x = this->currentVelocity.X();
    flowVelMsg.twist.linear.y = this->currentVelocity.Y();
    flowVelMsg.twist.linear.z = this->currentVelocity.Z();

    this->flowVelocityPub.publish(flowVelMsg);

    // Generate and publish stratified current velocity
    dave_gazebo_ros_plugins::StratifiedCurrentVelocity stratCurrentVelocityMsg;
    stratCurrentVelocityMsg.header.stamp = ros::Time().now();
    stratCurrentVelocityMsg.header.frame_id = "/world";
    for (int i = 0; i < this->currentStratifiedVelocity.size(); i++)
    {
        geometry_msgs::Vector3 velocity;
        velocity.x = this->currentStratifiedVelocity[i].X();
        velocity.y = this->currentStratifiedVelocity[i].Y();
        velocity.z = this->currentStratifiedVelocity[i].Z();
        stratCurrentVelocityMsg.velocities.push_back(velocity);
        stratCurrentVelocityMsg.depths.push_back(
          this->currentStratifiedVelocity[i].W());
    }
    this->stratifiedCurrentVelocityPub.publish(stratCurrentVelocityMsg);

    // Generate and publish stratified current database
    dave_gazebo_ros_plugins::StratifiedCurrentDatabase currentDatabaseMsg;
    for (int i = 0; i < this->stratifiedDatabase.size(); i++)
    {
      // Stratified current database
      geometry_msgs::Vector3 velocity;
      velocity.x = this->stratifiedDatabase[i].X();
      velocity.y = this->stratifiedDatabase[i].Y();
      velocity.z = 0.0;
      currentDatabaseMsg.velocities.push_back(velocity);
      currentDatabaseMsg.depths.push_back(this->stratifiedDatabase[i].Z());
    }

    if (this->tidalHarmonicFlag)
    {
      // Tidal harmonic constituents
      currentDatabaseMsg.M2amp = this->M2_amp;
      currentDatabaseMsg.M2phase = this->M2_phase;
      currentDatabaseMsg.M2speed = this->M2_speed;
      currentDatabaseMsg.S2amp = this->S2_amp;
      currentDatabaseMsg.S2phase = this->S2_phase;
      currentDatabaseMsg.S2speed = this->S2_speed;
      currentDatabaseMsg.N2amp = this->N2_amp;
      currentDatabaseMsg.N2phase = this->N2_phase;
      currentDatabaseMsg.N2speed = this->N2_speed;
      currentDatabaseMsg.tideConstituents = true;
    }
    else
    {
      for (int i = 0; i < this->dateGMT.size(); i++)
      {
        // Tidal oscillation database
        currentDatabaseMsg.timeGMTYear.push_back(this->dateGMT[i][0]);
        currentDatabaseMsg.timeGMTMonth.push_back(this->dateGMT[i][1]);
        currentDatabaseMsg.timeGMTDay.push_back(this->dateGMT[i][2]);
        currentDatabaseMsg.timeGMTHour.push_back(this->dateGMT[i][3]);
        currentDatabaseMsg.timeGMTMinute.push_back(this->dateGMT[i][4]);

        currentDatabaseMsg.tideVelocities.push_back(this->speedcmsec[i]);
      }
      currentDatabaseMsg.tideConstituents = false;
    }

    currentDatabaseMsg.ebbDirection = this->ebbDirection;
    currentDatabaseMsg.floodDirection = this->floodDirection;

    currentDatabaseMsg.worldStartTimeYear = this->world_start_time_year;
    currentDatabaseMsg.worldStartTimeMonth = this->world_start_time_month;
    currentDatabaseMsg.worldStartTimeDay = this->world_start_time_day;
    currentDatabaseMsg.worldStartTimeHour = this->world_start_time_hour;
    currentDatabaseMsg.worldStartTimeMinute = this->world_start_time_minute;

    this->stratifiedCurrentDatabasePub.publish(currentDatabaseMsg);
  }
}

/////////////////////////////////////////////////
bool UnderwaterCurrentROSPlugin::UpdateHorzAngle(
    dave_gazebo_ros_plugins::SetCurrentDirection::Request& _req,
    dave_gazebo_ros_plugins::SetCurrentDirection::Response& _res)
{
  _res.success = this->currentHorzAngleModel.SetMean(_req.angle);

  return true;
}

/////////////////////////////////////////////////
bool UnderwaterCurrentROSPlugin::UpdateStratHorzAngle(
    dave_gazebo_ros_plugins::SetStratifiedCurrentDirection::Request& _req,
    dave_gazebo_ros_plugins::SetStratifiedCurrentDirection::Response& _res)
{
  if (_req.layer >= this->stratifiedDatabase.size())
  {
      _res.success = false;
      return true;
  }
  _res.success =
    this->stratifiedCurrentModels[_req.layer][1].SetMean(_req.angle);
  if (_res.success)
  {
      // Update the database values (new angle, unchanged velocity)
      double velocity = hypot(this->stratifiedDatabase[_req.layer].X(),
                              this->stratifiedDatabase[_req.layer].Y());
      this->stratifiedDatabase[_req.layer].X() = cos(_req.angle) *
                                                     velocity;
      this->stratifiedDatabase[_req.layer].Y() = sin(_req.angle) *
                                                     velocity;
  }
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterCurrentROSPlugin::UpdateVertAngle(
    dave_gazebo_ros_plugins::SetCurrentDirection::Request& _req,
    dave_gazebo_ros_plugins::SetCurrentDirection::Response& _res)
{
  _res.success = this->currentVertAngleModel.SetMean(_req.angle);
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterCurrentROSPlugin::UpdateStratVertAngle(
    dave_gazebo_ros_plugins::SetStratifiedCurrentDirection::Request& _req,
    dave_gazebo_ros_plugins::SetStratifiedCurrentDirection::Response& _res)
{
  if (_req.layer >= this->stratifiedDatabase.size())
  {
      _res.success = false;
      return true;
  }
  _res.success =
    this->stratifiedCurrentModels[_req.layer][2].SetMean(_req.angle);

  return true;
}

/////////////////////////////////////////////////
bool UnderwaterCurrentROSPlugin::UpdateCurrentVelocity(
    dave_gazebo_ros_plugins::SetCurrentVelocity::Request& _req,
    dave_gazebo_ros_plugins::SetCurrentVelocity::Response& _res)
{
  if (this->currentVelModel.SetMean(_req.velocity) &&
      this->currentHorzAngleModel.SetMean(_req.horizontal_angle) &&
      this->currentVertAngleModel.SetMean(_req.vertical_angle))
  {
    gzmsg << "Current velocity [m/s] = " << _req.velocity << std::endl
      << "Current horizontal angle [rad] = " << _req.horizontal_angle
      << std::endl
      << "Current vertical angle [rad] = " << _req.vertical_angle
      << std::endl
      << "\tWARNING: Current velocity calculated in the ENU frame"
      << std::endl;
    _res.success = true;
  }
  else
  {
    gzmsg << "Error while updating the current velocity" << std::endl;
    _res.success = false;
  }
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterCurrentROSPlugin::UpdateStratCurrentVelocity(
    dave_gazebo_ros_plugins::SetStratifiedCurrentVelocity::Request& _req,
    dave_gazebo_ros_plugins::SetStratifiedCurrentVelocity::Response& _res)
{
  if (_req.layer >= this->stratifiedDatabase.size())
  {
      _res.success = false;
      return true;
  }
  if (this->stratifiedCurrentModels[_req.layer][0].SetMean(_req.velocity) &&
      this->stratifiedCurrentModels[_req.layer][1].SetMean(
        _req.horizontal_angle) &&
      this->stratifiedCurrentModels[_req.layer][2].SetMean(
        _req.vertical_angle))
  {
    // Update the database values as well
    this->stratifiedDatabase[_req.layer].X() = cos(_req.horizontal_angle) *
                                               _req.velocity;
    this->stratifiedDatabase[_req.layer].Y() = sin(_req.horizontal_angle) *
                                               _req.velocity;
    gzmsg << "Layer " << _req.layer << " current velocity [m/s] = "
          << _req.velocity << std::endl
      << "  Horizontal angle [rad] = " << _req.horizontal_angle
      << std::endl
      << "  Vertical angle [rad] = " << _req.vertical_angle
      << std::endl
      << "\tWARNING: Current velocity calculated in the ENU frame"
      << std::endl;
    _res.success = true;
  }
  else
  {
    gzmsg << "Error while updating the current velocity" << std::endl;
    _res.success = false;
  }
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterCurrentROSPlugin::GetCurrentVelocityModel(
    dave_gazebo_ros_plugins::GetCurrentModel::Request& _req,
    dave_gazebo_ros_plugins::GetCurrentModel::Response& _res)
{
  _res.mean = this->currentVelModel.mean;
  _res.min = this->currentVelModel.min;
  _res.max = this->currentVelModel.max;
  _res.noise = this->currentVelModel.noiseAmp;
  _res.mu = this->currentVelModel.mu;
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterCurrentROSPlugin::GetCurrentHorzAngleModel(
    dave_gazebo_ros_plugins::GetCurrentModel::Request& _req,
    dave_gazebo_ros_plugins::GetCurrentModel::Response& _res)
{
  _res.mean = this->currentHorzAngleModel.mean;
  _res.min = this->currentHorzAngleModel.min;
  _res.max = this->currentHorzAngleModel.max;
  _res.noise = this->currentHorzAngleModel.noiseAmp;
  _res.mu = this->currentHorzAngleModel.mu;
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterCurrentROSPlugin::GetCurrentVertAngleModel(
    dave_gazebo_ros_plugins::GetCurrentModel::Request& _req,
    dave_gazebo_ros_plugins::GetCurrentModel::Response& _res)
{
  _res.mean = this->currentVertAngleModel.mean;
  _res.min = this->currentVertAngleModel.min;
  _res.max = this->currentVertAngleModel.max;
  _res.noise = this->currentVertAngleModel.noiseAmp;
  _res.mu = this->currentVertAngleModel.mu;
  return true;
}


/////////////////////////////////////////////////
bool UnderwaterCurrentROSPlugin::UpdateCurrentVelocityModel(
    dave_gazebo_ros_plugins::SetCurrentModel::Request& _req,
    dave_gazebo_ros_plugins::SetCurrentModel::Response& _res)
{
  _res.success = this->currentVelModel.SetModel(
    std::max(0.0, _req.mean),
    std::min(0.0, _req.min),
    std::max(0.0, _req.max),
    _req.mu,
    _req.noise);

  for (int i = 0; i < this->stratifiedCurrentModels.size(); i++)
  {
    gazebo::GaussMarkovProcess model = this->stratifiedCurrentModels[i][0];
    model.SetModel(
      model.mean,
      std::max(0.0, _req.min),
      std::max(0.0, _req.max),
      _req.mu,
      _req.noise);
  }
  gzmsg << "Current velocity model updated" << std::endl
    << "\tWARNING: Current velocity calculated in the ENU frame"
    << std::endl;
  this->currentVelModel.Print();
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterCurrentROSPlugin::UpdateCurrentHorzAngleModel(
    dave_gazebo_ros_plugins::SetCurrentModel::Request& _req,
    dave_gazebo_ros_plugins::SetCurrentModel::Response& _res)
{
  _res.success = this->currentHorzAngleModel.SetModel(_req.mean, _req.min,
    _req.max, _req.mu, _req.noise);
  for (int i = 0; i < this->stratifiedCurrentModels.size(); i++)
  {
    gazebo::GaussMarkovProcess model = this->stratifiedCurrentModels[i][1];
    model.SetModel(model.mean,
      std::max(-M_PI, _req.min),
      std::min(M_PI, _req.max),
      _req.mu,
      _req.noise);
  }
  gzmsg << "Horizontal angle model updated" << std::endl
    << "\tWARNING: Current velocity calculated in the ENU frame"
    << std::endl;
  this->currentHorzAngleModel.Print();
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterCurrentROSPlugin::UpdateCurrentVertAngleModel(
    dave_gazebo_ros_plugins::SetCurrentModel::Request& _req,
    dave_gazebo_ros_plugins::SetCurrentModel::Response& _res)
{
  _res.success = this->currentVertAngleModel.SetModel(_req.mean, _req.min,
    _req.max, _req.mu, _req.noise);
  gzmsg << "Vertical angle model updated" << std::endl
    << "\tWARNING: Current velocity calculated in the ENU frame"
    << std::endl;
  this->currentVertAngleModel.Print();
  return true;
}

/////////////////////////////////////////////////
GZ_REGISTER_WORLD_PLUGIN(UnderwaterCurrentROSPlugin)
}
