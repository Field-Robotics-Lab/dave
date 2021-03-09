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

#include <dave_world_ros_plugins/UnderwaterCurrentROSPlugin.hh>

namespace dave_simulator_ros
{
/////////////////////////////////////////////////
UnderwaterCurrentROSPlugin::UnderwaterCurrentROSPlugin()
{
  this->rosPublishPeriod = gazebo::common::Time(0.05);
  this->lastRosPublishTime = gazebo::common::Time(0.0);
  this->db_path = ros::package::getPath("uuv_dave");
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
    dave_world_ros_plugins_msgs::StratifiedCurrentVelocity>(
    this->stratifiedCurrentVelocityTopic, 10);

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
  this->worldServices["set_current_horz_angle"] =
    this->rosNode->advertiseService(
      "set_current_horz_angle",
      &UnderwaterCurrentROSPlugin::UpdateHorzAngle, this);

  // Advertise the service to update the current velocity mean value
  this->worldServices["set_current_vert_angle"] =
    this->rosNode->advertiseService(
      "set_current_vert_angle",
      &UnderwaterCurrentROSPlugin::UpdateVertAngle, this);

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

    // Generate and publish stratified_current_velocity database
    dave_world_ros_plugins_msgs::StratifiedCurrentVelocity currentDatabaseMsg;
    for (int i = 0; i < this->database.size(); i++) {
      geometry_msgs::Vector3 velocity;
      velocity.x = this->database[i].X();
      velocity.y = this->database[i].Y();
      velocity.z = 0.0;
      currentDatabaseMsg.velocities.push_back(velocity);
      currentDatabaseMsg.depths.push_back(this->database[i].Z());
    }

    this->stratifiedCurrentVelocityPub.publish(currentDatabaseMsg);
  }
}

/////////////////////////////////////////////////
bool UnderwaterCurrentROSPlugin::UpdateHorzAngle(
    dave_world_ros_plugins_msgs::SetCurrentDirection::Request& _req,
    dave_world_ros_plugins_msgs::SetCurrentDirection::Response& _res)
{
  _res.success = this->currentHorzAngleModel.SetMean(_req.angle);

  return true;
}

/////////////////////////////////////////////////
bool UnderwaterCurrentROSPlugin::UpdateVertAngle(
    dave_world_ros_plugins_msgs::SetCurrentDirection::Request& _req,
    dave_world_ros_plugins_msgs::SetCurrentDirection::Response& _res)
{
  _res.success = this->currentVertAngleModel.SetMean(_req.angle);
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterCurrentROSPlugin::UpdateCurrentVelocity(
    dave_world_ros_plugins_msgs::SetCurrentVelocity::Request& _req,
    dave_world_ros_plugins_msgs::SetCurrentVelocity::Response& _res)
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
bool UnderwaterCurrentROSPlugin::GetCurrentVelocityModel(
    dave_world_ros_plugins_msgs::GetCurrentModel::Request& _req,
    dave_world_ros_plugins_msgs::GetCurrentModel::Response& _res)
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
    dave_world_ros_plugins_msgs::GetCurrentModel::Request& _req,
    dave_world_ros_plugins_msgs::GetCurrentModel::Response& _res)
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
    dave_world_ros_plugins_msgs::GetCurrentModel::Request& _req,
    dave_world_ros_plugins_msgs::GetCurrentModel::Response& _res)
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
    dave_world_ros_plugins_msgs::SetCurrentModel::Request& _req,
    dave_world_ros_plugins_msgs::SetCurrentModel::Response& _res)
{
  _res.success = this->currentVelModel.SetModel(
    std::max(0.0, _req.mean),
    std::max(0.0, _req.min),
    std::max(0.0, _req.max),
    _req.mu,
    _req.noise);
  gzmsg << "Current velocity model updated" << std::endl
    << "\tWARNING: Current velocity calculated in the ENU frame"
    << std::endl;
  this->currentVelModel.Print();
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterCurrentROSPlugin::UpdateCurrentHorzAngleModel(
    dave_world_ros_plugins_msgs::SetCurrentModel::Request& _req,
    dave_world_ros_plugins_msgs::SetCurrentModel::Response& _res)
{
  _res.success = this->currentHorzAngleModel.SetModel(_req.mean, _req.min,
    _req.max, _req.mu, _req.noise);
  gzmsg << "Horizontal angle model updated" << std::endl
    << "\tWARNING: Current velocity calculated in the ENU frame"
    << std::endl;
  this->currentHorzAngleModel.Print();
  return true;
}

/////////////////////////////////////////////////
bool UnderwaterCurrentROSPlugin::UpdateCurrentVertAngleModel(
    dave_world_ros_plugins_msgs::SetCurrentModel::Request& _req,
    dave_world_ros_plugins_msgs::SetCurrentModel::Response& _res)
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
