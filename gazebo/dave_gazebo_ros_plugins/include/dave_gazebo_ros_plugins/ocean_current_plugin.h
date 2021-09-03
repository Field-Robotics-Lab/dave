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

/// \file ocean_current_plugin.hh
/// \brief Publishes the ocean current velocity in ROS messages and creates a
/// service to alter the flow model in runtime

#ifndef OCEAN_CURRENT_PLUGIN_H_
#define OCEAN_CURRENT_PLUGIN_H_

#include <dave_gazebo_world_plugins/ocean_current_world_plugin.h>
#include <dave_gazebo_ros_plugins/SetCurrentModel.h>
#include <dave_gazebo_ros_plugins/GetCurrentModel.h>
#include <dave_gazebo_ros_plugins/SetCurrentVelocity.h>
#include <dave_gazebo_ros_plugins/SetStratifiedCurrentVelocity.h>
#include <dave_gazebo_ros_plugins/SetCurrentDirection.h>
#include <dave_gazebo_ros_plugins/SetStratifiedCurrentDirection.h>
#include <dave_gazebo_ros_plugins/SetOriginSphericalCoord.h>
#include <dave_gazebo_ros_plugins/GetOriginSphericalCoord.h>
#include <dave_gazebo_ros_plugins/StratifiedCurrentVelocity.h>
#include <dave_gazebo_ros_plugins/StratifiedCurrentDatabase.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>

#include <boost/shared_ptr.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>

#include <map>
#include <string>


namespace dave_simulator_ros
{
  class UnderwaterCurrentROSPlugin : public gazebo::UnderwaterCurrentPlugin
  {
    /// \brief Class constructor
    public: UnderwaterCurrentROSPlugin();

    /// \brief Class destructor
    public: virtual ~UnderwaterCurrentROSPlugin();

    /// \brief Load module and read parameters from SDF.
    public: void Load(gazebo::physics::WorldPtr _world,
        sdf::ElementPtr _sdf);

    /// \brief Service call to update the parameters for the velocity
    /// Gauss-Markov process model
    public: bool UpdateCurrentVelocityModel(
        dave_gazebo_ros_plugins::SetCurrentModel::Request& _req,
        dave_gazebo_ros_plugins::SetCurrentModel::Response& _res);

    /// \brief Service call to update the parameters for the horizontal angle
    /// Gauss-Markov process model
    public: bool UpdateCurrentHorzAngleModel(
        dave_gazebo_ros_plugins::SetCurrentModel::Request& _req,
        dave_gazebo_ros_plugins::SetCurrentModel::Response& _res);

    /// \brief Service call to update the parameters for the vertical angle
    /// Gauss-Markov process model
    public: bool UpdateCurrentVertAngleModel(
        dave_gazebo_ros_plugins::SetCurrentModel::Request& _req,
        dave_gazebo_ros_plugins::SetCurrentModel::Response& _res);

    /// \brief Service call to read the parameters for the velocity
    /// Gauss-Markov process model
    public: bool GetCurrentVelocityModel(
        dave_gazebo_ros_plugins::GetCurrentModel::Request& _req,
        dave_gazebo_ros_plugins::GetCurrentModel::Response& _res);

    /// \brief Service call to read the parameters for the horizontal angle
    /// Gauss-Markov process model
    public: bool GetCurrentHorzAngleModel(
        dave_gazebo_ros_plugins::GetCurrentModel::Request& _req,
        dave_gazebo_ros_plugins::GetCurrentModel::Response& _res);

    /// \brief Service call to read the parameters for the vertical angle
    /// Gauss-Markov process model
    public: bool GetCurrentVertAngleModel(
        dave_gazebo_ros_plugins::GetCurrentModel::Request& _req,
        dave_gazebo_ros_plugins::GetCurrentModel::Response& _res);

    /// \brief Service call to update the mean value of the flow velocity
    public: bool UpdateCurrentVelocity(
        dave_gazebo_ros_plugins::SetCurrentVelocity::Request& _req,
        dave_gazebo_ros_plugins::SetCurrentVelocity::Response& _res);

    /// \brief Service call to update the mean value of the flow velocity
    public: bool UpdateStratCurrentVelocity(
        dave_gazebo_ros_plugins::SetStratifiedCurrentVelocity::Request& _req,
        dave_gazebo_ros_plugins::SetStratifiedCurrentVelocity::Response& _res);

    /// \brief Service call to update the mean value of the horizontal angle
    public: bool UpdateHorzAngle(
        dave_gazebo_ros_plugins::SetCurrentDirection::Request& _req,
        dave_gazebo_ros_plugins::SetCurrentDirection::Response& _res);

    /// \brief Service update to a stratified crnt horizontal angle mean value
    public: bool UpdateStratHorzAngle(
        dave_gazebo_ros_plugins::SetStratifiedCurrentDirection::Request& _req,
        dave_gazebo_ros_plugins::SetStratifiedCurrentDirection::Response& _res);

    /// \brief Service call to update the mean value of the vertical angle
    public: bool UpdateVertAngle(
        dave_gazebo_ros_plugins::SetCurrentDirection::Request& _req,
        dave_gazebo_ros_plugins::SetCurrentDirection::Response& _res);

    /// \brief Service update to a stratified current vertical angle mean value
    public: bool UpdateStratVertAngle(
        dave_gazebo_ros_plugins::SetStratifiedCurrentDirection::Request& _req,
        dave_gazebo_ros_plugins::
        SetStratifiedCurrentDirection::Response& _res);

    /// \brief Publishes ROS topics
    private: void OnUpdateCurrentVel();

    /// \brief All underwater world services
    private: std::map<std::string, ros::ServiceServer> worldServices;

    /// \brief Pointer to this ROS node's handle.
    private: boost::shared_ptr<ros::NodeHandle> rosNode;

    /// \brief Connection for callbacks on update world.
    private: gazebo::event::ConnectionPtr rosPublishConnection;

    /// \brief Publisher for the flow velocity in the world frame
    private: ros::Publisher flowVelocityPub;

    /// \brief Publisher for the stratified current in the world frame
    private: ros::Publisher stratifiedCurrentVelocityPub;

    /// \brief Stratified ocean current database topic
    private: std::string stratifiedCurrentVelocityDatabaseTopic;

    /// \brief Publisher for the stratified current database in the world frame
    private: ros::Publisher stratifiedCurrentDatabasePub;

    /// \brief Period after which we should publish a message via ROS.
    private: gazebo::common::Time rosPublishPeriod;

    /// \brief Last time we published a message via ROS.
    private: gazebo::common::Time lastRosPublishTime;
  };
}

#endif  // OCEAN_CURRENT_PLUGIN_H_
