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

/// \file spherical_coordinates_interface.h

#ifndef SPHERICAL_COORDINATES_INTERFACE_H_
#define SPHERICAL_COORDINATES_INTERFACE_H_

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <dave_gazebo_ros_plugins/SetOriginSphericalCoord.h>
#include <dave_gazebo_ros_plugins/GetOriginSphericalCoord.h>
#include <dave_gazebo_ros_plugins/TransformToSphericalCoord.h>
#include <dave_gazebo_ros_plugins/TransformFromSphericalCoord.h>
#include <boost/shared_ptr.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/SphericalCoordinates.hh>
#include <gazebo/physics/World.hh>

#include <map>
#include <string>

namespace gazebo
{
  class SphericalCoordinatesROSInterfacePlugin : public WorldPlugin
  {
    /// \brief Constructor
    public: SphericalCoordinatesROSInterfacePlugin();

    /// \brief Destructor
    public: virtual ~SphericalCoordinatesROSInterfacePlugin();

    /// \brief Load module and read parameters from SDF.
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Service call that returns the origin in WGS84 standard
    public: bool GetOriginSphericalCoord(
        dave_gazebo_ros_plugins::GetOriginSphericalCoord::Request& _req,
        dave_gazebo_ros_plugins::GetOriginSphericalCoord::Response& _res);

    /// \brief Service call that returns the origin in WGS84 standard
    public: bool SetOriginSphericalCoord(
        dave_gazebo_ros_plugins::SetOriginSphericalCoord::Request& _req,
        dave_gazebo_ros_plugins::SetOriginSphericalCoord::Response& _res);

    /// \brief Service call to transform from Cartesian to spherical coordinates
    public: bool TransformToSphericalCoord(
        dave_gazebo_ros_plugins::TransformToSphericalCoord::Request& _req,
        dave_gazebo_ros_plugins::TransformToSphericalCoord::Response& _res);

    /// \brief Service call to transform from spherical to Cartesian coordinates
    public: bool TransformFromSphericalCoord(
        dave_gazebo_ros_plugins::TransformFromSphericalCoord::Request& _req,
        dave_gazebo_ros_plugins::TransformFromSphericalCoord::Response& _res);

    /// \brief Pointer to this ROS node's handle.
    protected: boost::shared_ptr<ros::NodeHandle> rosNode;

    /// \brief Connection for callbacks on update world.
    protected: event::ConnectionPtr rosPublishConnection;

    /// \brief Pointer to world
    protected: physics::WorldPtr world;

    /// \brief All underwater world services
    protected: std::map<std::string, ros::ServiceServer> worldServices;
  };
}

#endif  // SPHERICAL_COORDINATES_INTERFACE_H_
