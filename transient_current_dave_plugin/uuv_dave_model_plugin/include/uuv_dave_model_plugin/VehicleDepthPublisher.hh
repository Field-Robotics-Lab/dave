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

/// \file VehicleDepthPublisher.hh
/// \brief Vehicle depth publisher for transient depth current

#ifndef __VEHICLE_DEPTH_PUBLISHER_HH__
#define __VEHICLE_DEPTH_PUBLISHER_HH__

#include <map>
#include <string>
#include <iostream>
#include <fstream>

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/TransportTypes.hh>


namespace gazebo
{

class VehicleDepthPublisher : public ModelPlugin
{
/// \brief Constructor
public: VehicleDepthPublisher();

/// \brief Destructor
public: virtual ~VehicleDepthPublisher();

// Documentation inherited.
public: virtual void Load(physics::ModelPtr _model,
                        sdf::ElementPtr _sdf);

// Documentation inherited.
public: virtual void Init();

/// \brief Update the simulation state.
/// \param[in] _info Information used in the update event.
public: virtual void Update(const common::UpdateInfo &_info);

/// \brief Connects the update event callback
protected: virtual void Connect();

/// \brief Publish vehicle depth
protected: virtual void PublishVehicleDepth(
    physics::LinkPtr _link);

/// \brief Vehicle base depth topic
protected: std::string vehicleDepthTopic;

/// \brief Update event
protected: event::ConnectionPtr updateConnection;

/// \brief Pointer to the world plugin
protected: physics::WorldPtr world;

/// \brief Pointer to the model structure
protected: physics::ModelPtr model;

/// \brief Gazebo node
protected: transport::NodePtr node;

/// \brief Name of vehicle's base_link
protected: std::string baseLinkName;

/// \brief Publishers for vehicleDepth
protected: transport::PublisherPtr vehicleDepthPub;
};
}

#endif  // __VEHICLE_DEPTH_PUBLISHER_HH__
