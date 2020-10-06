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

/// \file TransientCurrentPlugin.hh
/// \brief Plugin for the transient current plugin to publish vehicle depth

#ifndef __TRANSIENT_CURRENT_PLUGIN_HH__
#define __TRANSIENT_CURRENT_PLUGIN_HH__

#include <map>
#include <cmath>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <sdf/sdf.hh>

#include <std_msgs/Float32.h>

namespace gazebo
{
  /// \brief Class for the underwater current plugin
  class TransientCurrentPlugin : public ModelPlugin
  {
    /// \brief Class constructor
    public: TransientCurrentPlugin();

    /// \brief Class destructor
    public: virtual ~TransientCurrentPlugin();

    // Documentation inherited.
    public: virtual void Load(physics::ModelPtr _model,
        sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: virtual void Init();

    /// \brief Update the simulation state.
    /// \param[in] _info Information used in the update event.
    public: void Update(const common::UpdateInfo &_info);

    /// \brief Publish current velocity and the pose of its frame
    protected: void PublishVehicleDepth();

    /// \brief Update event
    protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to world
    protected: physics::WorldPtr world;

    /// \brief Pointer to model
    protected: physics::ModelPtr model;

    /// \brief Pointer to sdf
    protected: sdf::ElementPtr sdf;

    /// \brief Vehicle base link name
    protected: std::string baseLinkName;

    /// \brief True if the sea surface is present
    protected: bool hasSurface;

    /// \brief Pointer to a node for communication
    protected: transport::NodePtr node;

    /// \brief Publishers
    protected: transport::PublisherPtr publisher;

    /// \brief Vehicle Depth topic
    protected: std::string vehicleDepthTopic;

    /// \brief Namespace for topics and services
    protected: std::string ns;
  };
}

#endif  // __TRANSIENT_CURRENT_PLUGIN_HH__
