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

/// \file TransientCurrentPlugin.cc

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

#include <dave_model_plugins/TransientCurrentPlugin.hh>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(TransientCurrentPlugin)

/////////////////////////////////////////////////
TransientCurrentPlugin::TransientCurrentPlugin()
{
  // Doing nothing for now
}

/////////////////////////////////////////////////
TransientCurrentPlugin::~TransientCurrentPlugin()
{
#if GAZEBO_MAJOR_VERSION >= 8
  this->updateConnection.reset();
#else
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
#endif
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
  // Initializing the transport node
  this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION >= 8
  this->node->Init(this->world->Name());
#else
  this->node->Init(this->world->GetName());
#endif

  // Read the topic name from the SDF file
  this->vehicleDepthTopic = "vehicle_depth";

  GZ_ASSERT(!this->vehicleDepthTopic.empty(),
    "Empty vehicle depth topic at model side");

  // Advertise the current velocity topic
  this->publisher = 
  this->node->Advertise<msgs::Any>(this->vehicleDepthTopic);

  gzmsg << "Current vehicle depth topic name: " 
    << this->vehicleDepthTopic << std::endl;

  // Read the base_link name from the SDF file
  this->baseLinkName = _sdf->Get<std::string>("base_link_name");

  // Connect the update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&TransientCurrentPlugin::Update,
    this, _1));

  gzmsg << "Transient current plugin loaded!" << std::endl;
}

/////////////////////////////////////////////////
void TransientCurrentPlugin::Init()
{
  // Doing nothing for now
}

/////////////////////////////////////////////////
void TransientCurrentPlugin::Update
(const common::UpdateInfo & /** _info */)
{
  this->PublishVehicleDepth();
}

/////////////////////////////////////////////////
void TransientCurrentPlugin::PublishVehicleDepth()
{
  msgs::Any _vehicleDepth;
  ignition::math::Pose3d pose = 
    this->model->GetLink(this->baseLinkName)->WorldPose();
  _vehicleDepth = msgs::ConvertAny(-pose.Pos().Z());
  this->publisher->Publish(_vehicleDepth);
}
