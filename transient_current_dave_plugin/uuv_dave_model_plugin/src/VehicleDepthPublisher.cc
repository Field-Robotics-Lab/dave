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

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/Shape.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>

#include <cmath>
#include <math.h>

#include <uuv_dave_model_plugin/VehicleDepthPublisher.hh>

namespace gazebo
{

  GZ_REGISTER_MODEL_PLUGIN(VehicleDepthPublisher)

  /////////////////////////////////////////////////
  VehicleDepthPublisher::VehicleDepthPublisher()
  {
  }

  /////////////////////////////////////////////////
  VehicleDepthPublisher::~VehicleDepthPublisher()
  {
    this->updateConnection.reset();
  }

  /////////////////////////////////////////////////
  void VehicleDepthPublisher::Load(physics::ModelPtr _model,
                                    sdf::ElementPtr _sdf)
  {
    GZ_ASSERT(_model != NULL, "Invalid model pointer");
    GZ_ASSERT(_sdf != NULL, "Invalid SDF element pointer");

    this->model = _model;
    this->world = _model->GetWorld();

    // Initialize the transport node
    this->node = transport::NodePtr(new transport::Node());
    std::string worldName;
    worldName = this->world->Name();
    this->node->Init(worldName);

    // Read topic name
    if (_sdf->HasElement("vehicle_depth_topic"))
      this->vehicleDepthTopic = _sdf->Get<std::string>("vehicle_depth_topic");
    else
      this->vehicleDepthTopic = "hydrodynamics/vehicle_depth";

    GZ_ASSERT(!this->vehicleDepthTopic.empty(),
            "Fluid velocity topic tag cannot be empty");

    // Advertise the topic
    this->vehicleDepthPub =
        this->node->Advertise<ignition::msgs::Double>(this->vehicleDepthTopic);

    this->baseLinkName = std::string();
    if (_sdf->HasElement("link"))
    {
      for (sdf::ElementPtr linkElem = _sdf->GetElement("link"); linkElem;
           linkElem = linkElem->GetNextElement("link"))
      {
        physics::LinkPtr link;
        std::string linkName = "";

        if (linkElem->HasAttribute("name"))
        {
          linkName = linkElem->Get<std::string>("name");
          std::size_t found = linkName.find("base_link");
          if (found != std::string::npos)
          {
            this->baseLinkName = linkName;
            gzmsg << "Name of the BASE_LINK: " << this->baseLinkName << std::endl;
          }

          link = this->model->GetLink(linkName);
          if (!link)
          {
            gzwarn << "Specified link [" << linkName << "] not found."
                   << std::endl;
            continue;
          }
        }
        else
        {
          gzwarn << "Attribute name missing from link [" << linkName
                 << "]" << std::endl;
          continue;
        }
      } // for each link mentioned in plugin sdf
    }

    // Connect the update event callback
    this->Connect();
  }

  /////////////////////////////////////////////////
  void VehicleDepthPublisher::Init()
  {
  }

  /////////////////////////////////////////////////
  void VehicleDepthPublisher::Update(const common::UpdateInfo &_info)
  {
    double time = _info.simTime.Double();
    this->PublishVehicleDepth(this->model->GetLink(this->baseLinkName));
  }

  /////////////////////////////////////////////////
  void VehicleDepthPublisher::Connect()
  {
    // Connect the update event
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&VehicleDepthPublisher::Update,
                    this, _1));
  }

  /////////////////////////////////////////////////
  void VehicleDepthPublisher::PublishVehicleDepth(
      physics::LinkPtr _link)
  {
    ignition::math::Pose3d pose = this->model->GetLink(this->baseLinkName)->WorldPose();
    ignition::msgs::Double msg;
    ignition::msgs::Set(&msg, pose.Pos().Z());
    this->vehicleDepthPub->Publish(msg);
  }
} 
