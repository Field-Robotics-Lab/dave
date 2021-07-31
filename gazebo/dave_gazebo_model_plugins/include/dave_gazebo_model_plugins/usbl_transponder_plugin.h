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

#ifndef USBL_TRANSPONDER_PLUGIN_H_
#define USBL_TRANSPONDER_PLUGIN_H_

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <string>
#include <thread>
#include <random>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/transport/Node.hh>
#include <ignition/math/Pose3.hh>

#include "dave_gazebo_model_plugins/UsblResponse.h"
#include "dave_gazebo_model_plugins/UsblCommand.h"

namespace gazebo
{
  class TransponderPlugin : public ModelPlugin
  {
    public:
      TransponderPlugin();
      ~TransponderPlugin();

      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      void sendLocation();
      void temperatureRosCallback(const std_msgs::Float64ConstPtr &msg);
      void iisRosCallback(const std_msgs::StringConstPtr &msg);
      void cisRosCallback(const std_msgs::StringConstPtr &msg);
      void commandRosCallback
           (const dave_gazebo_model_plugins::UsblCommandConstPtr& msg);
      void queueThread();

    private:
      std::string m_namespace;
      std::string m_transponderDevice;
      std::string m_transponderID;
      std::string m_transceiverDevice;
      std::string m_transceiverID;

      // environment variables
      double m_temperature;
      double m_soundSpeed;
      double m_noiseMu;
      double m_noiseSigma;

      // Gazebo nodes, publishers, and subscribers
      transport::NodePtr m_gzNode;
      transport::PublisherPtr m_globalPosPub;
      physics::ModelPtr m_model;

      // ROS nodes, publishers and subscibers
      ros::Publisher m_commandResponsePub;
      ros::Subscriber m_iisSub;
      ros::Subscriber m_cisSub;
      ros::Subscriber m_commandSub;
      ros::Subscriber m_temperatureSub;
      ros::CallbackQueue m_rosQueue;

      std::unique_ptr<ros::NodeHandle> m_rosNode;
      std::thread m_rosQueueThread;
  };

  GZ_REGISTER_MODEL_PLUGIN(TransponderPlugin)
}
#endif
