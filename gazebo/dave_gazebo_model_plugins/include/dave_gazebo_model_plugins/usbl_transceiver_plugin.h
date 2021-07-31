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

#ifndef USBL_TRANSCEIVER_PLUGIN_H_
#define USBL_TRANSCEIVER_PLUGIN_H_

#include <math.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Vector3.h>

#include <string>
#include <thread>
#include <vector>
#include <algorithm>
#include <functional>
#include <unordered_map>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/transport/Node.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/common/StringUtils.hh>

#include "dave_gazebo_model_plugins/UsblCommand.h"
#include "dave_gazebo_model_plugins/UsblResponse.h"

namespace gazebo
{
  class TransceiverPlugin : public ModelPlugin
  {
    public:
      TransceiverPlugin();
      ~TransceiverPlugin();
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      void commandingResponseTestCallback(const std_msgs::StringConstPtr &msg);
      void commandingResponseCallback
           (const dave_gazebo_model_plugins::UsblResponseConstPtr &msg);
      void sendCommand(int command_id, std::string& transponder_id);
      void sendPing(const ros::TimerEvent&);
      void channelSwitchCallback(const std_msgs::StringConstPtr &msg);
      void temperatureRosCallback(const std_msgs::Float64ConstPtr &msg);
      void interrogationModeRosCallback(const std_msgs::StringConstPtr &msg);
      void receiveGezeboCallback(ConstVector3dPtr& transponder_position);
      void publishPosition(double &bearing, double &range, double &elevation);
      void calcuateRelativePose(ignition::math::Vector3d position,
                                double &bearing, double &range,
                                double &elevation);
      void queueThread();

    public:
      // This entity's attributes
      std::string m_namespace;
      std::string m_transceiverDevice;
      std::string m_transceiverID;
      std::string m_transponderAttachedObject;
      std::string m_channel = "1";
      std::string m_interrogationMode;
      bool m_enablePingerScheduler;

    private:
      std::string m_transponderDevice;
      double m_temperature;
      double m_soundSpeed;
      std::vector<std::string> m_deployedTransponders;

      // Gazebo nodes, publishers, and subscribers
      ros::Timer m_timer;
      physics::ModelPtr m_model;
      transport::NodePtr m_gzNode;
      std::vector<transport::SubscriberPtr> m_transponderPoseSub;


      // ROS nodes, publishers and subscibers
      std::unique_ptr<ros::NodeHandle> m_rosNode;
      ros::Publisher m_publishTransponderRelPos;
      ros::Publisher m_publishTransponderRelPosCartesion;
      ros::Publisher m_cisPinger;
      std::unordered_map<std::string, ros::Publisher> m_iisPinger;
      std::unordered_map<std::string, ros::Publisher> m_commandPubs;
      ros::Subscriber m_temperatureSub;
      ros::Subscriber m_commandResponseSub;
      ros::Subscriber m_commandResponseTestSub;
      ros::Subscriber m_interrogationModeSub;
      ros::Subscriber m_channelSwitchSub;
      ros::CallbackQueue m_rosQueue;

      std::thread m_rosQueueThread;
  };

  GZ_REGISTER_MODEL_PLUGIN(TransceiverPlugin)
}
#endif
