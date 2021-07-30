/*
 * Copyright 2020 Naval Postgraduate School
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef PULSE_LIDAR_PLUGIN_H_
#define PULSE_LIDAR_PLUGIN_H_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <string>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class PulseLidarROSPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: PulseLidarROSPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Set the position of the lidar
    /// \param[in] _vel New target position
    public: void SetPanPosition(const double &_pos);

    /// \brief Set the position of the lidar
    /// \param[in] _vel New target position
    public: void SetTiltPosition(const double &_pos);

    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private: void OnMsg(ConstVector3dPtr &_msg);

    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the lidar.
    public: void OnRosPanMsg(const std_msgs::Float32ConstPtr &_msg);

    public: void OnRosTiltMsg(const std_msgs::Float32ConstPtr &_msg);

    /// \brief ROS helper function that processes messages
    private: void QueueThread();

    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    public: physics::JointControllerPtr jointController;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr pan_joint;

    /// \brief Pointer to the joint.
    private: physics::JointPtr tilt_joint;

     /// \brief A PID controller for pan and tilt motion.
    private: common::PID pan_pid;

    /// \brief A PID controller for pan and tilt motion.
    private: common::PID tilt_pid;

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSubPan;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSubTilt;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
  };
}

#endif  // PULSE_LIDAR_PLUGIN_H_
