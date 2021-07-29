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

#include <dave_gazebo_ros_plugins/pulse_lidar_plugin.h>

#include <string>

namespace gazebo
{
/// \brief The load function is called by Gazebo when the plugin is
/// inserted into simulation
/// \param[in] _model A pointer to the model that this plugin is
/// attached to.
/// \param[in] _sdf A pointer to the plugin's SDF element.
void PulseLidarROSPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Safety check
  if (_model->GetJointCount() == 0)
  {
    ROS_FATAL_STREAM_NAMED("pulse_lidar", "Invalid joint count; "
                           "NPS Gazebo ROS lidar plugin will not be loaded.");
    return;
  }
  ROS_INFO_NAMED("pulse_lidar",
                 "The NPS Gazebo ROS pulse lidar plugin is attached to "
                 "model [%s]", _model->GetName().c_str());

  // Store the model pointer for convenience.
  this->model = _model;

  // Check whether a particular lidar namespace has been specified
  // if not, default to uwl
  std::string ns = "uwl";
  if (_sdf->HasElement("robot_namespace"))
  {
    ns = _sdf->Get<std::string>("robot_namespace");
    ROS_INFO_NAMED("pulse_lidar", "lidar namespace = %s", ns.c_str());
  }

  // Get the joints
  this->pan_joint = this->model->GetJoint(ns + "/uwl_base_swivel_joint");
  this->tilt_joint = this->model->GetJoint(ns + "/uwl_swivel_tray_joint");

  // Setup a P-controller, with _imax = 1
  this->pan_pid = common::PID(1, 0, 2.5, 1);

  // Setup a P-controller with _imax = 10
  // This is very unstable out of water and requires very high gain values
  // to get close to compliance.
  this->tilt_pid = common::PID(250, 50, 100, 50);

  // Apply the P0-controller to the joint.
  this->model->GetJointController()->SetPositionPID(
      this->pan_joint->GetScopedName(), this->pan_pid);

  this->model->GetJointController()->SetPositionPID(
      this->tilt_joint->GetScopedName(), this->tilt_pid);

  // Default to no pan or tilt
  double pan_position = 0;
  double tilt_position = 0;

  // Check that the velocity element exists, then read the value
  if (_sdf->HasElement("pan_position"))
  {
    pan_position = _sdf->Get<double>("pan_position");
    ROS_INFO_NAMED("pulse_lidar", "pan_position = %f", pan_position);
  }

  // Check that the velocity element exists, then read the value
  if (_sdf->HasElement("tilt_position"))
  {
    tilt_position = _sdf->Get<double>("tilt_position");
    ROS_INFO_NAMED("pulse_lidar", "tilt_position = %f", tilt_position);
  }

  // Set the joints' target positions.
  this->model->GetJointController()->SetPositionTarget(
      this->pan_joint->GetScopedName(), pan_position);

  this->model->GetJointController()->SetPositionTarget(
      this->tilt_joint->GetScopedName(), tilt_position);

  // Create the node
  this->node = transport::NodePtr(new transport::Node());
  #if GAZEBO_MAJOR_VERSION < 8
  this->node->Init(this->model->GetWorld()->GetName());
  #else
  this->node->Init(this->model->GetWorld()->Name());
  #endif

  // Initialize ros, if it has not already bee initialized.
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client",
              ros::init_options::NoSigintHandler);
  }

  // Create our ROS node. This acts in a similar manner to
  // the Gazebo node
  this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

  // Create a named topic, and subscribe to it.
  ros::SubscribeOptions so_pan =
    ros::SubscribeOptions::create<std_msgs::Float32>(
        "/" + this->model->GetName() + "/uwl_cmd/pan",
        1,
        boost::bind(&PulseLidarROSPlugin::OnRosPanMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);
  this->rosSubPan = this->rosNode->subscribe(so_pan);

  this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

  // Create a named topic, and subscribe to it.
  ros::SubscribeOptions so_tilt =
    ros::SubscribeOptions::create<std_msgs::Float32>(
        "/" + this->model->GetName() + "/uwl_cmd/tilt",
        1,
        boost::bind(&PulseLidarROSPlugin::OnRosTiltMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);
  this->rosSubTilt = this->rosNode->subscribe(so_tilt);

  // Spin up the queue helper thread.
  this->rosQueueThread =
    std::thread(std::bind(&PulseLidarROSPlugin::QueueThread, this));
}


/// \brief Set the position of the lidar
/// \param[in] _vel New target position
void PulseLidarROSPlugin::SetPanPosition(const double &_pos)
{
  // Set the joint's target velocity.
  this->model->GetJointController()->SetPositionTarget(
        this->pan_joint->GetScopedName(), _pos);
}


/// \brief Set the position of the lidar
/// \param[in] _vel New target position
void PulseLidarROSPlugin::SetTiltPosition(const double &_pos)
{
  // Set the joint's target velocity.
  this->model->GetJointController()->SetPositionTarget(
      this->tilt_joint->GetScopedName(), _pos);
}


/// \brief Handle incoming message
/// \param[in] _msg Repurpose a vector3 message. This function will
/// only use the x component.
void PulseLidarROSPlugin::OnMsg(ConstVector3dPtr &_msg)
{
  this->SetPanPosition(_msg->x());
  this->SetTiltPosition(_msg->y());
}


/// \brief Handle an incoming message from ROS
/// \param[in] _msg A float value that is used to set the velocity
/// of the lidar.
void PulseLidarROSPlugin::OnRosPanMsg(const std_msgs::Float32ConstPtr &_msg)
{
  this->SetPanPosition(_msg->data);
}

void PulseLidarROSPlugin::OnRosTiltMsg(const std_msgs::Float32ConstPtr &_msg)
{
  this->SetTiltPosition(_msg->data);
}


/// \brief ROS helper function that processes messages
void PulseLidarROSPlugin::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(PulseLidarROSPlugin)
}
