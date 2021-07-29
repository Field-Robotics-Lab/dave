#ifndef GAZEBO_TRANSPONDER_PLUGIN_HPP_
#define GAZEBO_TRANSPONDER_PLUGIN_HPP_

#include <thread>
#include <random>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/transport/Node.hh>
#include <ignition/math/Pose3.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include "usbl_gazebo/USBLResponse.h"
#include "usbl_gazebo/USBLCommand.h"

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
      void commandRosCallback(const usbl_gazebo::USBLCommandConstPtr& msg);
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
