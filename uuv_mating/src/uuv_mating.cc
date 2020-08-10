#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <string_view>
#include <gazebo/physics/Collision.hh>
#include <gazebo/sensors/sensors.hh>

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <sstream>

namespace gazebo
{
  class WorldUuvPlugin : public WorldPlugin
  {

  enum states{
    unconnectable_unlocked, connectable_unlocked, connectable_locked
  };

  // private: states state = unconnectable_unlcoked;

  private: physics::WorldPtr world;

  private: physics::ModelPtr plugModel;

  private: physics::LinkPtr plugLink;

  private: physics::LinkPtr tubeLink;

  private: physics::ModelPtr socketModel;

  private: physics::LinkPtr socketLink;

  private: physics::LinkPtr sensorPlate;

  private: ignition::math::Pose3d socket_pose;

  private: ignition::math::Pose3d plug_pose;

  private: physics::JointPtr prismaticJoint;

  public: ignition::math::Vector3d grabAxis;

  public: ignition::math::Vector3d grabbedForce;

  public: ignition::math::Vector3d someforce;

  private: bool joined = false;
  
  private: bool unlocked = true;

  private: gazebo::event::ConnectionPtr updateConnection;

  public: ros::Publisher chatter_pub;


  /// \brief A node use for ROS transport
  private: std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief A ROS subscriber
  // private: ros::Subscriber rosSub;

  /// \brief A ROS callbackqueue that helps process messages
  // private: ros::CallbackQueue rosQueue;

  /// \brief A thread the keeps running the rosQueue
  // private: std::thread rosQueueThread;


  public: WorldUuvPlugin() : WorldPlugin(){}

  public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
      this->world = _world;
      this->socketModel = this->world->ModelByName("socket_bar");
      this->plugModel = this->world->ModelByName("grab_bar");


      
      this->sensorPlate = this->socketModel->GetLink("sensor_plate");
      this->tubeLink = this->socketModel->GetLink("tube");
      this->plugLink = this->plugModel->GetLink("grab_bar_link");

      this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
          std::bind(&WorldUuvPlugin::Update, this));


      // Make sure the ROS node for Gazebo has already been initialized
      if (!ros::isInitialized())
      {
        ROS_INFO("############ \n\n not inited\n#########");
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
      chatter_pub = this->rosNode->advertise<std_msgs::Float64>("chatter", 1000);

      
      // Create a named topic, and subscribe to it.
      // ros::SubscribeOptions so =
      //   ros::SubscribeOptions::create<std_msgs::Float32>(
      //       "/" + this->model->GetName() + "/vel_cmd",
      //       1,
      //       boost::bind(&VelodynePlugin::OnRosMsg, this, _1),
      //       ros::VoidPtr(), &this->rosQueue);
      // this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      // this->rosQueueThread =
      // std::thread(std::bind(&VelodynePlugin::QueueThread, this));
  
    }




  
  public: void Update()
    {

      /**
      * The publish() function is how you send messages. The parameter
      * is the message object. The type of this object must agree with the type
      * given as a template parameter to the advertise<>() call, as was done
      * in the constructor above.
      */


      // connect the socket and the plug after 5 seconds
      if (this->world->SimTime() > 0.0 && joined == false)
      {
        printf("joint formed\n");
        gzmsg << world->Physics()->GetType();

        this->joined = true;
        this->prismaticJoint = plugModel->CreateJoint(
          "plug_socket_joint",
          // "fixed",
          "prismatic",
          tubeLink,
          plugLink);
        prismaticJoint->Load(this->tubeLink, this->plugLink, 
          // ignition::math::Pose3<double>(0,0,0,0,0,0));
          ignition::math::Pose3<double>(ignition::math::Vector3<double>(0, 0, 0), 
          ignition::math::Quaternion<double>(0, 0, 0, 0)));
        // prismaticJoint->SetUpperLimit(0, 0.3);
        prismaticJoint->Init();
        prismaticJoint->SetAxis(0, ignition::math::Vector3<double>(0, 0, 1));
        // if (prismaticJoint->SetParam("hi_stop", 0, 3.0)){
        //     printf("Succeeded\n");
            
        //   }
        //   else{
        //     printf("Failed\n");

        // }
        // if (prismaticJoint->SetParam("lo_stop", 0, -3.0)){
        //     printf("Succeeded\n");
            
        //   }
        //   else{
        //     printf("Failed\n");

        // }



        prismaticJoint->SetUpperLimit(0, 1.0);
        // prismaticJoint->SetLowerLimit(0, -10);
        // prismaticJoint->stiffnessCoefficient[0] = 50;
        prismaticJoint->SetAnchor(0, ignition::math::Vector3<double>(1, 0, 0));
      }

      // if (this->world->SimTime() > 4.0 && joined)
      // {
        // this makes gazebo crash on touch
        // prismaticJoint->SetStiffness(0,100);
        // prismaticJoint->SetDamping(0,100);

      if (true){
        // grabbedForce = sensorPlate->RelativeForce();
        grabbedForce = plugLink->RelativeForce();
        if (true){
        // if (abs(grabbedForce[0] >= 0)){
          // printf("%.1f %.1f %.1f \n", grabbedForce[0], grabbedForce[1], grabbedForce[2]);
          if (grabbedForce[2] >5){
              printf("\t\t%.1f \n",  grabbedForce[2]);

          } else if (grabbedForce[2] < -5) {
              printf("%.1f \n",  grabbedForce[2]);

          }
          std_msgs::Float64 msg;
          msg.data = grabbedForce[2];
          chatter_pub.publish(msg);

          // ROS_INFO("%s", msg.data.c_str());
          // this->freezeJoint(this->prismaticJoint);
          // printf("forzen the joint \n");
        }

      }

      if (false){
        printf("%.1f \n",  prismaticJoint->GetForce(0));
      }


      // doesnt work for some reason
      if (false){
        prismaticJoint->LinkForce(0);
        // someforce = prismaticJoint->LinkForce(0);
        // gzmsg << f[0];
        // printf("%.1f \n",  f[0]);
      }
      // }
    }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldUuvPlugin)
} // namespace gazebo
