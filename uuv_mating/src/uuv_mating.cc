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

  public: double positiveCount = 0;

  public: double negativeCount = 0;

  public: physics::JointWrench FT;

  public: std::string collisionTopic;

  private: gazebo::transport::SubscriberPtr collisionSub;

  public: physics::ContactManager* _contactManagerPtr;

  public:  physics::ContactPtr _contactPtr;
  
  public: WorldUuvPlugin() 
    : WorldPlugin(){
  }

  public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
      this->world = _world;
      this->socketModel = this->world->ModelByName("socket_bar");
      this->plugModel = this->world->ModelByName("grab_bar");


      this->sensorPlate = this->socketModel->GetLink("sensor_plate");
      this->tubeLink = this->socketModel->GetLink("tube");
      this->plugLink = this->plugModel->GetLink("grab_bar_link");
      this->world->Physics()->GetContactManager()->SetNeverDropContacts(true);
      
      this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
          std::bind(&WorldUuvPlugin::Update, this));
    }

  public: void Update()
    {

      if (this->world->SimTime() > 0.0 && joined == false)
      {
        this->tubeLink->SetLinkStatic(true);
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
          ignition::math::Pose3<double>(ignition::math::Vector3<double>(0, 0, 0), 
          ignition::math::Quaternion<double>(0, 0, 0, 0)));
        // prismaticJoint->SetUpperLimit(0, 0.3);
        prismaticJoint->Init();
        prismaticJoint->SetAxis(0, ignition::math::Vector3<double>(0, 0, 1));

        prismaticJoint->SetUpperLimit(0, 1.0);
        
      }

      for(int i=0; i<this->world->Physics()->GetContactManager()->GetContactCount(); i++)
      {

        physics::Contact *contact = this->world->Physics()->GetContactManager()->GetContact(i);
        // ROS_INFO("%s\n", contact->collision1->GetLink()->GetName());
        if (contact->collision1->GetLink()->GetName() == "sensor_plate" && contact->collision2->GetLink()->GetName() == "grab_bar_link"
           ||
           contact->collision1->GetLink()->GetName() == "grab_bar_link" && contact->collision2->GetLink()->GetName() == "sensor_plate"
        ){
          ROS_INFO("%s %f %s %f", contact->collision1->GetLink()->GetName().c_str(),
          contact->wrench[i].body1Force[2],
          contact->collision2->GetLink()->GetName().c_str(),
          contact->wrench[i].body2Force[2]);
        }
      }
    }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldUuvPlugin)
} // namespace gazebo
