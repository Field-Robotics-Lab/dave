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

  public: double positiveCount = 0;

  public: double negativeCount = 0;

  public: physics::JointWrench FT;

  public: std::string collisionTopic;

  private: gazebo::transport::SubscriberPtr collisionSub;

  // private: gazebo::transport::NodePtr gzNode;

  public: physics::ContactManager* _contactManagerPtr;

  public:  physics::ContactPtr _contactPtr;
  
  // public:  std::vector<Contact*> & contactVector;

  /// \brief A node use for ROS transport
  // private: std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief A ROS subscriber
  // private: ros::Subscriber rosSub;

  /// \brief A ROS callbackqueue that helps process messages
  // private: ros::CallbackQueue rosQueue;

  /// \brief A thread the keeps running the rosQueue
  // private: std::thread rosQueueThread;


  public: WorldUuvPlugin() 
    : WorldPlugin(){
    // : WorldPlugin(), gzNode(new gazebo::transport::Node()){
  }

  public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
      // _contactManagerPtr = _world->Physics()->GetContactManager();
      // _contactManagerPtr->Init(_world);
      // gzNode->Init();
      // this->collisionTopic = "/gazebo/oceans_waves/physics/contacts";
      // this->collisionSub = gzNode->Subscribe(collisionTopic, &WorldUuvPlugin::OnCollisionMsg, this);

      this->world = _world;
      this->socketModel = this->world->ModelByName("socket_bar");
      this->plugModel = this->world->ModelByName("grab_bar");


      this->sensorPlate = this->socketModel->GetLink("sensor_plate");
      this->tubeLink = this->socketModel->GetLink("tube");
      this->plugLink = this->plugModel->GetLink("grab_bar_link");
      this->world->Physics()->GetContactManager()->SetNeverDropContacts(true);
      
      this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
          std::bind(&WorldUuvPlugin::Update, this));


      // Make sure the ROS node for Gazebo has already been initialized
      // if (!ros::isInitialized())
      // {
      //   ROS_INFO("############ \n\n not inited\n#########");
      //   int argc = 0;
      //   char **argv = NULL;
      //   ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      // }

      // this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
      // chatter_pub = this->rosNode->advertise<std_msgs::Float64>("chatter", 1000);

    }


  //     std::cout << typeid(_contacts->contact(i).count).name() << '\n';
  // public: void OnCollisionMsg(ConstContactsPtr &_contacts) {
  //   // _contacts->contact_size();
  //   // std::string wamvCollisionStr1 = _contacts->contact(1).collision1();
  //   // std::string wamvCollisionStr2 = _contacts->contact(2).collision2();
  //   for (unsigned int i = 0; i < _contacts->contact_size(); ++i) {
  //     std::string wamvCollisionStr1 = _contacts->contact(i).collision1();
  //     std::string wamvCollisionStr2 = _contacts->contact(i).collision2();
  //     ROS_INFO( "%s || %s", wamvCollisionStr1.c_str() , wamvCollisionStr2.c_str());
      
  //     // physics::JointWrench[255] forces_touch = _contacts->contact(i).wrench;
  //     // ignition::math::Vector3d 	ff1 = forces_touch.body1Force;
  //     // ignition::math::Vector3d 	ff2 = forces_touch.body2Force;
  //     // printf("%.2f %.2f %.2f || %.2f %.2f %.2f || ",  ff1[0],ff1[1],ff1[2], ff2[0],ff2[1],ff2[2]);
  //   }

  //   // ROS_INFO_THROTTLE(0.5, _contacts->contact_size());
  //   // std::string wamvCollisionSubStr1 =
  //   //     wamvCollisionStr1.substr(0, wamvCollisionStr1.find("lump"));
  //   // std::string wamvCollisionSubStr2 =
  //   //     wamvCollisionStr2.substr(0, wamvCollisionStr2.find("lump"));

  //   // return;
  // }

  
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
        
        // prismaticJoint->SetLowerLimit(0, -10);
        // prismaticJoint->stiffnessCoefficient[0] = 50;
        // prismaticJoint->SetAnchor(0, ignition::math::Vector3<double>(1, 0, 0));
        // prismaticJoint->SetProvideFeedback(true);
        // prismaticJoint->SetStiffness (0, 50000);
      }

      // if (this->world->SimTime() > 4.0 && joined)
      // {
        // this makes gazebo crash on touch
        // prismaticJoint->SetStiffness(0,100);
        // prismaticJoint->SetDamping(0,100);
// _world->Physics()->GetContactManager()->GetContactCount();
      // for(int i=0; i<this->world->Physics()->GetContactManager()->GetContacts().size(); i++)
      for(int i=0; i<this->world->Physics()->GetContactManager()->GetContactCount(); i++)
      {

        // ROS_INFO("collision %i of %u \n",i+1,this->world->Physics()->GetContactManager()->GetContactCount());
        // ROS_INFO("collision %i of %i \n",i,this->world->Physics()->GetContactManager()->GetContacts().size() - 1);
        physics::Contact *contact = this->world->Physics()->GetContactManager()->GetContact(i);
        // ROS_INFO("%s\n", contact->collision1->GetLink()->GetName());
        if (contact->collision1->GetLink()->GetName() == "sensor_plate" && contact->collision2->GetLink()->GetName() == "grab_bar_link"
           ||
           contact->collision1->GetLink()->GetName() == "grab_bar_link" && contact->collision2->GetLink()->GetName() == "sensor_plate"
        ){
          // ROS_INFO("its actually working!!");
          // std::cout << "contact between "<<contact->collision1->GetLink()->GetName()<<" and "
          // << contact-> collision2-> GetLink()->GetName() <<std::endl; 
          ROS_INFO("%f %f",contact->wrench[i].body1Force[2],contact->wrench[i].body2Force[2]);
          // ROS_INFO("%f",contact->wrench[i].body2Force[2]);

        }
        // printf("%.2f %.2f %.2f || %.2f %.2f %.2f || ",  f1[0],f1[1],f1[2], f2[0],f2[1],f2[2]);
        // ROS_INFO("/////// \n");
      }
      // ROS_INFO("/////// \n");

        // if (_contactManagerPtr->GetContactCount()){
        // 
        // }
        // ROS_INFO_STREAM(_contactManagerPtr->GetContactCount()\n);

        // try {
          
        //     std::cout << typeid(_contactManagerPtr->GetFilterCount()).name() << '\n';
        // } catch (const std::runtime_error& e) {
        //     printf("testing stuff ");
        // }
        // contactVector->GetContacts();

      if (false){
        // grabbedForce = sensorPlate->RelativeForce();
        this->FT = prismaticJoint->GetForceTorque(0);
        ignition::math::Vector3d f1 = this->FT.body1Force;
        ignition::math::Vector3d f2 = this->FT.body2Force;
        printf("%.2f %.2f %.2f || %.2f %.2f %.2f || ",  f1[0],f1[1],f1[2], f2[0],f2[1],f2[2]);
        // grabbedForce = sensorPlate->RelativeForce();
        grabbedForce = plugLink->RelativeForce();
        // plugLink->UpdateVisualMsg()
        if (false){
        // if (abs(grabbedForce[0] >= 0)){
          // printf("%.1f %.1f %.1f \n", grabbedForce[0], grabbedForce[1], grabbedForce[2]);
          double forceThresh =5;
          if (grabbedForce[2] >forceThresh){
              positiveCount+= grabbedForce[2];
              printf("\t\t%.1f \t\t %.1f\n",  grabbedForce[2],positiveCount);

          } else if (grabbedForce[2] < -forceThresh) {
              negativeCount+= grabbedForce[2];
              printf("%.1f \t\t %.1f\n",  grabbedForce[2],negativeCount);
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
        // printf("Joint force: %.1f \n",  prismaticJoint->GetForce(0));
        ROS_INFO_STREAM_THROTTLE(0.1,this->prismaticJoint->GetSpringReferencePosition(0));
        // printf("Joint psoiton: %.5f \n",  this->prismaticJoint->GetSpringReferencePosition(0));
        // printf("Joint psoiton: %.5f \n",  prismaticJoint->Position(0));
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
