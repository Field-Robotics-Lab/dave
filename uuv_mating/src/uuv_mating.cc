#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <string_view>
#include <gazebo/physics/Collision.hh>
#include <gazebo/sensors/sensors.hh>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <gazebo/common/common.hh>

#include <sstream>

#include <vector> 

#include <algorithm>    // std::lower_bound

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
  
  private: bool locked = false;

  private: gazebo::event::ConnectionPtr updateConnection;

  public: ros::Publisher chatter_pub;

  private: std::unique_ptr<ros::NodeHandle> rosNode;

  public: double positiveCount = 0;

  public: double negativeCount = 0;

  public: physics::JointWrench FT;

  public: std::string collisionTopic;

  private: gazebo::transport::SubscriberPtr collisionSub;

  public: physics::ContactManager* _contactManagerPtr;

  public:  physics::ContactPtr _contactPtr;

  public: int collisionForceCount = 0;
  
  std::vector<double> Zforces;
  
  std::vector<common::Time> timeStamps;

  double historyLength = 5.0; //seconds
 
  void trimForceVector(double trimDuration){
    std::vector<common::Time>::iterator low;
    low=std::lower_bound (this->timeStamps.begin(), this->timeStamps.end(), this->timeStamps.back()-trimDuration);
    this->timeStamps.erase (this->timeStamps.begin(), this->timeStamps.begin() + std::distance( this->timeStamps.begin(), low ));
    this->Zforces.erase (this->Zforces.begin(), this->Zforces.begin() + std::distance( this->timeStamps.begin(), low ));
  };

  double movingTimedAverage(){
    return accumulate( this->Zforces.begin(), this->Zforces.end(), 0.0) / this->Zforces.size(); 
  };

  void addForce(double force){
    if (abs(force) < 5.0){
      return;
    }
    this->Zforces.push_back(force);
    this->timeStamps.push_back(this->world->SimTime());
  }

  public: WorldUuvPlugin() 
    : WorldPlugin(){
  }

  public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
      this->world = _world;
      this->socketModel = this->world->ModelByName("socket_box");
      this->plugModel = this->world->ModelByName("plug");


      this->sensorPlate = this->socketModel->GetLink("sensor_plate");
      this->tubeLink = this->socketModel->GetLink("socket");
      this->plugLink = this->plugModel->GetLink("plug");
      this->world->Physics()->GetContactManager()->SetNeverDropContacts(true);
      
      this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
          std::bind(&WorldUuvPlugin::Update, this));

      // if (!ros::isInitialized())
      // {
      //   ROS_INFO("############ \n\n not inited\n#########");
      //   int argc = 0;
      //   char **argv = NULL;
      //   ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      // }

      // // Create our ROS node. This acts in a similar manner to
      // // the Gazebo node
      // this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
      // chatter_pub = this->rosNode->advertise<std_msgs::Float64>("chatter", 1000);

    }

  public: void freezeJoint(physics::JointPtr prismaticJoint){
      if (this->locked){
        ROS_INFO("already frozen!");
        return;
      }
      this->locked = true;
      ROS_INFO("frozen!");
      double currentPosition = prismaticJoint->Position(0);
      // ROS_INFO("current position is %f ", currentPosition );
      prismaticJoint->SetUpperLimit(0, currentPosition);
      prismaticJoint->SetLowerLimit(0, currentPosition);
  }

  public: void unfreezeJoint(physics::JointPtr prismaticJoint){
      if (!this->locked){
        ROS_INFO("already unlocked");
        return;
      }
      this->locked = false;
      ROS_INFO("UNFREEZE");
      prismaticJoint->SetUpperLimit(0, 100);
      prismaticJoint->SetLowerLimit(0, -100);
  }

 private: bool checkRollAlignment(bool verbose = false){
      ignition::math::Vector3<double> socketRotation = socketModel->RelativePose().Rot().Euler();
      ignition::math::Vector3<double> plugRotation = plugModel->RelativePose().Rot().Euler();
      if (verbose){
        ROS_INFO_THROTTLE(1, "socket euler: %.2f %.2f %.2f plug euler: %.2f %.2f %.2f", socketRotation[0], socketRotation[1], socketRotation[2],plugRotation[0],plugRotation[1],plugRotation[2]+1.57079632679  );
      }
      return abs(plugRotation[0] - socketRotation[0]) < 0.1;
    }

  private: bool checkPitchAlignment(bool verbose = false){
      ignition::math::Vector3<double> socketRotation = socketModel->RelativePose().Rot().Euler();
      ignition::math::Vector3<double> plugRotation = plugModel->RelativePose().Rot().Euler();
      // ROS_INFO_THROTTLE(1, "socket pitch: %f %f %f plug pitch: %f %f %f", socketRotation[0], socketRotation[1], socketRotation[2],plugRotation[0],plugRotation[1],plugRotation[2]);
      return abs(plugRotation[1] - socketRotation[1]) < 0.1;
    }

  private:bool checkYawAlignment(bool verbose = false){
      ignition::math::Vector3<double> socketRotation = socketModel->RelativePose().Rot().Euler();
      ignition::math::Vector3<double> plugRotation = plugModel->RelativePose().Rot().Euler();
      // ROS_INFO_THROTTLE(1, "socket yaw: %f %f %f plug yaw: %f %f %f", socketRotation[0], socketRotation[1], socketRotation[2],plugRotation[0],plugRotation[1],plugRotation[2]);
      return abs(plugRotation[2] - socketRotation[2]) < 0.1;
    }

  private: bool checkRotationalAlignment()
    {
      if (this->checkYawAlignment() && this->checkPitchAlignment() && this->checkRollAlignment())
      {
        ROS_INFO_THROTTLE(1,"SOCKET AND PLUG ALIGNED");
        // printf("Aligned, ready for insertion  \n");
        return true;
      }
      else
      {
        // printf("Disaligned, not ready for mating  \n");
        return false;
      }
    }

  private: bool checkVerticalAlignment(bool verbose = false)
    {
      socket_pose = socketModel->RelativePose();
      ignition::math::Vector3<double> socketPositon = socket_pose.Pos();
      // printf("%s  \n", typeid(socketPositon).name());

      plug_pose = plugModel->RelativePose();
      ignition::math::Vector3<double> plugPosition = plug_pose.Pos();

      bool onSameVerticalLevel = abs(plugPosition[2] - socketPositon[2]) < 0.1;
      if (verbose){
        ROS_INFO_THROTTLE(1,"Z plug: %f  Z socket: %f",plugPosition[2], socketPositon[2]);
      }
      if (onSameVerticalLevel)
      {
        // printf("z=%.2f  \n", plugPosition[2]);
        // printf("Share same vertical level  \n");
        return true;
      }
      return false;
    }

  private: bool isAlligned()
    {
      if(checkVerticalAlignment() && checkRotationalAlignment()){
        return true;
      } else {
        return false;
      }
    }

  private: bool checkProximity(bool verbose = false)
    {
      socket_pose = socketModel->WorldPose();
      // socket_pose = socketLink->RelativePose();
      ignition::math::Vector3<double> socketPositon = socket_pose.Pos();
      // printf("%s  \n", typeid(socketPositon).name());

      plug_pose = plugModel->RelativePose();
      ignition::math::Vector3<double> plugPosition = plug_pose.Pos();
      // printf("%f %f %f Within Proximity  \n", plugPosition[0], plugPosition[1], plugPosition[2]);
      float xdiff_squared = pow(abs(plugPosition[0] - socketPositon[0]),2);
      float ydiff_squared = pow(abs(plugPosition[1] - socketPositon[1]),2);
      float zdiff_squared = pow(abs(plugPosition[2] - socketPositon[2]),2);

      if (verbose){
        ROS_INFO_THROTTLE(1, "eucleadian distance: %f", pow(xdiff_squared+ydiff_squared+zdiff_squared,0.5));
      }

      bool withinProximity = pow(xdiff_squared+ydiff_squared+zdiff_squared,0.5) < 1;
      if (withinProximity)
      {
        // printf("%f Within Proximity  \n", plugPosition[0]);
        return true;
      }else {

        // printf("not within Proximity  \n");
      }
      return false;
    }

  private: bool isReadyForInsertion()
    {
      if (this->checkProximity() && this->isAlligned()){
        return true;
        // printf("Insert \n");

      } else{

        // printf("Cant insert \n");
        return false;
      }
    }
    
  private: void construct_joint(){
      // this->tubeLink->SetLinkStatic(true);
      printf("joint formed\n");
      // gzmsg << this->world->Physics()->GetType();

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

      // prismaticJoint->SetUpperLimit(0, 1.0);
  }

  public: void Update(){
    this->checkRollAlignment(true);
    this->checkPitchAlignment(false);
    this->checkYawAlignment(false);
    this->checkProximity(false);
    this->checkVerticalAlignment(true);
    // isAlligned


  }
    // {



    //   // if (false){

    //   for(int i=0; i<this->world->Physics()->GetContactManager()->GetContactCount(); i++)
    //     {
    //     physics::Contact *contact = this->world->Physics()->GetContactManager()->GetContact(i);

    //     // if not locked then check forces on the contact sensor
    //     if (!this->locked){
    //       // ROS_INFO("%s\n", contact->collision1->GetLink()->GetName());
    //       if (contact->collision1->GetLink()->GetName() == "sensor_plate" && contact->collision2->GetLink()->GetName() == "grab_bar_link"
    //         ||
    //         contact->collision1->GetLink()->GetName() == "grab_bar_link" && contact->collision2->GetLink()->GetName() == "sensor_plate"
    //       ){
    //         if(abs(contact->wrench[i].body1Force[2]) < 100){
    //           std_msgs::Float64 msg;
    //           msg.data = contact->wrench[i].body1Force[2];
    //           chatter_pub.publish(msg);
    //         }

    //         if (abs(contact->wrench[i].body1Force[2]) > 15){

    //           ROS_INFO("%s %f %s %f", contact->collision1->GetLink()->GetName().c_str(),
    //           contact->wrench[i].body1Force[2],
    //           contact->collision2->GetLink()->GetName().c_str(),
    //           contact->wrench[i].body2Force[2]);
    //           collisionForceCount++;

    //           ROS_INFO("%i", collisionForceCount);
    //           if (collisionForceCount > 10){
    //             ROS_INFO("freeze");
    //             this->freezeJoint(this->prismaticJoint);
    //           }
    //         } else {
    //           // collisionForceCount--;
    //         }
    //       }
    //       // if locked, then time to check for forces on the bar itself
    //     } else if (this->locked){
    //       double unlockingForce;
    //       double unlockingForceThresh;
    //       if (contact->collision2->GetLink()->GetName() == "grab_bar_link" || contact->collision1->GetLink()->GetName() == "grab_bar_link" ){
    //         if(contact->collision2->GetLink()->GetName() == "grab_bar_link" && contact->collision1->GetLink()->GetName() != "sensor_plate"){
    //           // ROS_INFO_THROTTLE(1,"%s ", contact->collision1->GetLink()->GetName().c_str());            
              
    //           // for(int i=0; i < this->Zforces.size(); i++){std::cout << this->Zforces.at(i) << ' ';} std::cout << "\n \n \n";  
    //           this->addForce(contact->wrench[i].body2Force[2]);
    //           this->trimForceVector(0.1);
    //           ROS_INFO_THROTTLE(0.01,"size is: %d || average force %f", this->Zforces.size() ,this->movingTimedAverage());
    //           // unlockingForce = contact->wrench[i].body2Force[2];
    //           if (this->movingTimedAverage() > 80 && this->Zforces.size() > 10){
    //             ROS_INFO_THROTTLE(1,"unlocking at force: %f and size %i", contact->wrench[i].body2Force[2], this->Zforces.size());
    //             this->unfreezeJoint(this->prismaticJoint);
    //           }

    //         } else if(contact->collision1->GetLink()->GetName() == "grab_bar_link" && contact->collision2->GetLink()->GetName() != "sensor_plate" ){
    //           // ROS_INFO_THROTTLE(1,"%s ", contact->collision2->GetLink()->GetName().c_str());

    //           // for(int i=0; i < this->Zforces.size(); i++){std::cout << this->Zforces.at(i) << ' ';} std::cout << "\n \n \n"; 
    //           this->addForce(contact->wrench[i].body1Force[2]);
    //           this->trimForceVector(0.1);

    //           ROS_INFO_THROTTLE(0.01,"size is: %d || average force %f", this->Zforces.size() ,this->movingTimedAverage());

    //           if (this->movingTimedAverage() > 80 && this->Zforces.size() > 10){
    //             ROS_INFO_THROTTLE(1,"unlocking at force: %f and size %i ", contact->wrench[i].body2Force[2], this->Zforces.size());
    //             this->unfreezeJoint(this->prismaticJoint);
    //           }
    //           // unlockingForce = contact->wrench[i].body1Force[2];
    //           // if (unlockingForce > 10 ){
    //           //   ROS_INFO_THROTTLE(1,"unlocking at force: %f ", contact->wrench[i].body1Force[2]);
    //           //   this->unfreezeJoint(this->prismaticJoint);
    //           // }
    //         }


    //       }
    //     }
    //   }
    // }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldUuvPlugin)
} // namespace gazebo
