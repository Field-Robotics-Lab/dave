#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/Collision.hh>
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

  private: physics::WorldPtr world;

  private: physics::ModelPtr plugModel;

  private: physics::LinkPtr plugLink;

  private: physics::LinkPtr tubeLink;

  private: physics::ModelPtr socketModel;

  private: physics::LinkPtr sensorPlate;

  private: ignition::math::Pose3d socket_pose;

  private: ignition::math::Pose3d plug_pose;

  private: physics::JointPtr prismaticJoint;

  public: ignition::math::Vector3d grabAxis;

  public: ignition::math::Vector3d someforce;

  private: bool joined = false;
  
  private: bool locked = false;

  private: gazebo::event::ConnectionPtr updateConnection;

  public: ros::Publisher chatter_pub;

  private: std::unique_ptr<ros::NodeHandle> rosNode;

  public: std::string collisionTopic;

  private: gazebo::transport::SubscriberPtr collisionSub;

  public: physics::ContactManager* _contactManagerPtr;

  public:  physics::ContactPtr _contactPtr;

  public: int collisionForceCount = 0;
  
  std::vector<double> Zforces;
  
  std::vector<common::Time> timeStamps;

  public: common::Time alignmentTime = 0;
  
  public: common::Time unfreezeTimeBuffer = 0;
  
  public: bool recentlyUnfrozen = false;

  public: int ccount = 0;

 
    void trimForceVector(double trimDuration){
        std::vector<common::Time>::iterator low;
        if (this->timeStamps.size() == 0){
            return;
        }
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
      // this->socketModel = this->world->ModelByName("bop_panel_With_socket");
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

  public: void lockJoint(physics::JointPtr prismaticJoint){
      if (this->locked){
        ROS_INFO("already locked!");
        return;
      }
      this->locked = true;
      ROS_INFO("locked!");
      double currentPosition = prismaticJoint->Position(0);
      prismaticJoint->SetUpperLimit(0, currentPosition);
      prismaticJoint->SetLowerLimit(0, currentPosition);
  }

  public: void unfreezeJoint(physics::JointPtr prismaticJoint){
      if (!this->locked){
        ROS_INFO("already unlocked");
        return;
      }
      this->recentlyUnfrozen = true;
      this->locked = false;
      this->unfreezeTimeBuffer =  this->world->SimTime();
      ROS_INFO("UNFREEZE");
      // prismaticJoint->SetUpperLimit(0, 100);
      // prismaticJoint->SetLowerLimit(0, -100);
      this->remove_joint();
  }

 private: bool checkRollAlignment(bool verbose = false){
      ignition::math::Vector3<double> socketRotation = socketModel->RelativePose().Rot().Euler();
      ignition::math::Vector3<double> plugRotation = plugModel->RelativePose().Rot().Euler();
      if (verbose){
        ROS_INFO_THROTTLE(1, "socket euler: %.2f %.2f %.2f plug euler: %.2f %.2f %.2f", socketRotation[0], socketRotation[1], socketRotation[2],plugRotation[0],plugRotation[1],plugRotation[2]+1.57079632679  );
      }
      return abs(plugRotation[0] - socketRotation[0]) < 0.3;
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
      return abs(plugRotation[2]+1.57079632679 - socketRotation[2]) < 0.2;
    }

  private: bool checkRotationalAlignment()
    {
      if (this->checkYawAlignment(true) && this->checkPitchAlignment() && this->checkRollAlignment())
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
      // socket_pose = socketModel->RelativePose();
      socket_pose = this->tubeLink->WorldPose();
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

  private: bool isAlligned(bool verbose = false)
    {
      if(checkVerticalAlignment(true) && checkRotationalAlignment()){
        if (verbose){ROS_INFO_THROTTLE(1,"ALLIGNED ROT and VERT");}
        return true;
      } else {
        return false;
      }
    }

  private: bool checkProximity(bool verbose = false)
    {
      socket_pose = this->tubeLink->WorldPose();
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

      bool withinProximity = pow(xdiff_squared+ydiff_squared+zdiff_squared,0.5) < 0.14;
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
      if (this->joined){
          ROS_INFO_THROTTLE(1,"already frozen");
        return;
      }
      this->joined = true;
      this->alignmentTime = 0;
      this->prismaticJoint = plugModel->CreateJoint(
        "plug_socket_joint",
        "prismatic",
        tubeLink,
        plugLink);
      prismaticJoint->Load(this->tubeLink, this->plugLink,
                            ignition::math::Pose3<double>(ignition::math::Vector3<double>(0, 0, 0),
                                                          ignition::math::Quaternion<double>(0, 0, 0, 0)));
      prismaticJoint->Init();
      prismaticJoint->SetAxis(0, ignition::math::Vector3<double>(1, 0, 0));
      ROS_INFO("joint formed\n");
  }
    
  private: void remove_joint(){
      if (this->joined == true){
      
      this->joined = false;
      this->prismaticJoint->Detach();
      this->prismaticJoint->Reset();
      this->prismaticJoint->~Joint();
      printf("joint removed\n");
      }

  }


    /// \brief Calculates the average force exerted by contact2 on contact 1
    /// \return Average force exerted by contact2 on contact1
  public: bool averageForceOnLink(std::string contact1, std::string contact2) {
            int contactIndex = this->getCollisionBetween(contact1, contact2);
            if (contactIndex == -1) {
                return false;
            }
            physics::Contact *contact = this->world->Physics()->GetContactManager()->GetContact(contactIndex);
            if (contact->collision1->GetLink()->GetName() == contact1) {
                this->addForce(contact->wrench[contactIndex].body1Force[1]);
            } else {
                this->addForce(contact->wrench[contactIndex].body2Force[1]);
            }
            this->trimForceVector(0.1);
            return true;
        }
        /// \brief Determine of Electrical Plug is pushing against electrical socket
        /// \return boolean weather the plug is pushing against the socket
  public: bool isPlugPushingSensorPlate(float averageForceThreshold = 50, int numberOfDatapointsThresh = 10) {
            if (!this->averageForceOnLink("plug", "sensor_plate")) {
                return false;
            } else {
                double averageForce = this->movingTimedAverage();
                if ((averageForce > averageForceThreshold) && (this->Zforces.size() > numberOfDatapointsThresh)) {
                    ROS_INFO("sensor plate average: %f, size %i", averageForce, this->Zforces.size());
                    this->Zforces.clear();
                    this->timeStamps.clear();
                    return true;
                }
            }
        }
            /// \brief Determine of Electrical Plug is pushing against electrical socket
            /// \return boolean weather the plug is pushing against the socket
  public: bool isEndEffectorPushingPlug(float averageForceThreshold = 200, int numberOfDatapointsThresh = 10){
            if (!this->averageForceOnLink("plug", "finger_tip")){
                return false;
            } else{
                double averageForce = this->movingTimedAverage();
                if ( (averageForce > averageForceThreshold) && (this->Zforces.size() > numberOfDatapointsThresh)){
                    ROS_INFO("end effector average: %f, size %i",averageForce, this->Zforces.size());
                    this->Zforces.clear();
                    this->timeStamps.clear();
                    return true;
                }
            }
  }



  public: int getCollisionBetween(std::string contact1, std::string contact2){
          for(int i=0; i<this->world->Physics()->GetContactManager()->GetContactCount(); i++){
              physics::Contact *contact = this->world->Physics()->GetContactManager()->GetContact(i);
              bool isPlugContactingSensorPlate =
                      (contact->collision1->GetLink()->GetName().find(contact1) != std::string::npos) &&
                      (contact->collision2->GetLink()->GetName().find(contact2) != std::string::npos) ||
                      (contact->collision1->GetLink()->GetName().find(contact2) != std::string::npos) &&
                      (contact->collision2->GetLink()->GetName().find(contact1) != std::string::npos);

                if ((contact->collision1->GetLink()->GetName().find(contact1) != std::string::npos) &&
                    (contact->collision2->GetLink()->GetName().find(contact2) != std::string::npos)){
                    ROS_INFO_THROTTLE(1,
                                      "%s %s", contact->collision1->GetLink()->GetName().c_str(),
                                      contact->collision2->GetLink()->GetName().c_str());

                } else if ((contact->collision1->GetLink()->GetName().find(contact2) != std::string::npos) &&
                           (contact->collision2->GetLink()->GetName().find(contact1) != std::string::npos)){
                    ROS_INFO_THROTTLE(1,
                                      "%s %s", contact->collision1->GetLink()->GetName().c_str(),
                                      contact->collision2->GetLink()->GetName().c_str());
                }



              if (isPlugContactingSensorPlate){
                  return i;
              }
          }
          return -1;
      }

  public: void Update(){

      //check if recently removed the joint (to avoid it locking right away after unlocked)
      if (this->world->SimTime() - unfreezeTimeBuffer < 2){
          return;
      }
      // If plug and socket are not joined yet, check the alignment
      // between them, and if alignment is maintained for more than
      // 2 seconds, then construct a joint between them
    if (!this->joined){
      if (this->isAlligned() && this->checkProximity(true)){
          if (alignmentTime == 0){
            alignmentTime = this->world->SimTime();
          } else if (this->world->SimTime() - alignmentTime > 2){
              this->construct_joint();
          }
      } else {
            alignmentTime = 0;
      }
      // If joint is constructed but plug is not yet fixed/locked
      // into the socket, measure the forces and lock the plug to
      // the socket if the plug is pushing against it.
    } else if (this->joined && !this->locked){
        if(this->isPlugPushingSensorPlate()){
            this->lockJoint(this->prismaticJoint);
        }
      //If plug is locked to socket, probe to see if there is
      // enough force being exerted to pull it out
    } else if (this->joined && this->locked){
        if(this->isEndEffectorPushingPlug()){
            this->unfreezeJoint(prismaticJoint);
        }
    }
  }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldUuvPlugin)
} // namespace gazebo
