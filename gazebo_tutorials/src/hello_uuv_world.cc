#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <string_view>

namespace gazebo
{
  class WorldUuvPlugin : public WorldPlugin
  {
  private: physics::WorldPtr world;

  private: physics::ModelPtr socket;

  private: physics::ModelPtr plug;

  private: physics::LinkPtr elecs;

  private: physics::LinkPtr elecp;

  private: ignition::math::Pose3d socket_pose;

  private: ignition::math::Pose3d plug_pose;

  private: physics::JointPtr prismaticJoint;

  private: bool joined = false;

  private: gazebo::event::ConnectionPtr updateConnection;
  // private: physics::JointPtr AddJoint(physics::WorldPtr _world,
  //                              physics::ModelPtr _model,
  //                              physics::LinkPtr _link1,
  //                              physics::LinkPtr _link2,
  //                              std::string _type,
  //                              ignition::math::Vector3<double> _anchor,
  //                              ignition::math::Vector3<double> _axis,
  //                              double _upper, double _lower,
  //                              bool _disableCollision)
  //   {
  //     physics::JointPtr joint;
  //     joint = _world->Physics()->CreateJoint(_type, _model);
  //     joint->Attach(_link1, _link2);
  //     // load adds the joint to a vector of shared pointers kept
  //     // in parent and child links, preventing joint from being destroyed.
  //     joint->Load(_link1, _link2, ignition::math::Pose3<double>(_anchor, ignition::math::Quaternion<double>()));
  //     joint->SetAxis(0, _axis);
  //     joint->SetUpperLimit(0, _upper);
  //     joint->SetLowerLimit(0, _lower);

  //     if (_link1)
  //       joint->SetName(_link1->GetName() + std::string("_") +
  //                      _link2->GetName() + std::string("_joint"));
  //     else
  //       joint->SetName(std::string("world_") +
  //                      _link2->GetName() + std::string("_joint"));
  //     joint->Init();

  //     // disable collision between the link pair
  //     if (_disableCollision)
  //     {
  //       if (_link1)
  //         _link1->SetCollideMode("fixed");
  //       if (_link2)
  //         _link2->SetCollideMode("fixed");
  //     }

  //     return joint;
  //   }


  public: WorldUuvPlugin() : WorldPlugin(){}

  public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
      this->world = _world;
      this->socket = this->world->ModelByName("electrical_socket");
      this->plug = this->world->ModelByName("electrical_plug");
      // DEBUG
      // if (!(this->socket==NULL)){
      //   printf("got socket  \n");
 
      // } else {
      //   printf("failed socket  \n");

      // }
      this->elecs = this->socket->GetLink("elecs");
      this->elecp = this->plug->GetLink("elecp");

      // DEBUG
      // physics::Link_V links = this->socket->GetLinks();
      // for(physics::Link_V::iterator li = links.begin(); li != links.end(); ++li)
      //   printf("%s  \n",(*li)->GetName().c_str());

      //DEBUG
      // if (!(this->elecp==NULL)){
      //   printf("succed  \n");
 
      // } else {
      //   printf("failed  \n");

      // }

      this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
          std::bind(&WorldUuvPlugin::Update, this));
    }

  private: bool checkRollAlignment(){
      ignition::math::Vector3<double> socketRotation = socket->RelativePose().Rot().Euler();
      ignition::math::Vector3<double> plugRotation = plug->RelativePose().Rot().Euler();
      return abs(plugRotation[0] - socketRotation[0]) < 0.1;
    }

  private: bool checkPitchAlignment(){
      ignition::math::Vector3<double> socketRotation = socket->RelativePose().Rot().Euler();
      ignition::math::Vector3<double> plugRotation = plug->RelativePose().Rot().Euler();
      return abs(plugRotation[1] - socketRotation[1]) < 0.1;
    }

  private:bool checkYawAlignment(){
      ignition::math::Vector3<double> socketRotation = socket->RelativePose().Rot().Euler();
      ignition::math::Vector3<double> plugRotation = plug->RelativePose().Rot().Euler();
      return abs(plugRotation[2] - socketRotation[2]) < 0.1;
    }

  private: bool checkRotationalAlignment()
    {
      if (this->checkYawAlignment() && this->checkPitchAlignment() && this->checkRollAlignment())
      {
        // printf("Aligned, ready for insertion  \n");
        return true;
      }
      else
      {
        // printf("Disaligned, not ready for mating  \n");
        return false;
      }
    }

  private: bool checkVerticalAlignment()
    {
      socket_pose = socket->RelativePose();
      ignition::math::Vector3<double> socketPositon = socket_pose.Pos();
      // printf("%s  \n", typeid(socketPositon).name());

      plug_pose = plug->RelativePose();
      ignition::math::Vector3<double> plugPosition = plug_pose.Pos();

      bool onSameVerticalLevel = abs(plugPosition[2] - socketPositon[2]) < 0.1;
      if (onSameVerticalLevel)
      {
        return true;
        // printf("z=%.2f  \n", plugPosition[2]);
        // printf("Share same vertical level  \n");
      }
      return false;
    }

  public: void Update()
    {
      // connect the socket and the plug after 5 seconds
      if (this->world->SimTime() > 5.0 && joined == false)
      {
        this->joined = true;
        physics::JointPtr joint;
        joint = world->Physics()->CreateJoint("prismatic", this->socket);
        joint->Attach(this->elecs, this->elecp);
        joint->Load(this->elecs, this->elecp, 
          ignition::math::Pose3<double>(ignition::math::Vector3<double>(1, 0, 0), 
          ignition::math::Quaternion<double>()));
        joint->SetAxis(0, ignition::math::Vector3<double>(1, 0, 0));
        joint->SetUpperLimit(0, 20);
        joint->SetLowerLimit(0, -20);
        joint->SetName(this->elecs->GetName() + std::string("_") +
                      this->elecp->GetName() + std::string("_joint"));
        joint->Init();
      }

      this->checkVerticalAlignment();
      this->checkRotationalAlignment();
    }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldUuvPlugin)
} // namespace gazebo
