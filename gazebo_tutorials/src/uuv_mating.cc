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


  public: WorldUuvPlugin() : WorldPlugin(){}

  public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
      this->world = _world;
      this->socket = this->world->ModelByName("electrical_socket");
      this->plug = this->world->ModelByName("electrical_plug");
      this->elecs = this->socket->GetLink("elecs");
      this->elecp = this->plug->GetLink("elecp");

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
        // printf("z=%.2f  \n", plugPosition[2]);
        // printf("Share same vertical level  \n");
        return true;
      }
      return false;
    }

  public: void Update()
    {
      // connect the socket and the plug after 5 seconds
      if (this->world->SimTime() > 2.0 && joined == false)
      {
        this->joined = true;
        prismaticJoint = world->Physics()->CreateJoint("prismatic");
        // prismaticJoint->SetName(this->elecs->GetName() + std::string("_") +
        //               this->elecp->GetName() + std::string("_joint"));
        // prismaticJoint = world->Physics()->CreateJoint("prismatic", this->socket);
        // prismaticJoint->Attach(this->elecs, this->elecp);
        prismaticJoint->Load(this->elecs, this->elecp, 
          ignition::math::Pose3<double>(ignition::math::Vector3<double>(1, 0, 0), 
          ignition::math::Quaternion<double>(0, 0, 0, 0)));
          // ignition::math::Quaternion<double>(0, 0.3428978, 0, 0.9393727)));
        // prismaticJoint->SetAxis(0, ignition::math::Vector3<double>(1, 0, 0));
        prismaticJoint->SetUpperLimit(0, 2);
        prismaticJoint->SetLowerLimit(0, -0.1);
        prismaticJoint->Init();
      }

      if (joined){
        prismaticJoint->SetVelocity(0, 0.1);
      }

      // this->checkVerticalAlignment();
      // this->checkRotationalAlignment();
    }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldUuvPlugin)
} // namespace gazebo
