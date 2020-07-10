#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <string_view>

namespace gazebo
{
  class WorldUuvMoveJointPlugin : public WorldPlugin
  {
  private: physics::WorldPtr world;

  private: physics::ModelPtr plugModel;

  private: physics::LinkPtr plugLink;

  private: gazebo::event::ConnectionPtr updateConnection;


  public: WorldUuvMoveJointPlugin() : WorldPlugin(){}

  public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
      this->world = _world;
      this->plugModel = this->world->ModelByName("electrical_plug_socket_system");
      this->plugLink = this->plugModel->GetLink("plug");
      printf("IN IN IN  \n");


      this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
          std::bind(&WorldUuvMoveJointPlugin::Update, this));
    }

  public: void Update()
    {
       this->plugModel->GetJoint("electrical_joint")->SetVelocity(0, 0.1);
    }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldUuvMoveJointPlugin)
} // namespace gazebo
