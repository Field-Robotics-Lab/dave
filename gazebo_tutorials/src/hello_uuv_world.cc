#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class WorldUuvPlugin : public WorldPlugin
  {
  private: physics::WorldPtr world;

  private: physics::ModelPtr socket;
  private: physics::ModelPtr plug;
  private: ignition::math::Pose3d socket_pose;
  private: ignition::math::Pose3d plug_pose;
  // private: ignition::math::Vector3<float> bla;

  private: gazebo::event::ConnectionPtr updateConnection;

  public: WorldUuvPlugin() : WorldPlugin()
            {
              printf("Hello World!\n");
            }

  public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
            {
              this->world =  _world;
              this->socket = this->world->ModelByName("electrical_plug");
              this->plug = this->world->ModelByName("electrical_socket");
              this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
                  std::bind(&WorldUuvPlugin::Update, this));
            }
  
  // private: bool checkAlignment(physics::ModelPtr plugPointer, physics::ModelPtr socketPointer)
  private: bool checkRollAlignment()
    {
      auto socketRotation = socket->RelativePose().Rot().Euler();
      auto plugRotation = plug->RelativePose().Rot().Euler();
      return abs(plugRotation[0] - socketRotation[0]) < 0.1;
    }
    
  private: bool checkPitchAlignment()
    {
      auto socketRotation = socket->RelativePose().Rot().Euler();
      auto plugRotation = plug->RelativePose().Rot().Euler();
      return abs(plugRotation[1] - socketRotation[1]) < 0.1;
    }
    
  private: bool checkYawAlignment()
    {
      auto socketRotation = socket->RelativePose().Rot().Euler();
      auto plugRotation = plug->RelativePose().Rot().Euler();
      return abs(plugRotation[2] - socketRotation[2]) < 0.1;
    }
    
  private: bool checkRotationalAlignment()
    {
      if (this->checkYawAlignment() && this->checkPitchAlignment() && this->checkRollAlignment()){
        printf("Aligned, ready for insertion  \n");
      } else {
        printf("Disalligned, not ready for mating  \n");
      }
    }

  private: bool checkVerticalAlignment()
    {
      socket_pose = socket->RelativePose();
      auto socketPositon = socket_pose.Pos();

      plug_pose = plug->RelativePose();
      auto plugPosition = plug_pose.Pos();

      bool onSameVerticalLevel = abs(plugPosition[2] - socketPositon[2]) < 0.1;
      if (onSameVerticalLevel)
      {
        // printf("z=%.2f  \n", plugPosition[2]);
        printf("Share same vertical level  \n");
      }
    }

  public: void Update()
    {
      // this->checkVerticalAlignment();
      this->checkRotationalAlignment();
    }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldUuvPlugin)
}
