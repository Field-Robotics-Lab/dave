#include <gazebo/gazebo.hh>

namespace gazebo
{
  class WorldUuvPlugin : public WorldPlugin
  {
  private: physics::WorldPtr world;
  private: physics::ModelPtr socket;
  private: physics::ModelPtr plug;
  private: gazebo::event::ConnectionPtr updateConnection;

  public: WorldUuvPlugin() : WorldPlugin()
            {
              printf("Hello World!\n");
            }

  public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
            {
              this->world = _world;
              this->socket = this->world->GetModel("atlas");
              this->plug = this->world->GetModel("atlas");
              this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
                  std::bind(&WorldUuvPlugin::Update, this));
            }

    public: void Update()
            {
              printf("yo yo yo !\n");
            }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldUuvPlugin)
}
