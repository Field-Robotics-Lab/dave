#include <gazebo/gazebo.hh>

namespace gazebo
{
  class WorldUuvPlugin : public WorldPlugin
  {
    public: WorldUuvPlugin() : WorldPlugin()
            {
              printf("Hello World!\n");
            }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
            {
            }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldUuvPlugin)
}
