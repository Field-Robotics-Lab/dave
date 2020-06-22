#include <gazebo/common/Plugin.hh>

namespace gazebo
{
    class LeadMating : public WorldPlugin
    {
    public:
        LeadMating();

    public:
        virtual ~LeadMating();

    public:
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    public:
        void OnUpdate();

    
    private:
        physics::WorldPtr world;

    };
}
