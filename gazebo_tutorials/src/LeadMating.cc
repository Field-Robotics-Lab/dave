#include <gazebo/physics/physics.hh>
#include "../include/LeadMating.hh"

using namespace gazebo;

void LeadMating::Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    this->world = _parent;
    gzlog << "VRCScoringPlugin: world name is \"" << this->world << "\"" << std::endl;
}

void LeadMating::OnUpdate()
{
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(LeadMating)
