#include <gazebo/gazebo.hh>
#include <ros/ros.h>

namespace gazebo
{
    class TransponderPlugin : public ModelPlugin
    {
        public:
            void Load(physics::ModelPtr, sdf::ElementPtr _sdf)
            {
                std::cout << "USBL plugin loaded" << std::endl;

                if ( !ros::isInitialized() )
                {
                    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized");
                }
            }
    };

    GZ_REGISTER_MODEL_PLUGIN(TransponderPlugin)
} // namespace gazebo
