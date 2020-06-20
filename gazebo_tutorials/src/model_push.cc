#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <iostream>
namespace gazebo
{
    class ModelPush : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            // Store the pointer to the model
            this->model = _parent;

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&ModelPush::OnUpdate, this));
        }

        // Called by the world update start event
    public:
        void OnUpdate()
        {
            // Apply a small linear velocity to the model.
            // gazebo::math::Pose pose;
            auto pose = model->WorldPose();
            ignition::math::Vector3<double> position = pose.Pos();

            // ignition::math::Pose3<float> pose;
            // pose = this->model->GetWorldPose();
            // auto position = pose.Pos();
            // std::cout << position[0] <<std::endl;
            ROS_INFO_THROTTLE(1, "%f %f %f", position[0], position[1], position[2]);
            // ROS_INFO(position[0]);
            // ROS_INFO(pose.Pos.x);

            // this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
        }

        // Pointer to the model
    private:
        physics::ModelPtr model;

        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ModelPush)
} // namespace gazebo