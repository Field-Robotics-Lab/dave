#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/transport/Node.hh>
#include <ignition/math/Pose3.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <thread>
#include <random>

namespace gazebo
{
    class TransponderPlugin : public ModelPlugin
    {
        public: TransponderPlugin(): m_temperature(10.0), m_noiseMu(0), m_noiseSigma(1) {}

        public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {   
            // Ensure ROS is initialized for publishers and subscribers
            if(!ros::isInitialized())
            {
                gzerr << "ROS has not been inintialized\n";
                int argc = 0;
                char** argv = NULL;
                ros::init(argc, argv, "transponder", ros::init_options::NoSigintHandler);
                return;
            }

            // Obtain Entity name from SDF
            if(!_sdf->HasElement("entity"))
            {
                gzerr << "Missing required parameter <entity>, plugin will not be initialized." << std::endl;
                return;
            }
            this->m_entityName = _sdf->Get<std::string>("entity");
            gzmsg << "Entity: " << this->m_entityName << std::endl;

            // Grab namespace from SDF
            if(!_sdf->HasElement("namespace"))
            {
                gzerr << "Missing required parameter <namespace>, plugin will not be initialized." << std::endl;
                return;
            }
            this->m_namespace = _sdf->Get<std::string>("namespace");

            // get the mean of normal distribution for the noise model
            if(_sdf->HasElement("mu"))
            {
                this->m_noiseMu = _sdf->Get<double>("mu");
            }

            // get the standard deviation of normal distribution for the noise model
            if(_sdf->HasElement("sigma"))
            {
                this->m_noiseSigma = _sdf->Get<double>("sigma");
            }

                
            // store this entity model
            this->m_model = _model;

            // initialize Gazebo node
            this->m_gzNode = transport::NodePtr(new transport::Node());
            this->m_gzNode->Init();

            // define Gazebo publisher for entity's global position
            this->m_globalPosPub = this->m_gzNode->Advertise<msgs::Vector3d>("/" + this->m_namespace + "/" + this->m_entityName + "/global_position");

            this->m_rosNode.reset(new ros::NodeHandle(m_entityName));

            ros::SubscribeOptions iis_ping = 
                ros::SubscribeOptions::create<std_msgs::String>(
                    "/" + this->m_namespace + "/" + this->m_entityName + "/individual_interrogation_ping",
                    1,
                    boost::bind(&TransponderPlugin::IIS_rosCB, this, _1),
                    ros::VoidPtr(), &this->m_rosQueue);
                
            this->m_iisSub = this->m_rosNode->subscribe(iis_ping);

            ros::SubscribeOptions cis_ping = 
                ros::SubscribeOptions::create<std_msgs::String>(
                    "/" + this->m_namespace + "/common_interrogation_ping",
                    1,
                    boost::bind(&TransponderPlugin::CIS_rosCB, this, _1),
                    ros::VoidPtr(), &this->m_rosQueue);
                
            this->m_cisSub = this->m_rosNode->subscribe(cis_ping);

            // create ROS subscriber for temperature
            this->m_rosNode.reset(new ros::NodeHandle(m_entityName));

            ros::SubscribeOptions temperature_sub = 
                ros::SubscribeOptions::create<std_msgs::Float64>(
                    "/" + this->m_namespace + "/" + this->m_entityName + "/temperature",
                    1,
                    boost::bind(&TransponderPlugin::Temperature_rosCB, this, _1),
                    ros::VoidPtr(), &this->m_rosQueue);
                
            this->m_temperatureSub = this->m_rosNode->subscribe(temperature_sub);

            this->m_rosQueueThread = std::thread(std::bind(&TransponderPlugin::QueueThread, this));
        }

        // currently publish noisy position to gazebo topic
        private: void Send()
        { 
            // randomly generate from normal distribution for noise
            std::random_device rd{};
            std::mt19937 gen{rd()};
            std::normal_distribution<> d(0, 1);

            // Gazebo publishing transponder's position with noise and delay
            auto curr_pose = this->m_model->WorldPose();
            ignition::math::Vector3<double> position = curr_pose.Pos();
            auto pub_msg = msgs::Vector3d();
            std::cout << position.X() << " " << position.Y() << " " << position.Z() << std::endl;
            pub_msg.set_x(position.X() + d(gen));
            pub_msg.set_y(position.Y() + d(gen));
            pub_msg.set_z(position.Z() + d(gen));
            this->m_globalPosPub->Publish(pub_msg);
        }

        // gets temperature from sensor to adjust sound speed
        private: void Temperature_rosCB(const std_msgs::Float64ConstPtr &msg)
        {
            this->m_temperature = msg->data;
            auto my_pos = this->m_model->WorldPose();

            // Base on https://dosits.org/tutorials/science/tutorial-speed/
            this->m_soundSpeed = 1540.4 + my_pos.Pos().Z() / 1000 * 17 + (this->m_temperature - 10) * 4;
            gzmsg << "Detected change of temperature, transponder sound speed is now: " << this->m_soundSpeed << " m/s\n";
        }

        // receives ping from transponder and call Send()
        private: void IIS_rosCB(const std_msgs::StringConstPtr &msg)
        {
            auto box = this->m_model->GetWorld()->ModelByName("box");
            double dist = (this->m_model->WorldPose().Pos() - box->WorldPose().Pos()).Length();
            std::string command = msg->data;
            if(!command.compare("ping"))
            {
                gzmsg << "Received iis_ping, responding\n";
                sleep(dist / this->m_soundSpeed);
                Send();
            }
            else
            {
                gzmsg << "Unknown command, ignore\n";
            }

        }

        // receives ping from transponder and call Send()
        private: void CIS_rosCB(const std_msgs::StringConstPtr &msg)
        {
            auto box = this->m_model->GetWorld()->ModelByName("box");
            double dist = (this->m_model->WorldPose().Pos() - box->WorldPose().Pos()).Length();
            std::string command = msg->data;
            if(!command.compare("ping"))
            {
                gzmsg << "Received cis_ping, responding\n";
                sleep(dist / this->m_soundSpeed);
                Send();
            }
            else
            {
                gzmsg << "Unknown command, ignore\n";
            }

        }

        void QueueThread()
        {
            static const double timeout = 0.01;
            while (this->m_rosNode->ok())
            {
                this->m_rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        private: double m_temperature;
        private: double m_soundSpeed;
        private: double m_noiseMu;
        private: double m_noiseSigma;

        private: std::string m_entityName;
        private: std::string m_namespace;
        private: transport::NodePtr m_gzNode;
        private: transport::PublisherPtr m_globalPosPub;
        private: physics::ModelPtr m_model;

        private: ros::Subscriber m_iisSub;
        private: ros::Subscriber m_cisSub;
        private: ros::CallbackQueue m_rosQueue;
        private: ros::Subscriber m_temperatureSub;

        private: std::unique_ptr<ros::NodeHandle> m_rosNode;
        private: std::thread m_rosQueueThread;
    };


GZ_REGISTER_MODEL_PLUGIN(TransponderPlugin)

}