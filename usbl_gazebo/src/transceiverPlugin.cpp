#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/transport/Node.hh>
#include <ignition/math/Pose3.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>

#include <thread>
#include <math.h>
#include <mutex>
#include <tuple>

namespace gazebo
{
    class TransceiverPlugin : public ModelPlugin
    {   
        // default temperature = 10 degrees Celsius
        public: TransceiverPlugin():m_temperature(10.0) {}

        public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Ensure ROS is initialized for publishers and subscribers
            if(!ros::isInitialized())
            {
                gzerr << "ROS has not been initialized\n";
                int argc = 0;
                char** argv = NULL;
                ros::init(argc, argv, "USBL_transceiver", ros::init_options::NoSigintHandler);
                return ;
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

            // enable automation of sending pings to transponder 
            if(!_sdf->HasElement("enable"))
            {
                gzerr << "Missing required parameter <enable>, plugin will not be initialized." << std::endl;
                return;
            }
            
            this->m_enablePinger = _sdf->Get<bool>("enable");
            gzmsg << "pinger enable? " << this->m_enablePinger << std::endl; 

            // Get object that transponder attached to
            if(!_sdf->HasElement("transponder_object"))
            {
                gzerr << "Missing required parameter <transponder_object>, plugin will not be initialized." << std::endl;
                return;
            }
            this->m_transponderName = _sdf->Get<std::string>("transponder_object");


            // store this entity model
            this->m_model = _model;

            // initialize Gazebo node
            this->m_gzNode = transport::NodePtr(new transport::Node());
            this->m_gzNode->Init();

            // Gazebo subscriber for getting position of the transponder
            std::string transponderPos = "/" + this->m_namespace + "/" + "Transponder" + "/global_position";
            this->m_transponderPoseSub = this->m_gzNode->Subscribe(transponderPos, &TransceiverPlugin::Receive_gzCB, this);

            // ROS node initialization
            this->m_rosNode.reset(new ros::NodeHandle(this->m_entityName));

            // ROS publisher for broadcasting beacon's relative location to the transceiver
            std::string beaconLocationTopic = "/" + this->m_namespace + "/" + this->m_entityName + "/beacon_location";
            this->m_publishTransponderRelPos = this->m_rosNode->advertise<geometry_msgs::Vector3>(beaconLocationTopic, 1);

            // ROS publisher for pinging the transponder
            std::string pingerTopic = "/" + this->m_namespace + "/" + "Transponder" + "/command";
            this->m_publishPinger = this->m_rosNode->advertise<std_msgs::String>(pingerTopic, 1);

            // create ROS subscriber for temperature
            this->m_rosNode.reset(new ros::NodeHandle(m_entityName));

            ros::SubscribeOptions temperature_sub = 
                ros::SubscribeOptions::create<std_msgs::Float64>(
                    "/" + this->m_namespace + "/" + this->m_entityName + "/temperature",
                    1,
                    boost::bind(&TransceiverPlugin::Temperature_rosCB, this, _1),
                    ros::VoidPtr(), &this->m_rosQueue);
                
            this->m_temperatureSub = this->m_rosNode->subscribe(temperature_sub);
            
            // timer to send ping Command
            if(this->m_enablePinger)
            {
                this->m_timer = this->m_rosNode->createTimer(ros::Duration(1.0), &TransceiverPlugin::Ping, this);
            }

            this->m_rosQueueThread = std::thread(std::bind(&TransceiverPlugin::QueueThread, this));
        }

        public: void Ping(const ros::TimerEvent&)
        {
            std_msgs::String ping_msg;
            ping_msg.data = "ping";

            physics::ModelPtr tranponder = this->m_model->GetWorld()->ModelByName(this->m_transponderName);
            double dist = (this->m_model->WorldPose().Pos() - tranponder->WorldPose().Pos()).Length();
            gzmsg << "distance to tranponder: " << dist << " m\n";
            sleep(dist/this->m_soundSpeed);

            this->m_publishPinger.publish(ping_msg);
        }

        public: void Temperature_rosCB(const std_msgs::Float64ConstPtr &msg)
        {
            this->m_temperature = msg->data;
            auto my_pos = this->m_model->WorldPose();

            // Base on https://dosits.org/tutorials/science/tutorial-speed/
            this->m_soundSpeed = 1540.4 + my_pos.Pos().Z() / 1000 * 17 + (this->m_temperature - 10) * 4;
            gzmsg << "Detected change of temperature, sound speed is now: " << this->m_soundSpeed << " m/s\n";
        }
        
        // Gazebo callback for receiving transponder position, simulating Transceiver's positioning calculation
        public: void Receive_gzCB(ConstVector3dPtr& beacon_position)
        {
            gzmsg << "Transceiver acquires transponders position: " << beacon_position->x() << " " << beacon_position->y() << " " << beacon_position->z() << std::endl;
            
            ignition::math::Vector3d beacon_position_ign = ignition::math::Vector3d(beacon_position->x(), beacon_position->y(), beacon_position->z());
            
            double bearing=0, range=0, elevation=0;
            calcuateRelativePose(beacon_position_ign, bearing, range, elevation);

            publishPosition(bearing, range, elevation);
        }

        // publish transponder's relative position in spherical coordinate(range, bearing, elevation)
        public: void publishPosition(double &bearing, double &range, double &elevation)
        {
            geometry_msgs::Vector3 location;
            location.x = bearing;
            location.y = range;
            location.z = elevation;


            this->m_publishTransponderRelPos.publish(location);
        }

        public: void calcuateRelativePose(ignition::math::Vector3d position, double &bearing, double &range, double &elevation)
        {
            
            auto my_pos = this->m_model->WorldPose();
            auto direction = position - my_pos.Pos();
            
            ignition::math::Vector3d directionTransceiverFrame = my_pos.Rot().RotateVectorReverse(direction);

            ignition::math::Vector3d directionTransceiverFrame2d = ignition::math::Vector3d(directionTransceiverFrame.X(), directionTransceiverFrame.Y(), 0);
            
            bearing = atan2(directionTransceiverFrame.Y(), directionTransceiverFrame.X()) * 180 / 3.1415926;
            range = directionTransceiverFrame.Length();
            elevation = atan2(directionTransceiverFrame.Z(), directionTransceiverFrame.Length()) * 180 / 3.1415926;

            gzmsg << "bearing: " << bearing << "\n";
            gzmsg << "range: " << range << "\n";
            gzmsg << "elevation: " << elevation << "\n\n";
        }

        public: void QueueThread()
        {
            static const double timeout = 0.01;
            while (this->m_rosNode->ok())
            {
                this->m_rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        // This entity's attributes
        private: double m_temperature;
        private: double m_soundSpeed;
        private: std::string m_transponderName;
        private: std::string m_namespace;
        private: std::string m_entityName;
        private: bool m_enablePinger;

        // Gazebo nodes, publishers, and subscribers
        private: event::ConnectionPtr m_updateConnect;
        private: ros::Timer m_timer;
        private: physics::ModelPtr m_model;
        private: transport::NodePtr m_gzNode;
        private: transport::SubscriberPtr m_transponderPoseSub;

        // ROS nodes, publishers and subscibers
        private: std::unique_ptr<ros::NodeHandle> m_rosNode;
        private: ros::Publisher m_publishTransponderRelPos;
        private: ros::Publisher m_publishPinger;
        private: ros::Subscriber m_temperatureSub;
        private: ros::CallbackQueue m_rosQueue;

        private: std::thread m_rosQueueThread;


    };
GZ_REGISTER_MODEL_PLUGIN(TransceiverPlugin)
}
