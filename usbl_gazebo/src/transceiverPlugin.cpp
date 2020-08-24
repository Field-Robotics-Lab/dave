#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/transport/Node.hh>
#include <ignition/math/Pose3.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Vector3.h>

#include <thread>
#include <math.h>
#include <mutex>
#include <vector>
#include <algorithm>
#include <functional>

std::vector<std::string> im = {"common", "individual"}; // available interrogation modes

namespace gazebo
{
    class TransceiverPlugin : public ModelPlugin
    {   
        // default temperature = 10 degrees Celsius
        public: TransceiverPlugin(): m_temperature(10.0) {}

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
            
            this->m_attr.entityName = _sdf->Get<std::string>("entity");
            gzmsg << "Entity: " << this->m_attr.entityName << std::endl;

            // Grab namespace from SDF
            if(!_sdf->HasElement("namespace"))
            {
                gzerr << "Missing required parameter <namespace>, plugin will not be initialized." << std::endl;
                return;
            }
            this->m_attr.entityNamespace = _sdf->Get<std::string>("namespace");

            // enable automation of sending pings to transponder 
            if(!_sdf->HasElement("enable"))
            {
                gzerr << "Missing required parameter <enable>, plugin will not be initialized." << std::endl;
                return;
            }
            
            this->m_attr.enablePinger = _sdf->Get<bool>("enable");
            gzmsg << "pinger enable? " << this->m_attr.enablePinger << std::endl; 

            // Get object that transponder attached to
            if(!_sdf->HasElement("transponder_object"))
            {
                gzerr << "Missing required parameter <transponder_object>, plugin will not be initialized." << std::endl;
                return;
            }
            this->m_attr.transponderName = _sdf->Get<std::string>("transponder_object");

            /*  interrogation mode - 2 options
             *  II (individual interrogation) <----->  CRS (common response signal)
             *  CI (common interrogation)     <----->  IRS (individual response signal) from beacon_01
             *                                    ͱ->  IRS from beacon_02
             *                                    ͱ->  IRS from beacon_03
             *                                            ⋮
             */ 
            if(_sdf->HasElement("interrogation_mode"))
            {
                std::string interrogation_mode = _sdf->Get<std::string>("interrogation_mode");
                if(std::find(im.begin(), im.end(), interrogation_mode) != im.end())
                {
                    gzmsg << interrogation_mode << " interrogation mode is used" << std::endl;
                    this->m_attr.interrogationMode = interrogation_mode;
                }
                else
                {
                    gzmsg << "Specified interrogation mode is unavailable, Common mode is used" << std::endl;
                    this->m_attr.interrogationMode = "common";
                }
            }
            else
            {
                gzmsg << "Interrogation mode is not specified, Common mode is used" << std::endl;
                this->m_attr.interrogationMode = "common";
            }

            // store this entity model
            this->m_model = _model;

            // initialize Gazebo node
            this->m_gzNode = transport::NodePtr(new transport::Node());
            this->m_gzNode->Init();

            // Gazebo subscriber for getting position of the transponder
            std::string transponder_position = "/" + this->m_attr.entityNamespace + "/" + "Transponder" + "/global_position";
            this->m_transponderPoseSub = this->m_gzNode->Subscribe(transponder_position, &TransceiverPlugin::receiveGezeboCallback, this);

            // ROS node initialization
            this->m_rosNode.reset(new ros::NodeHandle(this->m_attr.entityName));

            // ROS publisher for broadcasting beacon's relative location to the transceiver
            std::string beacon_location_topic = "/" + this->m_attr.entityNamespace + "/" + this->m_attr.entityName + "/beacon_location";
            this->m_publishTransponderRelPos = this->m_rosNode->advertise<geometry_msgs::Vector3>(beacon_location_topic, 1);

            // ROS publisher for pinging the transponder
            std::string cis_pinger_topic = "/" + m_attr.entityNamespace + "/common_interrogation_ping";
            this->m_cisPinger = this->m_rosNode->advertise<std_msgs::String>(cis_pinger_topic, 1);

            // create ROS subscriber for temperature
            this->m_rosNode.reset(new ros::NodeHandle(m_attr.entityName));

            // subscriber for temperature sensor
            ros::SubscribeOptions temperatureSub = 
                ros::SubscribeOptions::create<std_msgs::Float64>(
                    "/" + this->m_attr.entityNamespace + "/" + this->m_attr.entityName + "/temperature",
                    1,
                    boost::bind(&TransceiverPlugin::temperatureRosCallback, this, _1),
                    ros::VoidPtr(), &this->m_rosQueue);
                
            this->m_temperatureSub = this->m_rosNode->subscribe(temperatureSub);
            
            // subscriber for setting interrogation mode
            ros::SubscribeOptions interrogationModeSub = 
                ros::SubscribeOptions::create<std_msgs::String>(
                    "/" + this->m_attr.entityNamespace + "/" + this->m_attr.entityName + "/interrogation_mode",
                    1,
                    boost::bind(&TransceiverPlugin::interrogationModeRosCB, this, _1),
                    ros::VoidPtr(), &this->m_rosQueue);
                
            this->m_interrogationModeSub = this->m_rosNode->subscribe(interrogationModeSub);

            // timer to send ping Command
            if(this->m_attr.enablePinger)
            {
                this->m_timer = this->m_rosNode->createTimer(ros::Duration(1.0), &TransceiverPlugin::ping, this);
            }

            this->m_rosQueueThread = std::thread(std::bind(&TransceiverPlugin::queueThread, this));
        }

        // publish ROS ping topic to get response from transponder depending on the interrogation modes
        public: void ping(const ros::TimerEvent&)
        {
            std_msgs::String ping_msg;
            ping_msg.data = "ping";

            // need to fake the transmission by applying distance based delay
            physics::ModelPtr tranponder = this->m_model->GetWorld()->ModelByName(this->m_attr.transponderName);
            double dist = (this->m_model->WorldPose().Pos() - tranponder->WorldPose().Pos()).Length();
            gzmsg << "distance to tranponder: " << dist << " m\n";
            sleep(dist/this->m_soundSpeed);

            if(this->m_attr.interrogationMode.compare("common") == 0)
            {
                this->m_cisPinger.publish(ping_msg);
            }
            else if(this->m_attr.interrogationMode.compare("individual") == 0)
            {
                this->m_iisPinger.publish(ping_msg);
            }
            else
            {
                 gzmsg << "Interrogation mode not recognized\n";
            }
        }

        public: void channelSwitch(const std::string &channel)
        {
            std::string iis_pinger_topic = "/" + this->m_attr.entityNamespace + "/" + channel + "/individual_interrogation_ping";
            this->m_iisPinger = this->m_rosNode->advertise<std_msgs::String>(iis_pinger_topic, 1);
        }

        // gets temperature from sensor to adjust sound speed
        public: void temperatureRosCallback(const std_msgs::Float64ConstPtr &msg)
        {
            this->m_temperature = msg->data;
            auto my_pos = this->m_model->WorldPose();

            // Base on https://dosits.org/tutorials/science/tutorial-speed/
            this->m_soundSpeed = 1540.4 + my_pos.Pos().Z() / 1000 * 17 + (this->m_temperature - 10) * 4;
            gzmsg << "Detected change of temperature, sound speed is now: " << this->m_soundSpeed << " m/s\n";
        }

        public: void interrogationModeRosCB(const std_msgs::StringConstPtr &msg)
        {
            std::string mode = msg->data;
            if(std::find(im.begin(), im.end(), mode) != im.end())
            {
                this->m_attr.interrogationMode = mode;
            }
            else
            {
                gzmsg << "The input mode is not available\n";
            }
            
        }
        
        // Gazebo callback for receiving transponder position, simulating Transceiver's positioning calculation
        public: void receiveGezeboCallback(ConstVector3dPtr& beacon_position)
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

        public: void queueThread()
        {
            static const double timeout = 0.01;
            while (this->m_rosNode->ok())
            {
                this->m_rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        // This entity's attributes
        private: struct attribute{
            std::string transponderName;
            std::string entityNamespace;
            std::string entityName;
            std::string interrogationMode;
            bool enablePinger;
        } m_attr;

        private: double m_temperature;
        private: double m_soundSpeed;

        // Gazebo nodes, publishers, and subscribers
        private: ros::Timer m_timer;
        private: physics::ModelPtr m_model;
        private: transport::NodePtr m_gzNode;
        private: transport::SubscriberPtr m_transponderPoseSub;

        // ROS nodes, publishers and subscibers
        private: std::unique_ptr<ros::NodeHandle> m_rosNode;
        private: ros::Publisher m_publishTransponderRelPos;
        private: ros::Publisher m_cisPinger;
        private: ros::Publisher m_iisPinger;
        private: ros::Subscriber m_temperatureSub;
        private: ros::Subscriber m_interrogationModeSub;
        private: ros::CallbackQueue m_rosQueue;

        private: std::thread m_rosQueueThread;


    };
GZ_REGISTER_MODEL_PLUGIN(TransceiverPlugin)
}
