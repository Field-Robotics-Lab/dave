/*
 * Copyright 2020 Naval Postgraduate School
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#include <string>
#include <vector>

#include "dave_gazebo_model_plugins/usbl_transceiver_plugin.h"
#include "dave_gazebo_model_plugins/usbl_command_id.h"

using namespace gazebo;

// available interrogation modes
std::vector<std::string> im = {"common", "individual"};

// default temperature = 10 degrees Celsius
TransceiverPlugin::TransceiverPlugin(): m_temperature(10.0) {}

TransceiverPlugin::~TransceiverPlugin() {}

void TransceiverPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    /** SDF PARAMETERS*****************/

    // Ensure ROS is initialized for publishers and subscribers
    if (!ros::isInitialized())
    {
        gzerr << "ROS has not been initialized\n";
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, "Usbl_transceiver",
                  ros::init_options::NoSigintHandler);
        return;
    }

    /*------------------------------------------------------------------------*/
    // Grab namespace from SDF
    if (!_sdf->HasElement("namespace"))
    {
        gzerr << "Missing required parameter <namespace>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }
    this->m_namespace = _sdf->Get<std::string>("namespace");

    /*------------------------------------------------------------------------*/
    // Obtain transceiver device name from SDF
    if (!_sdf->HasElement("transceiver_device"))
    {
        gzerr << "Missing required parameter <transceiver_device>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }

    this->m_transceiverDevice = _sdf->Get<std::string>("transceiver_device");
    gzmsg << "Entity: " << this->m_transceiverDevice << std::endl;

    /*------------------------------------------------------------------------*/
    // get transceiver device id
    if (!_sdf->HasElement("transceiver_ID"))
    {
        gzerr << "Missing required parameter <transceiver_ID>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }

    this->m_transceiverID = _sdf->Get<std::string>("transceiver_ID");

    /*------------------------------------------------------------------------*/
    // get transponder device name
    if (!_sdf->HasElement("transponder_device"))
    {
        gzerr << "Missing required parameter <transponder_device>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }
    this->m_transponderDevice = _sdf->Get<std::string>("transponder_device");
    gzmsg << "Transponder device: " << this->m_transponderDevice << std::endl;

    /*------------------------------------------------------------------------*/
    // get commanding transponders
    if (!_sdf->HasElement("transponder_ID"))
    {
        gzerr << "Missing required parameter <transponder_ID>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }

    auto transponders = ignition::common::Split(_sdf->Get<std::string>(
        "transponder_ID"), ',');
    gzmsg << "Current deployed transponders are: \n";

    for (auto &transponder : transponders)
    {
        gzmsg << transponder << std::endl;
        this->m_deployedTransponders.push_back(transponder);
    }

    /*------------------------------------------------------------------------*/
    // enable automation of sending pings to transponder
    if (!_sdf->HasElement("enable_ping_scheduler"))
    {
        gzerr << "Missing required parameter <enable_ping_scheduler>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }

    this->m_enablePingerScheduler = _sdf->Get<bool>("enable_ping_scheduler");
    gzmsg << "pinger enable? " << this->m_enablePingerScheduler << std::endl;

    /*------------------------------------------------------------------------*/
    // Get object that transponder attached to
    if (!_sdf->HasElement("transponder_attached_object"))
    {
        gzerr << "Missing required parameter <transponder_attached_object>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }
    this->m_transponderAttachedObject = _sdf->Get<std::string>(
        "transponder_attached_object");

    /*------------------------------------------------------------------------*/
    /*  interrogation mode - 2 options
        *  II (individual interrogation) <----->  CRS (common response signal)
        *  CI (common interrogation)     <----->  IRS (individual response
        *                                         signal) from transponder_01
        *                                    ͱ->  IRS from transponder_02
        *                                    ͱ->  IRS from transponder_03
        *                                            ⋮
        */
    if (_sdf->HasElement("interrogation_mode"))
    {
        std::string interrogation_mode = _sdf->Get<std::string>(
            "interrogation_mode");
        if (std::find(im.begin(), im.end(), interrogation_mode) != im.end())
        {
            gzmsg << interrogation_mode << " interrogation mode is used"
                  << std::endl;
            this->m_interrogationMode = interrogation_mode;
        }
        else
        {
            gzmsg << "Specified interrogation mode is unavailable, "
                  << "Common mode is used" << std::endl;
            this->m_interrogationMode = "common";
        }
    }
    else
    {
        gzmsg << "Interrogation mode is not specified, Common mode is used"
              << std::endl;
        this->m_interrogationMode = "common";
    }

    /************************************************************************/

    /***************  GAZEBO SUBSCRIBER ************************/

    // store this entity model
    this->m_model = _model;

    // initialize Gazebo node
    this->m_gzNode = transport::NodePtr(new transport::Node());
    this->m_gzNode->Init();

    // Gazebo subscriber for getting position of the transponder
    for (auto& transponder : this->m_deployedTransponders)
    {
        std::string transponder_position = "/" + this->m_namespace
            + "/" + this->m_transceiverDevice + "_" + transponder
            + "/global_position";
        this->m_transponderPoseSub.push_back(this->m_gzNode->Subscribe(
            transponder_position, &TransceiverPlugin::receiveGezeboCallback,
            this));
    }
    /************************************************************************/

    /******************  ROS PUBLISHERS ************************/

    // ROS node initialization
    this->m_rosNode.reset(new ros::NodeHandle(this->m_transceiverDevice));

    // ROS publisher for broadcasting transponder's relative location
    // to the transceiver
    std::string transponder_location_topic = "/" + this->m_namespace
        + "/" + this->m_transceiverDevice + "_" + this->m_transceiverID
        + "/transponder_location";
    this->m_publishTransponderRelPos = this->m_rosNode->advertise<
        geometry_msgs::Vector3>(transponder_location_topic, 1);

    // ROS publisher for common interrogation signal ping
    std::string cis_pinger_topic = "/" + m_namespace
        + "/common_interrogation_ping";
    this->m_cisPinger = this->m_rosNode->advertise<std_msgs::String>(
        cis_pinger_topic, 1);

    // ROS publisher for individual signal ping and command for each transponder
    for (auto& transponder : this->m_deployedTransponders)
    {
        std::string ping_topic("/" + this->m_namespace + "/"
            + this->m_transponderDevice + "_" + transponder
            + "/individual_interrogation_ping");
        std::string command_topic("/" + this->m_namespace + "/"
            + this->m_transponderDevice + "_" + transponder
            + "/command_request");
        this->m_iisPinger[transponder] = this->m_rosNode->advertise<
            std_msgs::String>(ping_topic, 1);
        this->m_commandPubs[transponder] = this->m_rosNode->advertise<
            dave_gazebo_model_plugins::UsblCommand>(command_topic, 1);
    }

    std::string transponder_location_cartesion_topic = "/"
        + this->m_namespace + "/" + this->m_transceiverDevice + "_"
        + this->m_transceiverID + "/transponder_location_cartesion";
    this->m_publishTransponderRelPosCartesion = this->m_rosNode->advertise<
        geometry_msgs::Vector3>(transponder_location_cartesion_topic, 1);

    /************************************************************************/

    /******************  ROS SUBSCRIBERS ***********************/

    // create ROS subscriber for temperature
    this->m_rosNode.reset(new ros::NodeHandle(m_transceiverDevice));

    // subscriber for temperature sensor
    ros::SubscribeOptions temperature_sub =
        ros::SubscribeOptions::create<std_msgs::Float64>(
            "/" + this->m_namespace + "/" + this->m_transceiverDevice
            + "_" + this->m_transceiverID + "/temperature",
            1,
            boost::bind(&TransceiverPlugin::temperatureRosCallback, this, _1),
            ros::VoidPtr(), &this->m_rosQueue);

    this->m_temperatureSub = this->m_rosNode->subscribe(temperature_sub);

    // subscriber for setting interrogation mode
    ros::SubscribeOptions interrogation_mode_sub =
        ros::SubscribeOptions::create<std_msgs::String>(
            "/" + this->m_namespace + "/" + this->m_transceiverDevice
            + "_" + this->m_transceiverID + "/interrogation_mode",
            1,
            boost::bind(&TransceiverPlugin::interrogationModeRosCallback,
                        this, _1),
            ros::VoidPtr(), &this->m_rosQueue);

    this->m_interrogationModeSub = this->m_rosNode->subscribe(
        interrogation_mode_sub);

    // subscriber for command response
    ros::SubscribeOptions command_response =
        ros::SubscribeOptions::create<dave_gazebo_model_plugins::UsblResponse>(
            "/" + this->m_namespace + "/" + this->m_transceiverDevice + "_"
            + this->m_transceiverID + "/command_response",
            1,
            boost::bind(&TransceiverPlugin::commandingResponseCallback,
                        this, _1),
            ros::VoidPtr(), &this->m_rosQueue);

    this->m_commandResponseSub = this->m_rosNode->subscribe(command_response);

    // subscriber for testing command response
    ros::SubscribeOptions channel_switch =
        ros::SubscribeOptions::create<std_msgs::String>(
            "/" + this->m_namespace + "/" + this->m_transceiverDevice
            + "_" + this->m_transceiverID + "/channel_switch",
            1,
            boost::bind(&TransceiverPlugin::channelSwitchCallback, this, _1),
            ros::VoidPtr(), &this->m_rosQueue);

    this->m_channelSwitchSub = this->m_rosNode->subscribe(channel_switch);

    // subscriber for testing command response
    ros::SubscribeOptions command_response_test =
        ros::SubscribeOptions::create<std_msgs::String>(
            "/" + this->m_namespace + "/" + this->m_transceiverDevice
            + "_" + this->m_transceiverID + "/command_response_test",
            1,
            boost::bind(&TransceiverPlugin::commandingResponseTestCallback,
                        this, _1),
            ros::VoidPtr(), &this->m_rosQueue);

    this->m_commandResponseTestSub = this->m_rosNode->subscribe(
            command_response_test);

    /************************************************************************/

    /******************  ROS MISC  *****************************/
    // timer to send ping Command
    if (this->m_enablePingerScheduler)
    {
        this->m_timer = this->m_rosNode->createTimer(ros::Duration(1.0),
            &TransceiverPlugin::sendPing, this);
    }

    this->m_rosQueueThread = std::thread(std::bind(
        &TransceiverPlugin::queueThread, this));
}

void TransceiverPlugin::commandingResponseTestCallback(
        const std_msgs::StringConstPtr &msg)
{
    std::string transponder_id = msg->data;
    sendCommand(BATTERY_LEVEL, transponder_id);
}

void TransceiverPlugin::commandingResponseCallback(
        const dave_gazebo_model_plugins::UsblResponseConstPtr &msg)
{
    gzmsg << "Response_id: " << msg->responseID << ", transponder id: "
          << msg->transceverID << ", data: " << msg->data << std::endl;
}

void TransceiverPlugin::sendCommand(int command_id, std::string& transponder_id)
{
    dave_gazebo_model_plugins::UsblCommand command;
    command.commandID = command_id;
    command.transponderID = std::stoi(transponder_id);
    if (command_id == BATTERY_LEVEL)
    {
        command.data = "report battery level";
    }
    else if (command_id == GO_TO)
    {
        command.data = "go to this location";
    }
    else
    {
        command.data = "this is dummy message";
    }
    this->m_commandPubs[transponder_id].publish(command);
}

// publish ROS ping topic to get response from transponder depending
// on the interrogation modes
void TransceiverPlugin::sendPing(const ros::TimerEvent&)
{
    std_msgs::String ping_msg;
    ping_msg.data = "ping";

    // need to fake the transmission by applying distance based delay
    physics::ModelPtr tranponder = this->m_model->GetWorld()->ModelByName(
        this->m_transponderAttachedObject);
    double dist = (this->m_model->WorldPose().Pos()
        - tranponder->WorldPose().Pos()).Length();
    // gzmsg << "distance to tranponder: " << dist << " m\n";
    sleep(dist/this->m_soundSpeed);

    if (this->m_interrogationMode.compare("common") == 0)
    {
        this->m_cisPinger.publish(ping_msg);
    }
    else if (this->m_interrogationMode.compare("individual") == 0)
    {
        this->m_iisPinger[this->m_channel].publish(ping_msg);
    }
    else
    {
            gzmsg << "Interrogation mode not recognized\n";
    }
}

// switch channel to ping another transponder or all transponders
void TransceiverPlugin::channelSwitchCallback(
        const std_msgs::StringConstPtr &msg)
{
    gzmsg << "Switching to transponder_" << msg->data << " channel\n";
    this->m_channel = msg->data;
}

// gets temperature from sensor to adjust sound speed
void TransceiverPlugin::temperatureRosCallback(
        const std_msgs::Float64ConstPtr &msg)
{
    this->m_temperature = msg->data;
    auto my_pos = this->m_model->WorldPose();

    // Base on https://dosits.org/tutorials/science/tutorial-speed/
    this->m_soundSpeed = 1540.4 + my_pos.Pos().Z() / 1000 * 17
        + (this->m_temperature - 10) * 4;
    gzmsg << "Detected change of temperature, sound speed is now: "
          << this->m_soundSpeed << " m/s\n";
}

// callback for interrogation mode switch
void TransceiverPlugin::interrogationModeRosCallback(
         const std_msgs::StringConstPtr &msg)
{
    std::string mode = msg->data;
    if (std::find(im.begin(), im.end(), mode) != im.end())
    {
        this->m_interrogationMode = mode;
    }
    else
    {
        gzmsg << "The input mode is not available\n";
    }
}

// Gazebo callback for receiving transponder position, simulating
// Transceiver's positioning calculation
void TransceiverPlugin::receiveGezeboCallback(
        ConstVector3dPtr& transponder_position)
{
    // gzmsg << "Transceiver acquires transponders position: "
    //       << transponder_position->x() << " " << transponder_position->y()
    //       << " " << transponder_position->z() << std::endl;

    ignition::math::Vector3d transponder_position_ign
        = ignition::math::Vector3d(transponder_position->x(),
        transponder_position->y(), transponder_position->z());

    double bearing = 0, range = 0, elevation = 0;
    calcuateRelativePose(transponder_position_ign, bearing, range, elevation);

    publishPosition(bearing, range, elevation);
}

// publish transponder's relative position in spherical
// coordinate(range, bearing, elevation)
void TransceiverPlugin::publishPosition(double &bearing, double &range,
                                        double &elevation)
{
    geometry_msgs::Vector3 location;
    location.x = bearing;
    location.y = range;
    location.z = elevation;

    geometry_msgs::Vector3 location_cartesion;
    location_cartesion.x = range * cos(elevation * M_PI/180)
                           * cos(bearing * M_PI/180);
    location_cartesion.y = range * cos(elevation * M_PI/180)
                           * sin(bearing * M_PI/180);
    location_cartesion.z = range * sin(elevation * M_PI/180);


    gzmsg << "Spherical Coordinate: \n\tBearing: " << location.x
          << " degree(s)\n\tRange: " << location.y
          << " m\n\tElevation: " << location.z << " degree(s)\n";
    gzmsg << "Cartesion Coordinate: \n\tX: " << location_cartesion.x
          << " m\n\tY: " << location_cartesion.y
          << " m\n\tZ: " << location_cartesion.z << " m\n\n";

    this->m_publishTransponderRelPos.publish(location);
    this->m_publishTransponderRelPosCartesion.publish(location_cartesion);
}

void TransceiverPlugin::calcuateRelativePose(ignition::math::Vector3d position,
        double &bearing, double &range, double &elevation)
{
    auto my_pos = this->m_model->WorldPose();
    auto direction = position - my_pos.Pos();

    bearing = atan2(direction.Y(), direction.X()) * 180 / M_PI;
    range = sqrt(direction.X()*direction.X() + direction.Y()*direction.Y()
          + direction.Z()*direction.Z());
    elevation = asin(direction.Z()/direction.Length()) * 180 / M_PI;

    // gzmsg << "bearing: " << bearing << "\n";
    // gzmsg << "range: " << range << "\n";
    // gzmsg << "elevation: " << elevation << "\n\n";
}

// use threads to execute callback associated with each subscriber
void TransceiverPlugin::queueThread()
{
    static const double timeout = 0.01;
    while (this->m_rosNode->ok())
    {
        this->m_rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}

