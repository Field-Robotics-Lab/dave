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
 *
*/
#include <gazebo/physics/Collision.hh>
#include <algorithm>    // std::lower_bound
#include <dave_gazebo_model_plugins/plug_and_socket_plugin.h>


using namespace gazebo;

// Counters are used for message throttle of some output
const int LOG_THROTTLE_RATE = 750;

// Distance at which mating joint creation tests commence
const float JOINT_TEST_RANGE = 0.5;

// Distance at which the joint will actually be created
const float JOINT_CREATE_RANGE = 0.14;

// Wait time between lock/unlock and continued tests
const float JOINT_LOCK_TIMEOUT = 5.0;

// Default plugin parameter values
const float DEFAULT_ALIGNMENT_TOLERANCE = 0.15;
const float DEFAULT_MATING_FORCE = 50.0;
const float DEFAULT_UNMATING_FORCE = 90.0;

// Default model and link name strings
const std::string DEFAULT_SENSOR_PLATE_NAME = "sensor_plate";
const std::string DEFAULT_TUBE_LINK_NAME = "socket";
const std::string DEFAULT_PLUG_MODEL_NAME = "plug";
const std::string DEFAULT_PLUG_LINK_NAME = "plug";
const std::string DEFAULT_GRIPPER_LINK_SUBSTRING = "finger_tip";

//////////////////////////////////////////////////
void PlugAndSocketMatingPlugin::Load(physics::ModelPtr _model,
                                     sdf::ElementPtr _sdf)
{
  this->world = _model->GetWorld();

  // set the socket model and retrieve the link info from the SDF
  this->socketModel = _model;

  if (_sdf->HasElement("sensorPlateLink"))
  {
    this->sensorPlateName = _sdf->GetElement("sensorPlateLink")
                                ->Get<std::string>();
    gzmsg << "Socket Sensor Plate Link name set to " <<
             this->sensorPlateName << std::endl;
  }
  else
  {
    this->sensorPlateName = DEFAULT_SENSOR_PLATE_NAME;
     gzmsg << "Socket Sensor Plate link name not specified, " <<
              "set to default " << this->sensorPlateName << std::endl;
  }
  this->sensorPlate = this->socketModel->GetLink(this->sensorPlateName);
  gzmsg << "Socket Sensor Plate link set from SDF to " <<
           this->sensorPlate->GetName() << std::endl;

  if (_sdf->HasElement("socketTubeLink"))
  {
    this->tubeLinkName =
      _sdf->GetElement("socketTubeLink")->Get<std::string>();
    gzmsg << "Socket Tube Link name set to " <<
             this->tubeLinkName << std::endl;
  }
  else
  {
    this->tubeLinkName = DEFAULT_TUBE_LINK_NAME;
    gzmsg << "Socket Tube Link name not specified, set to default " <<
             this->tubeLinkName << std::endl;
  }
  this->tubeLink = this->socketModel->GetLink(this->tubeLinkName);
  gzmsg << "Socket Tube Link set from SDF to " <<
           this->tubeLink->GetName() << std::endl;

  // Retrieve plug model and link info from the SDF
  if (_sdf->HasElement("plugModel"))
  {
    this->plugModelName = _sdf->GetElement("plugModel")
                              ->Get<std::string>();
    gzmsg << "Plug Model name set to " << this->plugModelName << std::endl;
  }
  else
  {
    this->plugModelName = DEFAULT_PLUG_MODEL_NAME;
    gzmsg << "Plug Model name not specified, set to default " <<
             this->plugModelName << std::endl;
  }
  this->plugModel = this->world->ModelByName(this->plugModelName);
  gzmsg << "Plug Model set from SDF" << std::endl;

  if (_sdf->HasElement("plugLink"))
  {
    this->plugLinkName =
      _sdf->GetElement("plugLink")->Get<std::string>();
    gzmsg << "Plug Link name set to " << this->plugLinkName << std::endl;
  }
  else
  {
    this->plugLinkName = DEFAULT_PLUG_LINK_NAME;
    gzmsg << "Plug Link name not specified, set to default " <<
             this->plugLinkName << std::endl;
  }
  this->plugLink = this->plugModel->GetLink(this->plugLinkName);
  gzmsg << "Plug Link set from SDF to " << this->plugLink->GetName() <<
           std::endl;

  if (_sdf->HasElement("gripperLinkSubstring"))
  {
    this->gripperLinkSubstring =
      _sdf->GetElement("gripperLinkSubstring")->Get<std::string>();
    gzmsg << "Gripper link substring set to " <<
             this->gripperLinkSubstring << std::endl;
  }
  else
  {
    this->gripperLinkSubstring = DEFAULT_GRIPPER_LINK_SUBSTRING;
    gzmsg << "Gripper link substring not specified, set to default " <<
             this->gripperLinkSubstring << std::endl;
  }

  // Retrieve socket tolerance parameters from SDF
  if (_sdf->HasElement("rollAlignmentTolerance"))
  {
    this->rollAlignmentTolerance =
      _sdf->GetElement("rollAlignmentTolerance")->Get<double>();
    gzmsg << this->tubeLinkName <<
             " socket Roll Mating Alignment Tolerance is: " <<
             this->rollAlignmentTolerance << std::endl;
  }
  else
  {
    this->rollAlignmentTolerance = DEFAULT_ALIGNMENT_TOLERANCE;
    gzmsg << this->tubeLinkName <<
             " socket Roll Mating Alignment Tolerance was not " <<
             "specified, using default value of " <<
             this->rollAlignmentTolerance << std::endl;
  }

  if (_sdf->HasElement("pitchAlignmentTolerance"))
  {
    this->pitchAlignmentTolerance =
      _sdf->GetElement("pitchAlignmentTolerance")->Get<double>();
    gzmsg << this->tubeLinkName <<
             " socket Pitch Mating Alignment Tolerance is: " <<
             this->pitchAlignmentTolerance << std::endl;
  }
  else
  {
    this->pitchAlignmentTolerance = DEFAULT_ALIGNMENT_TOLERANCE;
    gzmsg << this->tubeLinkName <<
             " socket Pitch Mating Alignment Tolerance was not " <<
             "specified, using default value of " <<
             this->pitchAlignmentTolerance << std::endl;
  }

  if (_sdf->HasElement("yawAlignmentTolerance"))
  {
    this->yawAlignmentTolerance =
      _sdf->GetElement("yawAlignmentTolerance")->Get<double>();
    gzmsg << this->tubeLinkName <<
             " socket Yaw Mating Alignment Tolerance is: " <<
             this->yawAlignmentTolerance << std::endl;
  }
  else
  {
    this->yawAlignmentTolerance = DEFAULT_ALIGNMENT_TOLERANCE;
    gzmsg << this->tubeLinkName <<
             " socket Yaw Mating Alignment Tolerance was not " <<
             "specified, using default value of " <<
             this->yawAlignmentTolerance << std::endl;
  }

  if (_sdf->HasElement("matingForce"))
  {
    this->matingForce = _sdf->GetElement("matingForce")->Get<double>();
    gzmsg << this->tubeLinkName <<
             " socket Mating Force: " << this->matingForce << std::endl;
  }
  else
  {
    this->matingForce = DEFAULT_MATING_FORCE;
    gzmsg << this->tubeLinkName <<
             " socket Mating Force not specified, " <<
             "using default value of " << this->matingForce << std::endl;
  }

  if (_sdf->HasElement("unmatingForce"))
  {
    this->unmatingForce = _sdf->GetElement("unmatingForce")->Get<double>();
    gzmsg << this->tubeLinkName <<
             " socket Unmating Force: " << this->unmatingForce << std::endl;
  }
  else
  {
    this->unmatingForce = DEFAULT_UNMATING_FORCE;
    gzmsg << this->tubeLinkName <<
             " socket Unmating Force not specified, " <<
             "using default value of " << this->unmatingForce << std::endl;
  }

  this->ns = "/" + this->plugModelName + "/";
  if (_sdf->HasElement("linkForceTopic"))
  {
    this->linkForceTopic = this->ns + _sdf->GetElement("linkForceTopic")
                                          ->Get<std::string>();
    gzmsg << "Plug applied force topic name: " <<
             this->linkForceTopic << std::endl;
  }
  else
  {
    this->linkForceTopic = this->ns + "appliedForce";
    gzmsg << "Plug applied force topic name not specified, " <<
             "using default value of " << this->linkForceTopic << std::endl;
  }

  this->world->Physics()->GetContactManager()->SetNeverDropContacts(true);
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&PlugAndSocketMatingPlugin::Update, this));

  // Set up ROS stuff so that we can publish the force applied to the plug link
  this->rosNode.reset(new ros::NodeHandle(this->ns));
  this->linkForcePub = this->rosNode->advertise<geometry_msgs::Vector3Stamped>(
    this->linkForceTopic, 5);
}

//////////////////////////////////////////////////
void PlugAndSocketMatingPlugin::trimForceVector(double trimDuration)
{
    std::vector<common::Time>::iterator low;
    if (this->timeStamps.size() == 0)
      return;

    low = std::lower_bound(this->timeStamps.begin(), this->timeStamps.end(),
        this->timeStamps.back()-trimDuration);
    this->timeStamps.erase(this->timeStamps.begin(), this->timeStamps.begin()
        + std::distance(this->timeStamps.begin(), low));
    this->forcesBuffer.erase(this->forcesBuffer.begin(),
        this->forcesBuffer.begin() + std::distance(this->timeStamps.begin(),
        low));
};

//////////////////////////////////////////////////
double PlugAndSocketMatingPlugin::movingTimedAverage()
{
  return accumulate(this->forcesBuffer.begin(),
                    this->forcesBuffer.end(), 0.0) / this->forcesBuffer.size();
};

//////////////////////////////////////////////////
void PlugAndSocketMatingPlugin::addForce(double force)
{
  if (abs(force) < 5.0)
    return;

  this->forcesBuffer.push_back(force);
  this->timeStamps.push_back(this->world->SimTime());
}

//////////////////////////////////////////////////
PlugAndSocketMatingPlugin::PlugAndSocketMatingPlugin() : ModelPlugin()
{
}

//////////////////////////////////////////////////
void PlugAndSocketMatingPlugin::lockJoint()
{
  if (this->locked)
  {
      gzdbg << this->tubeLinkName << "-" << this->plugLinkName <<
               " joint already locked!" << std::endl;
    return;
  }
  this->locked = true;
  this->unfreezeTimeBuffer =  this->world->SimTime();
  gzmsg << this->tubeLinkName << "-" << this->plugLinkName <<
           " joint locked!" << std::endl;
  double currentPosition = this->prismaticJoint->Position(0);
  this->prismaticJoint->SetUpperLimit(0, currentPosition);
  this->prismaticJoint->SetLowerLimit(0, currentPosition);
}

//////////////////////////////////////////////////
void PlugAndSocketMatingPlugin::unlockJoint()
{
  if (!this->locked)
  {
    gzdbg << this->tubeLinkName << "-" << this->plugLinkName <<
             " joint already unlocked" << std::endl;
    return;
  }
  this->locked = false;
  this->unfreezeTimeBuffer =  this->world->SimTime();
  gzmsg << this->tubeLinkName << "-" << this->plugLinkName <<
           " joint unlocked!" << std::endl;
  this->removeJoint();
}

//////////////////////////////////////////////////
double PlugAndSocketMatingPlugin::normalizeError(double error)
{
  while (error > M_PI)
  {
    error -= (2.0 * M_PI);
  }
  while (error <= -M_PI)
  {
    error += (2.0 * M_PI);
  }
  return error;
}

//////////////////////////////////////////////////
bool PlugAndSocketMatingPlugin::isAligned()
{
  ignition::math::Pose3d socketPose = this->tubeLink->WorldPose();
  ignition::math::Pose3d plugPose = this->plugLink->WorldPose();
  ignition::math::Pose3d poseDiff = plugPose - socketPose;

  ignition::math::Vector3<double> rotError = poseDiff.Rot().Euler();
  ignition::math::Vector3<double> posError = poseDiff.Pos();
  double lateralOffset = hypot(posError[1], posError[2]);
  double offsetAngle = normalizeError(atan2(lateralOffset, posError[0]));
  double range = sqrt(posError[0] * posError[0] +
                      posError[1] * posError[1] +
                      posError[2] * posError[2]);

  bool orientOK =
      (abs(normalizeError(rotError[0])) <= this->rollAlignmentTolerance) &&
      (abs(normalizeError(rotError[1])) <= this->pitchAlignmentTolerance) &&
      (abs(normalizeError(rotError[2])) <= this->yawAlignmentTolerance);
  bool lateralOK = (abs(offsetAngle) <= std::max(
                                           this->yawAlignmentTolerance,
                                           this->pitchAlignmentTolerance));
  bool rangeOK = (range >= 0.0) && (range <= JOINT_TEST_RANGE);

  if (orientOK && lateralOK && rangeOK)
  {
    if (this->alignLogThrottle == 0)
    {
      gzmsg << this->tubeLinkName << " and " << this->plugLinkName <<
               " are aligned in orientation and distance" << std::endl;
    }
    return true;
  }
  else if (orientOK && lateralOK)
  {
    if (this->alignLogThrottle == 0)
    {
      gzmsg << this->tubeLinkName << " and " << this->plugLinkName <<
               " are aligned but too far apart" << std::endl;
    }
    return false;
  }
  else
  {
    return false;
  }
}

//////////////////////////////////////////////////
bool PlugAndSocketMatingPlugin::checkProximity()
{
  ignition::math::Pose3d socket_pose = this->tubeLink->WorldPose();
  ignition::math::Vector3<double> socketPositon = socket_pose.Pos();
  ignition::math::Pose3d plug_pose = plugModel->RelativePose();
  ignition::math::Vector3<double> plugPosition = plug_pose.Pos();
  float xdiff_squared = pow(abs(plugPosition[0] - socketPositon[0]), 2);
  float ydiff_squared = pow(abs(plugPosition[1] - socketPositon[1]), 2);
  float zdiff_squared = pow(abs(plugPosition[2] - socketPositon[2]), 2);

  if (DEBUG)
    gzmsg << this->tubeLinkName << " and " << this->plugLinkName <<
             " euclidean distance: " <<
             pow(xdiff_squared+ydiff_squared+zdiff_squared, 0.5) << std::endl;

  bool withinProximity =
    pow(xdiff_squared+ydiff_squared+zdiff_squared, 0.5) < JOINT_CREATE_RANGE;

  if (withinProximity)
  {
    if (this->proximityLogThrottle == 0)
    {
      gzmsg << this->tubeLinkName << " and " << this->plugLinkName <<
               " within proximity" << std::endl;
    }
    return true;
  }
  else
  {
    if (this->proximityLogThrottle == 0)
    {
      gzmsg << this->tubeLinkName << " and " << this->plugLinkName <<
               " not within proximity, please move the plug closer" <<
               std::endl;
    }
  }
  return false;
}

//////////////////////////////////////////////////
void PlugAndSocketMatingPlugin::constructJoint()
{
  if (this->joined)
  {
    gzmsg << this->tubeLinkName << "-" << this->plugLinkName <<
             " joint already frozen" << std::endl;
    return;
  }
  this->joined = true;
  this->alignmentTime = 0;
  this->prismaticJoint = plugModel->CreateJoint(
    this->tubeLinkName + "_plug_joint", "prismatic",
    this->tubeLink, this->plugLink);
  this->prismaticJoint->Load(this->tubeLink, this->plugLink,
                             ignition::math::Pose3<double>(
                                ignition::math::Vector3<double>(0, 0, 0),
                                ignition::math::Quaternion<double>(0, 0, 0)));
  this->prismaticJoint->Init();
  this->prismaticJoint->SetAxis(0, ignition::math::Vector3<double>(1, 0, 0));
  this->prismaticJoint->SetLowerLimit(0, this->prismaticJoint->
                                               Position(0)-JOINT_CREATE_RANGE);
  this->prismaticJoint->SetUpperLimit(0, this->prismaticJoint->
                                               Position(0)+JOINT_CREATE_RANGE);
  gzmsg << this->tubeLinkName << "-" << this->plugLinkName <<
           " joint formed" << std::endl;
}

//////////////////////////////////////////////////
void PlugAndSocketMatingPlugin::removeJoint()
{
  if (this->joined == true)
  {
    this->joined = false;
    this->prismaticJoint->Detach();
    this->plugModel->RemoveJoint(this->tubeLinkName + "_plug_joint");
    this->prismaticJoint.reset();
    gzmsg << this->tubeLinkName << "-" << this->plugLinkName <<
             " joint removed" << std::endl;
  }
}

//////////////////////////////////////////////////
bool PlugAndSocketMatingPlugin::averageForceOnLink(std::string contact1,
                                                   std::string contact2)
{
  msgs::Contact contactMsg;
  std::vector<int> contacts = this->getCollisionsBetween(contact1, contact2);
  if (contacts.size() == 0)
  {
    return false;
  }

  this->plugLinkForce.Set(0.0, 0.0, 0.0);
  for (int i = 0; i < contacts.size(); i++)
  {
    physics::Contact *contact =
      this->world->Physics()->GetContactManager()->GetContact(contacts[i]);
    contact->FillMsg(contactMsg);
    ignition::math::Vector3d body1Force(0.0, 0.0, 0.0);
    ignition::math::Vector3d body2Force(0.0, 0.0, 0.0);
    for (int j = 0; j < contactMsg.wrench().size(); j++)
    {
      msgs::Vector3d f1 = contactMsg.wrench()[j].body_1_wrench().force();
      msgs::Vector3d f2 = contactMsg.wrench()[j].body_2_wrench().force();
      body1Force += ignition::math::Vector3d(f1.x(), f1.y(), f1.z());
      body2Force += ignition::math::Vector3d(f2.x(), f2.y(), f2.z());
    }
    // Add force applied to the plug along its link X axis
    if (contact->collision1->GetLink()->GetName() == contact1)
    {
      this->plugLinkForce += body1Force;
    }
    else
    {
      this->plugLinkForce += body2Force;
    }
  }
  if (this->linksInContactLogThrottle == 0)
  {
    gzmsg << "Force of " << this->plugLinkForce[0] << " by " << contact1
          << " on " << contact2 << " from " << contacts.size()
          << " contact points." <<std::endl;
  }
  this->addForce(this->plugLinkForce[0]);
  this->trimForceVector(0.1);

  // Generate and publish the ROS message with the applied force
  geometry_msgs::Vector3Stamped forceMsg;
  forceMsg.header.stamp = ros::Time::now();
  forceMsg.header.frame_id = this->plugLinkName;
  forceMsg.vector.x = this->plugLinkForce.X();
  forceMsg.vector.y = this->plugLinkForce.Y();
  forceMsg.vector.z = this->plugLinkForce.Z();
  this->linkForcePub.publish(forceMsg);

  return true;
}

//////////////////////////////////////////////////
bool PlugAndSocketMatingPlugin::isPlugPushingSensorPlate(
    int numberOfDatapointsThresh)
{
  if (!this->averageForceOnLink(this->plugLinkName, this->sensorPlateName))
  {
    return false;
  }
  else
  {
    double averageForce = this->movingTimedAverage();
    if ((averageForce > this->matingForce) &&
        (this->forcesBuffer.size() > numberOfDatapointsThresh))
    {
      if (DEBUG)
        gzdbg << this->tubeLinkName << "-" << this->plugLinkName
              << " sensor plate average: " << averageForce
              << ", size " << this->forcesBuffer.size() << std::endl;
      this->forcesBuffer.clear();
      this->timeStamps.clear();
      return true;
    }
    else
    {
      return false;
    }
  }
}

//////////////////////////////////////////////////
bool PlugAndSocketMatingPlugin::isEndEffectorPushingPlug(
    int numberOfDatapointsThresh)
{
  // This will sum forces from all "*finger_tip*" links in contact
  // with the plug (right & left in the demo)
  if (!this->averageForceOnLink(this->plugLinkName,
                                this->gripperLinkSubstring))
  {
    return false;
  }
  else
  {
    double averageForce = this->movingTimedAverage();
    if ((averageForce > this->unmatingForce) &&
        (this->forcesBuffer.size() > numberOfDatapointsThresh))
    {
      if (DEBUG)
        gzdbg << this->tubeLinkName << "-" << this->plugLinkName
              << " end effector average: " << averageForce
              << ", size " << this->forcesBuffer.size() << std::endl;
      this->forcesBuffer.clear();
      this->timeStamps.clear();
      return true;
    }
    else
    {
      return false;
    }
  }
}

//////////////////////////////////////////////////
std::vector<int>
PlugAndSocketMatingPlugin::getCollisionsBetween(std::string contact1,
                                                std::string contact2)
{
  std::vector<int> collisions;
  for (int i = 0;
       i < this->world->Physics()->GetContactManager()->GetContactCount(); i++)
  {
    physics::Contact *contact =
      this->world->Physics()->GetContactManager()->GetContact(i);
    bool inContact =
           (contact->collision1->GetLink()->GetName().find(contact1) !=
                                                          std::string::npos) &&
           (contact->collision2->GetLink()->GetName().find(contact2) !=
                                                          std::string::npos) ||
           (contact->collision1->GetLink()->GetName().find(contact2) !=
                                                          std::string::npos) &&
           (contact->collision2->GetLink()->GetName().find(contact1) !=
                                                          std::string::npos);
    if (inContact)
    {
      collisions.push_back(i);
    }
  }

  return collisions;
}

//////////////////////////////////////////////////
void PlugAndSocketMatingPlugin::Update()
{
  // check if recently locked or removed the joint
  // (to avoid recreating it or removing it right away)
  if (this->world->SimTime() - unfreezeTimeBuffer < JOINT_LOCK_TIMEOUT)
      return;

  // If plug and socket are not joined yet, check the alignment
  // between them, and if alignment is maintained for more than
  // 2 seconds, then construct a joint between them
  if (!this->joined)
  {
    if (this->isAligned() && this->checkProximity())
    {
      if (alignmentTime == 0)
      {
        alignmentTime = this->world->SimTime();
      }
      else if (this->world->SimTime() - alignmentTime > 2)
      {
        this->constructJoint();
      }
    }
    else
    {
      alignmentTime = 0;
    }
  }
  // If joint is constructed but plug is not yet fixed/locked
  // into the socket, measure the forces and lock the plug to
  // the socket if the plug is pushing against it.
  else if (this->joined && !this->locked)
  {
    if (this->isPlugPushingSensorPlate())
    {
      this->lockJoint();
    }
  }
  // If plug is locked to socket, see if there is
  // enough force being exerted to pull it out
  else if (this->joined && this->locked)
  {
    if (this->isEndEffectorPushingPlug())
    {
      this->unlockJoint();
    }
  }

  // Increment the logging throttle counters
  this->proximityLogThrottle =
      (this->proximityLogThrottle + 1) % LOG_THROTTLE_RATE;
  this->alignLogThrottle =
      (this->alignLogThrottle + 1) % LOG_THROTTLE_RATE;
  this->linksInContactLogThrottle =
      (this->linksInContactLogThrottle + 1) % LOG_THROTTLE_RATE;
}
