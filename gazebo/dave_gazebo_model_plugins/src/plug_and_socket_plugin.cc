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

// ROS_INFO_STREAM_THROTTLE suppresses too much, so a
// counter-based message throttle is being used for some output
const int LOG_THROTTLE_RATE = 750;

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
    ROS_INFO_STREAM("Socket Sensor Plate Link name set to " <<
                    this->sensorPlateName);
  }
  else
  {
    this->sensorPlateName = "sensor_plate";
    ROS_INFO_STREAM("Socket Sensor Plate link name not specified, " <<
                    "set to default " << this->sensorPlateName);
  }
  this->sensorPlate = this->socketModel->GetLink(this->sensorPlateName);
  ROS_INFO_STREAM("Socket Sensor Plate link set from SDF to " <<
                  this->sensorPlate->GetName());

  if (_sdf->HasElement("socketTubeLink"))
  {
    this->tubeLinkName =
      _sdf->GetElement("socketTubeLink")->Get<std::string>();
    ROS_INFO_STREAM("Socket Tube Link name set to " << this->tubeLinkName);
  }
  else
  {
    this->tubeLinkName = "socket";
    ROS_INFO_STREAM("Socket Tube Link name not specified, " <<
                    "set to default " << this->tubeLinkName);
  }
  this->tubeLink = this->socketModel->GetLink(this->tubeLinkName);
  ROS_INFO_STREAM("Socket Tube Link set from SDF to " <<
                  this->tubeLink->GetName());

  // Retrieve plug model and link info from the SDF
  if (_sdf->HasElement("plugModel"))
  {
    this->plugModelName = _sdf->GetElement("plugModel")
                              ->Get<std::string>();
    ROS_INFO_STREAM("Plug Model name set to " << this->plugModelName);
  }
  else
  {
    this->plugModelName = "plug";
    ROS_INFO_STREAM("Plug Model name not specified, set to default " <<
                    this->plugModelName);
  }
  this->plugModel = this->world->ModelByName(this->plugModelName);
  ROS_INFO_STREAM("Plug Model set from SDF");

  if (_sdf->HasElement("plugLink"))
  {
    this->plugLinkName =
      _sdf->GetElement("plugLink")->Get<std::string>();
    ROS_INFO_STREAM("Plug Link name set to " << this->plugLinkName);
  }
  else
  {
    this->plugLinkName = "plug";
    ROS_INFO_STREAM("Plug Link name not specified, set to default "
                    << this->plugLinkName);
  }
  this->plugLink = this->plugModel->GetLink(this->plugLinkName);
  ROS_INFO_STREAM("Plug Link set from SDF to " <<
                  this->plugLink->GetName());

  // Retrieve socket tolerance parameters from SDF
  if (_sdf->HasElement("rollAlignmentTolerance"))
  {
    this->rollAlignmentTolerance =
      _sdf->GetElement("rollAlignmentTolerance")->Get<double>();
    ROS_INFO_STREAM(this->tubeLinkName << 
                    " socket Roll Mating Alignment Tolerance is: " <<
                    this->rollAlignmentTolerance);
  }
  else
  {
    this->rollAlignmentTolerance = 0.3;
    ROS_INFO_STREAM(this->tubeLinkName << 
                    " socket Roll Mating Alignment Tolerance was not " <<
                    "specified, using default value of " <<
                     this->rollAlignmentTolerance);
  }

  if (_sdf->HasElement("pitchAlignmentTolerance"))
  {
    this->pitchAlignmentTolerance =
      _sdf->GetElement("pitchAlignmentTolerance")->Get<double>();
    ROS_INFO_STREAM(this->tubeLinkName << 
                    " socket Pitch Mating Alignment Tolerance is: " <<
                    this->pitchAlignmentTolerance);
  }
  else
  {
    this->pitchAlignmentTolerance = 0.3;
    ROS_INFO_STREAM(this->tubeLinkName << 
                    " socket Pitch Mating Alignment Tolerance was not " <<
                    "specified, using default value of " <<
                    this->pitchAlignmentTolerance);
  }

  if (_sdf->HasElement("yawAlignmentTolerance"))
  {
    this->yawAlignmentTolerance =
      _sdf->GetElement("yawAlignmentTolerance")->Get<double>();
    ROS_INFO_STREAM(this->tubeLinkName << 
                    " socket Yaw Mating Alignment Tolerance is: " <<
                    this->yawAlignmentTolerance);
  }
  else
  {
    this->yawAlignmentTolerance = 0.3;
    ROS_INFO_STREAM(this->tubeLinkName << 
                    " socket Yaw Mating Alignment Tolerance was not " <<
                    "specified, using default value of " << 
                    this->yawAlignmentTolerance);
  }

  if (_sdf->HasElement("zAlignmentTolerance"))
  {
    this->zAlignmentTolerance =
      _sdf->GetElement("zAlignmentTolerance")->Get<double>();
    ROS_INFO_STREAM(this->tubeLinkName << 
                    " socket Z Mating Alignment Tolerance is: " <<
                    this->zAlignmentTolerance);
  }
  else
  {
    this->zAlignmentTolerance = 0.1;
    ROS_INFO_STREAM(this->tubeLinkName << 
                    " socket Z Mating Alignment Tolerance was not " <<
                    "specified, using default value of " <<
                    this->zAlignmentTolerance);
  }

  if (_sdf->HasElement("matingForce"))
  {
    this->matingForce = _sdf->GetElement("matingForce")->Get<double>();
    ROS_INFO_STREAM(this->tubeLinkName << 
                    " socket Mating Force: " << this->matingForce);
  }
  else
  {
    this->matingForce = 50;
    ROS_INFO_STREAM(this->tubeLinkName << 
                    " socket Mating Force not specified, " <<
                    "using default value of " << this->matingForce);
  }

  if (_sdf->HasElement("unmatingForce"))
  {
    this->unmatingForce = _sdf->GetElement("unmatingForce")->Get<double>();
    ROS_INFO_STREAM(this->tubeLinkName << 
                    " socket Unmating Force: " << this->unmatingForce);
  }
  else
  {
    this->unmatingForce = 190;
    ROS_INFO_STREAM(this->tubeLinkName << 
                    " socket Unmating Force not specified, " <<
                    "using default value of " << this->unmatingForce);
  }

  this->world->Physics()->GetContactManager()->SetNeverDropContacts(true);
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&PlugAndSocketMatingPlugin::Update, this));
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
void PlugAndSocketMatingPlugin::lockJoint(physics::JointPtr prismaticJoint)
{
  if (this->locked)
  {
      ROS_DEBUG_STREAM(this->tubeLinkName << "-" << this->plugLinkName <<
                       " joint already locked!");
    return;
  }
  this->locked = true;
  ROS_INFO_STREAM(this->tubeLinkName << "-" << this->plugLinkName <<
                  " joint locked!");
  double currentPosition = prismaticJoint->Position(0);
  prismaticJoint->SetUpperLimit(0, currentPosition);
  prismaticJoint->SetLowerLimit(0, currentPosition);
}

//////////////////////////////////////////////////
void PlugAndSocketMatingPlugin::unfreezeJoint(physics::JointPtr prismaticJoint)
{
  if (!this->locked)
  {
    ROS_DEBUG_STREAM(this->tubeLinkName << "-" << this->plugLinkName <<
                     " joint already unlocked");
    return;
  }
  this->locked = false;
  this->unfreezeTimeBuffer =  this->world->SimTime();
  ROS_INFO_STREAM(this->tubeLinkName << "-" << this->plugLinkName <<
                  " joint unlocked!");
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
bool PlugAndSocketMatingPlugin::checkRotationalAlignment(bool verbose)
{
  ignition::math::Vector3<double> socketRotation =
                       this->tubeLink->WorldPose().Rot().Euler();
  ignition::math::Vector3<double> plugRotation =
                       this->plugLink->WorldPose().Rot().Euler();
  double rollError = abs(normalizeError(plugRotation[0] - socketRotation[0]));
  double pitchError = abs(normalizeError(plugRotation[1] - socketRotation[1]));
  double yawError = abs(normalizeError(plugRotation[2] - socketRotation[2]));
  if (verbose)
  {
    ROS_DEBUG_STREAM(this->tubeLinkName << "-" << 
                     this->plugLinkName << std::endl <<
                     "socket euler: " <<
                     socketRotation[0] << ", " <<
                     socketRotation[1] << ", " <<
                     socketRotation[2] << std::endl <<
                     "plug euler: " <<
                     plugRotation[0] << ", " <<
                     plugRotation[1] << ", " <<
                     plugRotation[2]);
  }

  this->rotateAlignLogThrottle =
      (this->rotateAlignLogThrottle + 1) % LOG_THROTTLE_RATE;
  if (rollError <= this->rollAlignmentTolerance &&
      pitchError <= this->pitchAlignmentTolerance &&
      yawError <= this->yawAlignmentTolerance)
  {
    if (this->rotateAlignLogThrottle == 0)
    {
      ROS_INFO_STREAM(this->tubeLinkName << " and " <<
                      this->plugLinkName << " are aligned");
    }
    return true;
  }
  else
  {
    return false;
  }
}

//////////////////////////////////////////////////
bool PlugAndSocketMatingPlugin::checkVerticalAlignment(
    double alignmentThreshold, bool verbose)
{
  ignition::math::Pose3d socket_pose = this->tubeLink->WorldPose();
  ignition::math::Vector3<double> socketPositon = socket_pose.Pos();
  ignition::math::Pose3d plug_pose = plugModel->RelativePose();
  ignition::math::Vector3<double> plugPosition = plug_pose.Pos();
  bool onSameVerticalLevel =
    abs(plugPosition[2] - socketPositon[2]) < alignmentThreshold;

  if (verbose)
    ROS_DEBUG_STREAM(
        this->plugLinkName << " Z plug: " << plugPosition[2] << ", " <<
        this->tubeLinkName << "  Z socket: " << socketPositon[2]);

  if (onSameVerticalLevel)
    return true;

  return false;
}

//////////////////////////////////////////////////
bool PlugAndSocketMatingPlugin::isAlligned(bool verbose)
{
  if (checkVerticalAlignment(true) && checkRotationalAlignment())
  {
    if (verbose)
      ROS_DEBUG_STREAM(this->tubeLinkName << " and " << 
                       this->plugLinkName << " are aligned in " <<
                       "orientation and altitude");
    return true;
  }
  else
  {
    return false;
  }
}

//////////////////////////////////////////////////
bool PlugAndSocketMatingPlugin::checkProximity(bool verbose)
{
  ignition::math::Pose3d socket_pose = this->tubeLink->WorldPose();
  ignition::math::Vector3<double> socketPositon = socket_pose.Pos();
  ignition::math::Pose3d plug_pose = plugModel->RelativePose();
  ignition::math::Vector3<double> plugPosition = plug_pose.Pos();
  float xdiff_squared = pow(abs(plugPosition[0] - socketPositon[0]), 2);
  float ydiff_squared = pow(abs(plugPosition[1] - socketPositon[1]), 2);
  float zdiff_squared = pow(abs(plugPosition[2] - socketPositon[2]), 2);

  if (verbose)
    ROS_DEBUG_STREAM(this->tubeLinkName << " and " << this->plugLinkName <<
                     " eucleadian distance: " <<
                     pow(xdiff_squared+ydiff_squared+zdiff_squared, 0.5));

  bool withinProximity =
    pow(xdiff_squared+ydiff_squared+zdiff_squared, 0.5) < 0.14;

  this->proximityLogThrottle = 
      (this->proximityLogThrottle + 1) % LOG_THROTTLE_RATE;
  if (withinProximity)
  {
    if (this->proximityLogThrottle == 0)
    {
      ROS_INFO_STREAM(this->tubeLinkName << " and " <<
                      this->plugLinkName << " within proximity");
    }
    return true;
  }
  else
  {
    if (this->proximityLogThrottle == 0)
    {
      ROS_INFO_STREAM(this->tubeLinkName << " and " <<
                      this->plugLinkName <<
                      " not within proximity, please move the plug closer");
    }
  }
  return false;
}

//////////////////////////////////////////////////
void PlugAndSocketMatingPlugin::constructJoint()
{
  if (this->joined)
  {
    ROS_DEBUG_STREAM(this->tubeLinkName << "-" << this->plugLinkName <<
                     " joint already frozen");
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
  this->prismaticJoint->SetLowerLimit(0, this->prismaticJoint->Position(0)-10);
  this->prismaticJoint->SetUpperLimit(0, this->prismaticJoint->Position(0)+10);
  ROS_INFO_STREAM(this->tubeLinkName << "-" << this->plugLinkName <<
                  " joint formed, position ");
}

//////////////////////////////////////////////////
void PlugAndSocketMatingPlugin::removeJoint()
{
  if (this->joined == true)
  {
    this->joined = false;
    this->prismaticJoint->Detach();
    this->prismaticJoint->Reset();
    this->prismaticJoint->~Joint();
    ROS_INFO_STREAM(this->tubeLinkName << "-" << this->plugLinkName <<
                    " joint removed");
  }
}

//////////////////////////////////////////////////
bool PlugAndSocketMatingPlugin::averageForceOnLink(std::string contact1,
                                                   std::string contact2)
{
  int contactIndex = this->getCollisionBetween(contact1, contact2);
  if (contactIndex == -1)
      return false;
  physics::Contact *contact =
    this->world->Physics()->GetContactManager()->GetContact(contactIndex);
  // TODO: update to use vectored force rather than just magnitude
  if (contact->collision1->GetLink()->GetName() == contact1)
  {
    this->addForce(contact->wrench[0].body1Force.Length());
  }
  else
  {
    this->addForce(contact->wrench[0].body2Force.Length());
  }
  this->trimForceVector(0.1);
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
      //ROS_INFO_STREAM(this->tubeLinkName << "-" << this->plugLinkName <<
      //                " sensor plate average: " << average force " <<
      //                ", size ", this->forcesBuffer.size());
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
  if (!this->averageForceOnLink(this->plugLinkName, "finger_tip"))
  {
    return false;
  }
  else
  {
    double averageForce = this->movingTimedAverage();
    if ((averageForce > this->unmatingForce) &&
        (this->forcesBuffer.size() > numberOfDatapointsThresh))
    {
      //ROS_INFO_STREAM(this->tubeLinkName << "-" << this->plugLinkName <<
      //                " end effector average: " << averageForce <<
      //                ", size " << this->forcesBuffer.size());
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
int PlugAndSocketMatingPlugin::getCollisionBetween(std::string contact1,
                                                   std::string contact2)
{
  for (int i = 0;
       i < this->world->Physics()->GetContactManager()->GetContactCount(); i++)
  {
    physics::Contact *contact =
      this->world->Physics()->GetContactManager()->GetContact(i);
    bool isPlugContactingSensorPlate =
           (contact->collision1->GetLink()->GetName().find(contact1) !=
                                                          std::string::npos) &&
           (contact->collision2->GetLink()->GetName().find(contact2) !=
                                                          std::string::npos) ||
           (contact->collision1->GetLink()->GetName().find(contact2) !=
                                                          std::string::npos) &&
           (contact->collision2->GetLink()->GetName().find(contact1) !=
                                                          std::string::npos);

    this->linksInContactLogThrottle =
        (this->linksInContactLogThrottle + 1) % LOG_THROTTLE_RATE;
    if (isPlugContactingSensorPlate)
    {
        if (this->linksInContactLogThrottle == 0)
        {
            ROS_INFO_STREAM(contact1 << " and " << contact2 << " in contact.");
        }
        return i;
    }
  }
  return -1;
}

//////////////////////////////////////////////////
void PlugAndSocketMatingPlugin::Update()
{
  // check if recently removed the joint
  // (to avoid it locking right away after unlocked)
  if (this->world->SimTime() - unfreezeTimeBuffer < 2)
      return;
      // If plug and socket are not joined yet, check the alignment
      // between them, and if alignment is maintained for more than
      // 2 seconds, then construct a joint between them
  if (!this->joined)
  {
    // TODO: update isAlligned & checkProximity to work
    //       better in various orientations.
    if (this->isAlligned() && this->checkProximity(true))
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
      this->lockJoint(this->prismaticJoint);
    }
  }
  // If plug is locked to socket, see if there is
  // enough force being exerted to pull it out
  else if (this->joined && this->locked)
  {
    if (this->isEndEffectorPushingPlug())
    {
      this->unfreezeJoint(prismaticJoint);
    }
  }
}
