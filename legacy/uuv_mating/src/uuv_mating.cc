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
#include "uuv_mating/uuv_mating.hh"

using namespace gazebo;

//////////////////////////////////////////////////
void WorldUuvPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->world = _world;

  // Retrieve models' parameters from SDF
  if (_sdf->HasElement("rollAlignmentTolerence"))
  {
    this->rollAlignmentTolerence =
      _sdf->GetElement("rollAlignmentTolerence")->Get<double>();
    ROS_INFO("Roll Mating Alignment Tolerence is: %f",
             this->rollAlignmentTolerence);
  }
  else
  {
    this->rollAlignmentTolerence = 0.3;
    ROS_INFO("Roll Mating Alignment Tolerence was not stated, "
             "using value of %f", this->rollAlignmentTolerence);
  }

  if (_sdf->HasElement("pitchAlignmentTolerence"))
  {
    this->pitchAlignmentTolerence =
      _sdf->GetElement("pitchAlignmentTolerence")->Get<double>();
    ROS_INFO("Pitch Mating Alignment Tolerence is: %f",
             this->pitchAlignmentTolerence);
  }
  else
  {
    this->pitchAlignmentTolerence = 0.3;
    ROS_INFO("Pitch Mating Alignment Tolerence was not stated, "
             "using value of %f", this->pitchAlignmentTolerence);
  }

  if (_sdf->HasElement("yawAlignmentTolerence"))
  {
    this->yawAlignmentTolerence =
      _sdf->GetElement("yawAlignmentTolerence")->Get<double>();
    ROS_INFO("Yaw Mating Alignment Tolerence is: %f",
             this->yawAlignmentTolerence);
  }
  else
  {
    this->yawAlignmentTolerence = 0.3;
    ROS_INFO("Yaw Mating Alignment Tolerence was not stated, "
             "using value of %f", this->yawAlignmentTolerence);
  }

  if (_sdf->HasElement("zAlignmentTolerence"))
  {
    this->zAlignmentTolerence =
      _sdf->GetElement("zAlignmentTolerence")->Get<double>();
    ROS_INFO("Z Mating Alignment Tolerence is: %f", this->zAlignmentTolerence);
  }
  else
  {
    this->zAlignmentTolerence = 0.1;
    ROS_INFO("Yaw Mating Alignment Tolerence was not stated, "
             "using value of %f", this->zAlignmentTolerence);
  }

  if (_sdf->HasElement("matingForce"))
  {
    this->matingForce = _sdf->GetElement("matingForce")->Get<double>();
    ROS_INFO("Mating force: %f", this->matingForce);
  }
  else
  {
    this->matingForce = 50;
    ROS_INFO("Mating Force not stated, using value of %f", this->matingForce);
  }

  if (_sdf->HasElement("unmatingForce"))
  {
    this->unmatingForce = _sdf->GetElement("unmatingForce")->Get<double>();
    ROS_INFO("Unmating Force: %f", this->unmatingForce);
  }
  else
  {
    this->unmatingForce = 190;
    ROS_INFO("Unmating Force not stated, using value of %f",
             this->unmatingForce);
  }

  this->socketModel = this->world->ModelByName("socket_box");
  this->plugModel = this->world->ModelByName("plug");
  this->sensorPlate = this->socketModel->GetLink("sensor_plate");
  this->tubeLink = this->socketModel->GetLink("socket");
  this->plugLink = this->plugModel->GetLink("plug");
  this->world->Physics()->GetContactManager()->SetNeverDropContacts(true);
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&WorldUuvPlugin::Update, this));
}

//////////////////////////////////////////////////
void WorldUuvPlugin::trimForceVector(double trimDuration)
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
double WorldUuvPlugin::movingTimedAverage()
{
  return accumulate(this->forcesBuffer.begin(),
                    this->forcesBuffer.end(), 0.0) / this->forcesBuffer.size();
};

//////////////////////////////////////////////////
void WorldUuvPlugin::addForce(double force)
{
  if (abs(force) < 5.0)
    return;

  this->forcesBuffer.push_back(force);
  this->timeStamps.push_back(this->world->SimTime());
}

//////////////////////////////////////////////////
WorldUuvPlugin::WorldUuvPlugin() : WorldPlugin()
{
}

//////////////////////////////////////////////////
void WorldUuvPlugin::lockJoint(physics::JointPtr prismaticJoint)
{
  if (this->locked)
  {
    ROS_INFO("already locked!");
    return;
  }
  this->locked = true;
  ROS_INFO("locked!");
  double currentPosition = prismaticJoint->Position(0);
  prismaticJoint->SetUpperLimit(0, currentPosition);
  prismaticJoint->SetLowerLimit(0, currentPosition);
}

//////////////////////////////////////////////////
void WorldUuvPlugin::unfreezeJoint(physics::JointPtr prismaticJoint)
{
  if (!this->locked)
  {
    ROS_INFO("Already unlocked");
    return;
  }
  this->locked = false;
  this->unfreezeTimeBuffer =  this->world->SimTime();
  ROS_INFO("Unfreeze joint");
  this->remove_joint();
}

//////////////////////////////////////////////////
bool WorldUuvPlugin::checkRollAlignment(double alignmentThreshold)
{
  ignition::math::Vector3<double> socketRotation =
                             socketModel->RelativePose().Rot().Euler();
  ignition::math::Vector3<double> plugRotation =
                               plugModel->RelativePose().Rot().Euler();
  return abs(plugRotation[0] - socketRotation[0]) < alignmentThreshold;
}

//////////////////////////////////////////////////
bool WorldUuvPlugin::checkPitchAlignment(double alignmentThreshold)
{
  ignition::math::Vector3<double> socketRotation =
                             socketModel->RelativePose().Rot().Euler();
  ignition::math::Vector3<double> plugRotation =
                               plugModel->RelativePose().Rot().Euler();
  return abs(plugRotation[1] - socketRotation[1]) < alignmentThreshold;
}

//////////////////////////////////////////////////
bool WorldUuvPlugin::checkYawAlignment(double alignmentThreshold)
{
  ignition::math::Vector3<double> socketRotation =
                         socketModel->RelativePose().Rot().Euler();
  ignition::math::Vector3<double> plugRotation =
                           plugModel->RelativePose().Rot().Euler();
  return abs(plugRotation[2]+1.57079632679 - socketRotation[2]) <
                                                alignmentThreshold;
}

//////////////////////////////////////////////////
bool WorldUuvPlugin::checkRotationalAlignment(bool verbose)
{
  if (verbose)
  {
    ignition::math::Vector3<double> socketRotation =
                         socketModel->RelativePose().Rot().Euler();
    ignition::math::Vector3<double> plugRotation =
                           plugModel->RelativePose().Rot().Euler();
    ROS_INFO_THROTTLE(1, "socket euler: %.2f %.2f %.2f "
                         "plug euler: %.2f %.2f %.2f",
                      socketRotation[0], socketRotation[1], socketRotation[2],
                      plugRotation[0], plugRotation[1],
                      plugRotation[2]+1.57079632679);
  }
  if (this->checkYawAlignment(this->yawAlignmentTolerence) &&
      this->checkPitchAlignment(this->pitchAlignmentTolerence) &&
      this->checkRollAlignment(this->rollAlignmentTolerence))
  {
    ROS_INFO_THROTTLE(1, "Socket and plug are aligned");
    return true;
  }
  else
  {
    return false;
  }
}

//////////////////////////////////////////////////
bool WorldUuvPlugin::checkVerticalAlignment(double alignmentThreshold,
                                            bool verbose)
{
  ignition::math::Pose3d socket_pose = this->tubeLink->WorldPose();
  ignition::math::Vector3<double> socketPositon = socket_pose.Pos();
  ignition::math::Pose3d plug_pose = plugModel->RelativePose();
  ignition::math::Vector3<double> plugPosition = plug_pose.Pos();
  bool onSameVerticalLevel =
    abs(plugPosition[2] - socketPositon[2]) < alignmentThreshold;

  if (verbose)
    ROS_INFO_THROTTLE(1, "Z plug: %f  Z socket: %f",
                      plugPosition[2], socketPositon[2]);

  if (onSameVerticalLevel)
    return true;

  return false;
}

//////////////////////////////////////////////////
bool WorldUuvPlugin::isAlligned(bool verbose)
{
  if (checkVerticalAlignment(true) && checkRotationalAlignment())
  {
    if (verbose)
      ROS_INFO_THROTTLE(1, "Plug and socket are aligned in "
                           "orientation and altitude");
    return true;
  }
  else
  {
    return false;
  }
}

//////////////////////////////////////////////////
bool WorldUuvPlugin::checkProximity(bool verbose)
{
  ignition::math::Pose3d socket_pose = this->tubeLink->WorldPose();
  ignition::math::Vector3<double> socketPositon = socket_pose.Pos();
  ignition::math::Pose3d plug_pose = plugModel->RelativePose();
  ignition::math::Vector3<double> plugPosition = plug_pose.Pos();
  float xdiff_squared = pow(abs(plugPosition[0] - socketPositon[0]), 2);
  float ydiff_squared = pow(abs(plugPosition[1] - socketPositon[1]), 2);
  float zdiff_squared = pow(abs(plugPosition[2] - socketPositon[2]), 2);

  if (verbose)
    ROS_INFO_THROTTLE(1, "eucleadian distance: %f",
                      pow(xdiff_squared+ydiff_squared+zdiff_squared, 0.5));

  bool withinProximity =
    pow(xdiff_squared+ydiff_squared+zdiff_squared, 0.5) < 0.14;
  if (withinProximity)
  {
    ROS_INFO_THROTTLE(1, "Within proximity");
    return true;
  }
  else
  {
    ROS_INFO_THROTTLE(1, "Not within proximity, please more the plug closer");
  }
  return false;
}

//////////////////////////////////////////////////
void WorldUuvPlugin::construct_joint()
{
  if (this->joined)
  {
    ROS_INFO_THROTTLE(1, "already frozen");
    return;
  }
  this->joined = true;
  this->alignmentTime = 0;
  this->prismaticJoint = plugModel->CreateJoint(
    "plug_socket_joint",
    "prismatic",
    tubeLink,
    plugLink);
  prismaticJoint->Load(this->tubeLink, this->plugLink,
                        ignition::math::Pose3<double>(
                          ignition::math::Vector3<double>(0, 0, 0),
                          ignition::math::Quaternion<double>(0, 0, 0, 0)));
  prismaticJoint->Init();
  prismaticJoint->SetAxis(0, ignition::math::Vector3<double>(1, 0, 0));
  prismaticJoint->SetLowerLimit(0, prismaticJoint->Position(0));
  ROS_INFO("joint formed\n");
}

//////////////////////////////////////////////////
void WorldUuvPlugin::remove_joint()
{
  if (this->joined == true)
  {
    this->joined = false;
    this->prismaticJoint->Detach();
    this->prismaticJoint->Reset();
    this->prismaticJoint->~Joint();
    ROS_INFO("Joint removed");
  }
}

//////////////////////////////////////////////////
bool WorldUuvPlugin::averageForceOnLink(std::string contact1,
                                        std::string contact2)
{
  int contactIndex = this->getCollisionBetween(contact1, contact2);
  if (contactIndex == -1)
      return false;
  physics::Contact *contact =
    this->world->Physics()->GetContactManager()->GetContact(contactIndex);
  if (contact->collision1->GetLink()->GetName() == contact1)
  {
      this->addForce(contact->wrench[contactIndex].body1Force[1]);
  }
  else
  {
      this->addForce(contact->wrench[contactIndex].body2Force[1]);
  }
  this->trimForceVector(0.1);
  return true;
}

//////////////////////////////////////////////////
bool WorldUuvPlugin::isPlugPushingSensorPlate(int numberOfDatapointsThresh)
{
  if (!this->averageForceOnLink("plug", "sensor_plate"))
  {
      return false;
  }
  else
  {
    double averageForce = this->movingTimedAverage();
    if ((averageForce > this->matingForce) &&
        (this->forcesBuffer.size() > numberOfDatapointsThresh))
    {
      // ROS_INFO("sensor plate average: %f, size %i",
      //          averageForce, this->forcesBuffer.size());
      this->forcesBuffer.clear();
      this->timeStamps.clear();
      return true;
    }
  }
}

//////////////////////////////////////////////////
bool WorldUuvPlugin::isEndEffectorPushingPlug(int numberOfDatapointsThresh)
{
  if (!this->averageForceOnLink("plug", "finger_tip"))
  {
    return false;
  }
  else
  {
    double averageForce = this->movingTimedAverage();
    if ((averageForce > this->unmatingForce) &&
        (this->forcesBuffer.size() > numberOfDatapointsThresh))
    {
      // ROS_INFO("end effector average: %f, size %i",
      //          averageForce, this->forcesBuffer.size());
      this->forcesBuffer.clear();
      this->timeStamps.clear();
      return true;
    }
  }
}

//////////////////////////////////////////////////
int WorldUuvPlugin::getCollisionBetween(std::string contact1,
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

      if ((contact->collision1->GetLink()->GetName().find(contact1) !=
                                                          std::string::npos) &&
          (contact->collision2->GetLink()->GetName().find(contact2) !=
                                                          std::string::npos))
      {
        ROS_INFO_THROTTLE(1, "%s %s",
                          contact->collision1->GetLink()->GetName().c_str(),
                          contact->collision2->GetLink()->GetName().c_str());
      }
      else if ((contact->collision1->GetLink()->GetName().find(contact2) !=
                                                           std::string::npos) &&
               (contact->collision2->GetLink()->GetName().find(contact1) !=
                                                             std::string::npos))
      {
        ROS_INFO_THROTTLE(1, "%s %s",
                          contact->collision1->GetLink()->GetName().c_str(),
                          contact->collision2->GetLink()->GetName().c_str());
      }

    if (isPlugContactingSensorPlate)
        return i;
  }
  return -1;
}

//////////////////////////////////////////////////
void WorldUuvPlugin::Update()
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
    if (this->isAlligned() && this->checkProximity(true))
    {
        if (alignmentTime == 0)
        {
          alignmentTime = this->world->SimTime();
        }
        else if (this->world->SimTime() - alignmentTime > 2)
        {
            this->construct_joint();
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
    // If plug is locked to socket, probe to see if there is
    // enough force being exerted to pull it out
  }
  else if (this->joined && this->locked)
  {
    if (this->isEndEffectorPushingPlug())
    {
        this->unfreezeJoint(prismaticJoint);
    }
  }
}
