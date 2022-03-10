#!/usr/bin/env python
'''
Teleport the vehicle to different locations in any world

'''

# System imports
import sys, os
import argparse, yaml

# ROS/Gazebo imports
import rospy
import tf

# For GetModelState Service
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist

class Teleport:

    def __init__(self):
        # Start node
        rospy.init_node('teleport_vehicle')

        # Initialize vars
        self.locationDict = {}

    def main(self, configFileName):
        # Parse yaml for locations
        path = os.getcwd() + "/examples/dave_nodes/config/" + configFileName
        self.locationDict = yaml.load(open(path), Loader=yaml.FullLoader)
        
        # Parser for user commands
        rospy.loginfo("Teleporter ready: Waiting for commands")
        parser = argparse.ArgumentParser()
        parser.add_argument('vehicle_type', help="Vehicle's model name", type=str)
        parser.add_argument('location_name', help="Which location it should move to?", type=str)
        try:
            while not rospy.is_shutdown():
                # Wait for user input
                print("<vehicle> <location>: ", end="")
                user_input = input()
                args = parser.parse_args(user_input.split())

                # Check for valid model name
                if args.vehicle_type in self.locationDict:
                    # Move vehicle
                    self.teleport(args.vehicle_type, args.location_name)
                else:
                    rospy.logerr("Invalid vehicle. Choose from: ["+",".join(self.locationDict.keys())+"]")
                    pass
        except (rospy.ROSInterruptException, KeyboardInterrupt):
            sys.exit()

    def teleport(self, vehicle, location):
        # Wait for Gazebo services
        rospy.wait_for_service("/gazebo/set_model_state")

        # Prepare initialization SetModelState service
        self.set_model_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        thisVehicle = self.locationDict[vehicle]['places']
        # Generate ModelState msg based on valid location id
        try:
            pose = Pose()
            pose.position.x = thisVehicle[location][0]
            pose.position.y = thisVehicle[location][1]
            pose.position.z = thisVehicle[location][2]
            quaternion = tf.transformations.quaternion_from_euler(thisVehicle[location][3],\
                         thisVehicle[location][4], thisVehicle[location][5])
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]

            # maneuver
            self.set_model_srv(ModelState(vehicle, pose, Twist(), "world"))

            rospy.loginfo("Teleported " + vehicle + " to " + location + " location at pose (" \
                + repr(pose.position.x) + "," + repr(pose.position.y) + "," + repr(pose.position.z) \
                + ") and" + " orientation ("+ repr(thisVehicle[location][3]) + ","\
                + repr(thisVehicle[location][4]) + "," + repr(thisVehicle[location][5]) + ")")

        except KeyError:
            rospy.logerr("Invalid location. Choose from: ["+",".join(thisVehicle.keys())+"]")
            pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('config_filename', help="configuration file *.yaml", type=str)
    args = parser.parse_args()
    
    teleport = Teleport()
    teleport.main(args.config_filename)
