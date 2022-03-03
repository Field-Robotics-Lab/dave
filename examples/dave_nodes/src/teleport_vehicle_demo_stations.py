#!/usr/bin/env python
'''
Teleport the vehicle to different demo stations

'''

# System imports
import math

# ROS/Gazebo imports
import rospy
import tf
import sys
from rosgraph_msgs.msg import Clock

# For GetModelState Service
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist

class Teleport:

    def __init__(self):

        # Parameters
        self.model_name = "rexrov"
        # Demo station vehicle location [x,y,z,roll,pitch,yaw]
        self.demoLocationDict = {'home':[-21,-34,-108,0,0,2.5],
                                 'electrical':[-30,-34,-108,0,0,3.141592],
                                 'mud':[-15,-34,-120,0,0,0]}

        # Wait for ROS init
        self.t0 = rospy.get_time()
        while self.t0 < 0.01:
            rospy.logwarn("Waiting for ROS time to be received")
            rospy.sleep(0.5)
            self.t0 = rospy.get_time()
            self.t0_init = self.t0

        # Launch
        self.launch()

    def launch(self):

        # Wait for Gazebo services
        rospy.loginfo("Teleporting...")
        rospy.wait_for_service("/gazebo/set_model_state")
        wait_for_sim_to_start = rospy.wait_for_message('/clock', Clock)

        # Prepare initialization SetModelState service
        self.set_model_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    def teleport(self, station):
        # Timing
        now = rospy.get_time()
        dt = now - self.t0
        if dt < 1.0e-5:
            return
        self.t0 = now

        try:
            # Generate ModelState msg based on valid input
            pose = Pose()
            pose.position.x = self.demoLocationDict[station][0]
            pose.position.y = self.demoLocationDict[station][1]
            pose.position.z = self.demoLocationDict[station][2]
            quaternion = tf.transformations.quaternion_from_euler(self.demoLocationDict[station][3],\
                         self.demoLocationDict[station][4], self.demoLocationDict[station][5])
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
        except KeyError:
            rospy.logerr("Invalid demo station. Choose from: ["+",".join(self.demoLocationDict.keys())+"]")
            sys.exit()

        # maneuver
        self.set_model_srv(ModelState(self.model_name, pose, Twist(), "world"))

        rospy.loginfo("Vehicle is teleported to " + station + " location at pose (" \
            + repr(pose.position.x) + "," + repr(pose.position.y) + "," + repr(pose.position.z) \
            + ") and" + " orientation ("+ repr(self.demoLocationDict[station][3]) + ","\
            + repr(self.demoLocationDict[station][4]) + "," + repr(self.demoLocationDict[station][5]) + ")")


if __name__ == '__main__':

    # Start node
    rospy.init_node('teleport_vehicle_demo_stations')
    Teleport = Teleport()

    # Parse argument
    station = str(sys.argv[1:][0])

    # Move vehicle
    Teleport.teleport(station)
