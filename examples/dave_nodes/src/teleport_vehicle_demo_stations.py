#!/usr/bin/env python
'''
Teleport the vehicle to different demo stations

'''

# System imports
import math

# ROS/Gazebo imports
import rospy
import tf
from rosgraph_msgs.msg import Clock

# For GetModelState Service
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist, Transform

class Teleport:

    def __init__(self):

        # Parse parameters
        self.model_name = "rexrov"
        self.homeX = -21
        self.homeY = -34
        self.homeZ = -108

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

        rospy.loginfo("Vehicle will be teleported to (" \
            + repr(self.homeX) + "," + repr(self.homeY) + "," + repr(self.homeZ) + ")")

    def teleport(self):

        # Timing
        now = rospy.get_time()
        dt = now - self.t0
        if dt < 1.0e-5:
            return
        self.t0 = now

        # Generate ModelState msg
        pose = Pose()
        pose.position.x = self.homeX
        pose.position.y = self.homeY
        pose.position.z = self.homeZ

        # meneuver
        self.set_model_srv(ModelState(self.model_name, pose, Twist(), "world"))


if __name__ == '__main__':

    # Start node
    rospy.init_node('teleport_vehicle_demo_stations')
    Teleport = Teleport()

    # Update rate
    update_rate = rospy.get_param('~update_rate', 20.0)

    # Spin
    r = rospy.Rate(update_rate)
    #(TODO) run it only once
    try:
        while not rospy.is_shutdown():
            Teleport.teleport()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
