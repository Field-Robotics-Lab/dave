#!/usr/bin/env python
'''
Maneuver the vehicle like merry-go-round

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

class MerryGoRound:

    def __init__(self):

        # Parse parameters
        self.model_name = "rexrov"
        self.xCenter = -1400
        self.yCenter = -300
        self.radius = 3000
        self.depth = 0
        self.angularSpeed = 50

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
        rospy.loginfo("Starting Merry-Go-Round maneuver...")
        rospy.wait_for_service("/gazebo/set_model_state")
        wait_for_sim_to_start = rospy.wait_for_message('/clock', Clock)

        # Prepare initialization SetModelState service
        self.set_model_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        rospy.loginfo("Vehicle will be maneuvered to rotate with (" \
            + repr(self.xCenter) + "," + repr(self.yCenter) + ") with radius of " + repr(self.radius) \
            + " [m] for " + repr(self.angularSpeed) + " [deg/sim_sec] angular speed at " + repr(self.depth) \
            + " [m] depth")

    def maneuver(self):

        # Timing
        now = rospy.get_time()
        dt = now - self.t0
        if dt < 1.0e-5:
            # rospy.logwarn("Timestep is too small (%f) - skipping this update"
            #               %dt)
            return
        self.t0 = now

        # Generate ModelState msg
        pose = Pose()
        pose.position.x = self.xCenter + self.radius*math.sin(now*self.angularSpeed/180.0*math.pi)
        pose.position.y = self.yCenter + self.radius*math.cos(now*self.angularSpeed/180.0*math.pi)
        pose.position.z = self.depth

        # meneuver
        self.set_model_srv(ModelState(self.model_name, pose, Twist(), "world"))

if __name__ == '__main__':

    # Start node
    rospy.init_node('merry_go_round')
    MerryGoRound = MerryGoRound()

    # Update rate
    update_rate = rospy.get_param('~update_rate', 20.0)

    # Spin
    r = rospy.Rate(update_rate)
    try:
        while not rospy.is_shutdown():
            MerryGoRound.maneuver()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
