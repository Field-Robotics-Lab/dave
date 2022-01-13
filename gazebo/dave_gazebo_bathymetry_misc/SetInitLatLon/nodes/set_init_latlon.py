#!/usr/bin/env python
'''
Set vehicle initial location using lat/lon

'''

# System imports
from math import *

# ROS/Gazebo imports
import rospy
import tf
from rosgraph_msgs.msg import Clock

# For GetModelState Service
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist, Transform

# For lat/lon coordinate frame transformations
from osgeo import ogr
from osgeo import osr

class SetInitLatLon:

    def __init__(self):

        # Parse parameters
        self.model_name = rospy.get_param('~namespace', "rexrov")
        self.init_lat = rospy.get_param('~init_lat', 36.790)
        self.init_lon = rospy.get_param('~init_lon', -121.815)
        self.depth = rospy.get_param('~depth', -5.0)

        # Calculate center coordinates in X/Y (UTM; epsg3857)
        source = osr.SpatialReference()
        source.ImportFromEPSG(4326)
        target = osr.SpatialReference()
        target.ImportFromEPSG(3857)
        transform = osr.CoordinateTransformation(source, target)
        point = ogr.CreateGeometryFromWkt("POINT (" + repr(self.init_lat) + " " + repr(self.init_lon) + ")")
        point.Transform(transform)
        self.xCoord = point.ExportToWkt().split(" ")[1].split("(")[1]
        self.yCoord = point.ExportToWkt().split(" ")[2].split(")")[0]

        # Wait for ROS init
        self.t0 = rospy.get_time()
        while self.t0 < 0.01:
            rospy.logwarn("Waiting for ROS time to be received")
            rospy.sleep(0.5)
            self.t0 = rospy.get_time()
            self.t0_init = self.t0

        # Launch
        self.attempt()

    def attempt(self):

       # Wait for Gazebo services and simulation start
        rospy.wait_for_service("/gazebo/set_model_state")
        wait_for_sim_to_start = rospy.wait_for_message('/clock', Clock)

        # Prepare initialization SetModelState service
        set_model_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        # Generate ModelState msg
        pose = Pose()
        pose.position.x = float(self.xCoord)
        pose.position.y = float(self.yCoord)
        pose.position.z = float(self.depth)

        # meneuver
        success = False
        while success is False:
            try:
                success = set_model_srv(ModelState(self.model_name, pose, Twist(), "world"))
            except rospy.ROSInterruptException:
                print("Could not move the vehicle to initial position")

        rospy.loginfo("Initial vehicle position set at (lat/lon) = (" \
            + repr(self.init_lat) + "," + repr(self.init_lon) + ")")

if __name__ == '__main__':

    # Start node
    rospy.init_node('set_init_latlon')
    node = SetInitLatLon()

    rospy.spin()
