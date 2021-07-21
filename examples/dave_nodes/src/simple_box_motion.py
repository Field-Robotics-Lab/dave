#!/usr/bin/env python

#----------------------------------------------------------------------------
# Instantiates a ROS node that sets velocities for a model in the gazebo
# scene.  The velocity is set relative to the a specified base link to drive
# in an octagonal pattern (forward, forward left, left,# backwards left,
# backwards, backwards right, right, roward right) at a fixed linear speed
# (1 meter/second).  Descents and climbs at at between 0.1 and 0.4 meters per
# second are included in some legs resulting in a slow descent over time.
# Each leg is allowed to run for 10 seconds.  The motion is intended to test
# or demonstrate the DVL plugin water and bottom tracking, in particular, the
# inclusion of ocean current in the water tracking solution.
#----------------------------------------------------------------------------

import rospy
import gazebo_msgs.msg as gm
import time
import math

TOPIC_NAME = 'gazebo/set_model_state'

if __name__ == '__main__':
    diagonal = math.sqrt(0.5)   # diagonal components for 1 m/s speed
    rospy.init_node('apply_velocity')
    rate = rospy.Rate(0.1)
    publisher = rospy.Publisher(TOPIC_NAME, gm.ModelState, queue_size=1)
    command = gm.ModelState()
    command.model_name = rospy.get_param('model_name')
    command.reference_frame = rospy.get_param('base_link_name')
    time.sleep(10)  # Give things time to start up
    while not rospy.is_shutdown():
        for (command.twist.linear.x, \
             command.twist.linear.y, \
             command.twist.linear.z) in \
            ((1.0, 0.0, -0.4), (diagonal, diagonal, 0.0), \
             (0.0, 1.0, 0.2), (-diagonal, diagonal, 0.0), \
             (-1.0, 0.0, -0.3), (-diagonal, -diagonal, 0.0), \
             (0.0, -1.0, 0.1), (diagonal, -diagonal, 0.0)):
             publisher.publish(command)
             rate.sleep()

