#!/usr/bin/env python

#-----------------------------------------------------------------------
# Instantiates a ROS node that sets a fixed velocity to a standalone
# Teledyne WHN DVL model in the gazebo scene.  The velocity is set
# relative to the DVL base link at a rate of 50 hertz to effect a slow
# right hand turn (0.25 radians per second) with a forward velocity of
# 1 meter per second and a descent rate of 0.5 meters per scend
#-----------------------------------------------------------------------

import rospy
import gazebo_msgs.msg as gm
import time

TOPIC_NAME = 'gazebo/set_model_state'
#MODEL_NAME = 'teledyne_whn'
#LINK_NAME = 'dvl_link'

if __name__ == '__main__':
    rospy.init_node('apply_velocity')
    rate = rospy.Rate(50)
    publisher = rospy.Publisher(TOPIC_NAME, gm.ModelState, queue_size=1)
    command = gm.ModelState()
    command.model_name = rospy.get_param('model_name')
    command.reference_frame = rospy.get_param('base_link_name')
    print("****** Frame: %s"%command.reference_frame)
    command.twist.linear.x = 1.0
    command.twist.linear.z = -0.5
    command.twist.angular.z = 0.25
    time.sleep(10)  # Give things time to start up
    while not rospy.is_shutdown():
        publisher.publish(command)
        rate.sleep()
