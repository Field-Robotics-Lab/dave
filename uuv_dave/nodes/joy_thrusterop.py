#!/usr/bin/env python
'''
Node to directly control the thrusters
'''
import os

import rospy
from sensor_msgs.msg import Joy

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

class ThrusterOp:
    def __init__(self, namespace='smilodon'):
        self.gain = 1000.0
        # Joystick to thruster i.d. mapping
        # Keys are the joystick axes, publishers
        self.joy2thrust = dict()
        self.joy2thrust[1] = rospy.Publisher('/%s/thrusters/%d/input'%(namespace,0), FloatStamped, queue_size=1)
        self.joy2thrust[4] = rospy.Publisher('/%s/thrusters/%d/input'%(namespace,1), FloatStamped, queue_size=1)
        self.joy2thrust[7] = rospy.Publisher('/%s/thrusters/%d/input'%(namespace,2), FloatStamped, queue_size=1)
        self.joy2thrust[6] = rospy.Publisher('/%s/thrusters/%d/input'%(namespace,3), FloatStamped, queue_size=1)
        self.joy2thrust[0] = rospy.Publisher('/%s/thrusters/%d/input'%(namespace,4), FloatStamped, queue_size=1)
        self.joy2thrust[3] = rospy.Publisher('/%s/thrusters/%d/input'%(namespace,5), FloatStamped, queue_size=1)

        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    def joy_callback(self, joy):
        msg = FloatStamped()
        for aa, ii in zip(joy.axes, range(len(joy.axes))):
            if ii in self.joy2thrust.keys():
                msg.data = aa*self.gain
                self.joy2thrust[ii].publish(msg)
                
if __name__ == '__main__':
    # Start the node
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    rospy.loginfo('Starting [%s] node' % node_name)

    # Get params
    ns = 'smilodon'
    if rospy.has_param('~namespace'):
        ns = rospy.get_param('~namespace')

    teleop = ThrusterOp(namespace = ns)
    rospy.spin()
    rospy.loginfo('Shutting down [%s] node' % node_name)
