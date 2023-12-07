#!/usr/bin/env python3
'''
Node to directly control the thrusters
'''
import os

import rospy
from sensor_msgs.msg import Joy

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

class ThrusterOp:
    def __init__(self, namespace='virgil'):        
        
        # max thrusters used for movement
        self.num_horiz_thrusters = 4
        self.num_vert_thrusters = 3

        # Put thruster gains into dict
        self.gain_dict = dict()
        self.gain_dict[1] = [1000.0, 1000.0, 1000.0]
        self.gain_dict[0] = [-250.0, 250.0, -250.0, -250.0]
        self.gain_dict[4] = [500.0, 500.0, 500.0, -500.0]
        self.gain_dict[3] = [700.0, -700.0, -750.0, -750.0]
        
        # Joystick to thruster i.d. mapping
        # Keys are the joystick axes, publishers
        self.joy2thrust = dict()
        
        # up down
        self.joy2thrust[1] = [rospy.Publisher('/%s/thrusters/%d/input'%(namespace,4), FloatStamped, queue_size=1),
                              rospy.Publisher('/%s/thrusters/%d/input'%(namespace,5), FloatStamped, queue_size=1),
                              rospy.Publisher('/%s/thrusters/%d/input'%(namespace,6), FloatStamped, queue_size=1)]
        # rotate
        self.joy2thrust[0] = [rospy.Publisher('/%s/thrusters/%d/input'%(namespace,0), FloatStamped, queue_size=1),
                              rospy.Publisher('/%s/thrusters/%d/input'%(namespace,1), FloatStamped, queue_size=1),
                              rospy.Publisher('/%s/thrusters/%d/input'%(namespace,2), FloatStamped, queue_size=1),
                              rospy.Publisher('/%s/thrusters/%d/input'%(namespace,3), FloatStamped, queue_size=1)]

        # c
        self.joy2thrust[4] = [rospy.Publisher('/%s/thrusters/%d/input'%(namespace,0), FloatStamped, queue_size=1),
                              rospy.Publisher('/%s/thrusters/%d/input'%(namespace,1), FloatStamped, queue_size=1),
                              rospy.Publisher('/%s/thrusters/%d/input'%(namespace,2), FloatStamped, queue_size=1),
                              rospy.Publisher('/%s/thrusters/%d/input'%(namespace,3), FloatStamped, queue_size=1)]

        # d
        self.joy2thrust[3] = [rospy.Publisher('/%s/thrusters/%d/input'%(namespace,0), FloatStamped, queue_size=1),
                              rospy.Publisher('/%s/thrusters/%d/input'%(namespace,1), FloatStamped, queue_size=1),
                              rospy.Publisher('/%s/thrusters/%d/input'%(namespace,2), FloatStamped, queue_size=1),
                              rospy.Publisher('/%s/thrusters/%d/input'%(namespace,3), FloatStamped, queue_size=1)]



        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


    def joy_callback(self, joy):
        # Need to combine horizontal thrusters before sending out the message
        msg = FloatStamped()
        aa_dict = dict()

        for aa, ii in zip(joy.axes, range(len(joy.axes))):
            aa_dict[ii] = aa

        key_list = list(self.joy2thrust.keys())
        msg_values = [0] * self.num_horiz_thrusters
        vert_values = [0] * self.num_vert_thrusters
        
        for key_val in key_list:
            # Keep vertical thrusters separate
            if key_val == 1:
                for ii in range(len(self.gain_dict[key_val])):
                    msg.data = aa_dict[key_val] * self.gain_dict[key_val][ii]
                    self.joy2thrust[key_val][ii].publish(msg)
            # Mix 4 corner thrusters
            else:
                for ii in range(len(self.gain_dict[key_val])):
                    msg_values[ii] += aa_dict[key_val] * self.gain_dict[key_val][ii]

        for ii in range(len(self.joy2thrust[key_list[-1]])):
            msg.data = msg_values[ii]
            self.joy2thrust[key_list[-1]][ii].publish(msg)
            
                
if __name__ == '__main__':
    # Start the node
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    rospy.loginfo('Starting [%s] node' % node_name)

    # Get params
    ns = 'virgil'
    if rospy.has_param('~namespace'):
        ns = rospy.get_param('~namespace')

    teleop = ThrusterOp(namespace = ns)
    rospy.spin()
    rospy.loginfo('Shutting down [%s] node' % node_name)
