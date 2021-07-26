#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
from ds_sensor_msgs.msg import Dvl

import numpy as np
import math
from tf.transformations import euler_from_quaternion


'''
Class for moving a DVL sensor through an underwater environment using
set_model_state and get_model_state functions. Estimates slope of seabed
over which it is moving.

- Publishes direction and magnitude of gradient as a geometry_msgs.msg Point
    /dvl_gradient

- Subscribes to
    Sonar values
    /dvl/dvl
    Keyboard control
    /cmd_vel
    Gazebo model pose
    /gazebo/model_states

Author: Carson Vogt, crvogt@nps.edu
'''


class DVLGradMap:
    def __init__(self):
        self.initNode()
        self.rate = rospy.Rate(1)

        # TODO Why do we need this?
        self.correctionFact = 1. # 0.92

        self.sin30 = 0.5
        self.cos30 = 0.86602540378

        self.runVals = np.zeros(4)
        self.riseVals = np.zeros(4)
        self.prevTwistCmd = 0

        # Slope values for opposite sonars
        self.slopePoint = Point()
        self.slope02 = 0
        self.slope13 = 0
        self.relYaw = 0

        self.sonarVals = Dvl()
        self.setStateMsg = ModelState()
        self.getStateMsg = ModelStates()
        self.keyboardTwist = Twist()

        # Subscribers
        # Sonar
        self.dvlSub = rospy.Subscriber('/whn/dvl', Dvl, self.dvlCB)
        # Model state
        self.modelStateSub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.getStateCB)
        # Get command vel
        self.keyboardTwistSub = rospy.Subscriber('/cmd_vel', Twist, self.keyTwistCB)

        # Services
        self.serviceName = '/gazebo/set_model_state'
        self.setStateSrv = rospy.ServiceProxy(self.serviceName, SetModelState)

        # Publisher
        self.gradPointPub = rospy.Publisher('dvl_gradient', Point, queue_size=10)

    def initNode(self):
        print('Creating Node')
        rospy.init_node('whoi_dvl_command')

    def dvlCB(self, _data):
        self.sonarVals = _data

    def getStateCB(self, _data):
        self.getStateMsg = _data

    def keyTwistCB(self, _data):
        self.keyboardTwist = _data

    def runNode(self):
        print('Running Node')
        while not rospy.is_shutdown():
            # Wait until we are properly receiving data
            if len(self.getStateMsg.name) > 0:
                # print(len(self.sonarVals), self.sonarVals[0].range, type(self.sonarVals[0]))
                # print()
                # Get model name
                self.setStateMsg.model_name = self.getStateMsg.name[-1]
                # Get model pose, position
                self.setStateMsg.pose.position.x = self.getStateMsg.pose[-1].position.x
                self.setStateMsg.pose.position.y = self.getStateMsg.pose[-1].position.y
                self.setStateMsg.pose.position.z = self.getStateMsg.pose[-1].position.z
                # Get model pose, orientation
                self.setStateMsg.pose.orientation.x = self.getStateMsg.pose[-1].orientation.x
                self.setStateMsg.pose.orientation.y = self.getStateMsg.pose[-1].orientation.y
                self.setStateMsg.pose.orientation.z = self.getStateMsg.pose[-1].orientation.z
                self.setStateMsg.pose.orientation.w = self.getStateMsg.pose[-1].orientation.w

                # As per https://github.com/ros/geometry/issues/109
                explicitQuaterion = [self.getStateMsg.pose[-1].orientation.x,
                                     self.getStateMsg.pose[-1].orientation.y,
                                     self.getStateMsg.pose[-1].orientation.z,
                                     self.getStateMsg.pose[-1].orientation.w]
                self.relYaw = euler_from_quaternion(explicitQuaterion)[2]

                # Check that a new twist command has been sent
                if self.keyboardTwist != self.prevTwistCmd:
                    self.setStateMsg.twist.linear.x = self.keyboardTwist.linear.x
                    self.setStateMsg.twist.linear.y = self.keyboardTwist.linear.y
                    self.setStateMsg.twist.linear.z = self.keyboardTwist.linear.z

                    self.setStateMsg.twist.angular.x = self.keyboardTwist.angular.x
                    self.setStateMsg.twist.angular.y = self.keyboardTwist.angular.y
                    self.setStateMsg.twist.angular.z = self.keyboardTwist.angular.z

                    rospy.wait_for_service(self.serviceName)
                    self.setStateSrv(self.setStateMsg)

                # Keep track, change twist command only when new one is sent
                self.prevTwistCmd = self.keyboardTwist
                # Calculate the gradient
                self.calculateGradient()

            else:
                print('Waiting for model_state topic to be published')

            self.rate.sleep()

    def calculateGradient(self):
        # Calculate the gradient for the sensor
        # Get rise and run values for the orthogonal pairs

        for iVal in range(4):
            # print(self.sonarVals[iVal].range)
            self.runVals[iVal] = self.sonarVals.range[iVal] * self.sin30
            self.riseVals[iVal] = self.sonarVals.range[iVal] * self.cos30

        # print(self.runVals)
        # print(self.riseVals)
        # print()
        run13 = self.runVals[1] + self.runVals[3]
        run02 = self.runVals[0] + self.runVals[2]

        # Want positive slope when neg side sensor is greater than pos side
        rise13 = self.riseVals[3] - self.riseVals[1]
        rise02 = self.riseVals[2] - self.riseVals[0]

        self.slope02 = (rise02 / run02) / self.correctionFact
        self.slope13 = (rise13 / run13) / self.correctionFact

        # Populate point with slopes and magnitude
        self.slopePoint.x = self.slope02
        self.slopePoint.y = self.slope13
        self.slopePoint.z = math.sqrt((self.slope13**2)+(self.slope02**2))

        # Calculate direction relative to world coords (0, 0, 0, 1)
        # Add a shift due to current orientation of DVL
        self.relYaw -= np.pi / 4
        cs = math.cos(self.relYaw)
        sn = math.sin(self.relYaw)

        sx = self.slopePoint.x * cs - self.slopePoint.y * sn
        sy = self.slopePoint.x * sn + self.slopePoint.y * cs

        self.slopePoint.x = sx
        self.slopePoint.y = sy

        # Publish the slope and magnitude
        self.gradPointPub.publish(self.slopePoint)


dvlGM = DVLGradMap()
dvlGM.runNode()
