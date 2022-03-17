#!/usr/bin/env python

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import numpy as np

from gazebo_msgs.msg import ModelStates, ModelState

try:
    from math import pi, tau, dist, fabs, cos
except:
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterface(object):
    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()

        # Initialize `moveit_commander` and node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface", anonymous=True)

        # Instantiate a `RobotCommander` object. Provides robot's
        # kinematic model and current joint states
        self.robot = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface` object. Provides a remote interface
        # for getting, setting, and updating robot's state
        self.scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).
        self.move_group_hand_r = moveit_commander.MoveGroupCommander("hand_r")
        self.move_group_hand_l = moveit_commander.MoveGroupCommander("hand_l")
        self.move_group_arm_r = moveit_commander.MoveGroupCommander("arm_r")
        self.move_group_arm_l = moveit_commander.MoveGroupCommander("arm_l")

        ## Set planner
        self.move_group_arm_r.set_planner_id("RRTConnect")
        self.move_group_arm_l.set_planner_id("RRTConnect")

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # Print the entire state of the robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")

        # Misc variables
        self.box_name = ""

        # Logging starting pose. There are better ways to do this.
        self.arm_l_start = self.move_group_arm_l.get_current_pose().pose
        self.arm_r_start = self.move_group_arm_r.get_current_pose().pose

        self.rate = rospy.Rate(1.0)

    def move_gripper(self, gripper='right', command='open'):
        print()
        print(f"=========={gripper} gripper will {command}...==========")

        move_group = self.move_group_hand_r
        if gripper == "left":
            move_group = self.move_group_hand_l

        joint_goal = move_group.get_current_joint_values()
        print(f"gripper current joints:{joint_goal}")
        joint_goal[0] = 0.3
        joint_goal[2] = 0.3
        if command == 'close':
            joint_goal[0] = 0.11344640137963143
            joint_goal[2] = 0.11344640137963143

        move_group.go(joint_goal, wait=True)
        move_group.stop()
        print(self.robot.get_current_state())

    def move_arm(self, arm='right', joint_angles=[]):
        print()
        print(f"==========Moving {arm} to goal:{joint_angles}...==========")

        if len(joint_angles) < 6:
            print("Error: Fewer than 6 joint angles provided. Aborting...")
            return

        move_group = self.move_group_arm_r
        if arm == 'left':
            move_group = self.move_group_arm_l
        # Call the planner to compute the plan and execute it.
        plan = move_group.go(joint_angles, wait=True)
        # Call `stop()` to ensure there is no residual movement
        move_group.stop()
        # Clear your targets after planning with poses.
        move_group.clear_pose_targets()

        # current_pose = move_group.get_current_pose().pose
        # return all_close(pose_goal, current_pose, 0.01)
        print(self.robot.get_current_state())


    def cartesian_move_z(self, move_group, z, scale=1):
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.z += z
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0)
        move_group.execute(plan, wait=True)
        # return plan, fraction

def main():
    try:
        print("Setting up moveit commander")
        bimanual_demo = MoveGroupPythonInterface()
        bimanual_demo.move_gripper('left', 'close')
        bimanual_demo.move_arm(
          'left',
          [0.3021649574089649, 0.3450843028538877, 0.2316461488895012, -0.1962645695906149, -0.3577391278283173, 0.4782096174274093])
        bimanual_demo.move_gripper('left', 'open')

        # bimanual_demo.move_arm(
        #   'left',
        #   [0.31613485986845585,-0.002946032655327358,0.5855320626785936,-0.20069689279160094,-0.3650814889164338,0.4956174800839028])
        #works
        bimanual_demo.move_arm(
          'left',
          [0.31530026907915226,-0.03332714431637267,0.6262400013177802,-0.19507179964110966,-0.37505485279981504,0.4887380549805158])


        bimanual_demo.move_gripper('left', 'close')
        bimanual_demo.cartesian_move_z(bimanual_demo.move_group_arm_l, 0.05)

        bimanual_demo.move_arm(
          'left',
          [0.00013608091240816647,0.2668105697647139,-0.24472597921466507,0,-0.02203698399576485,5.7884153903040726e-05])

        bimanual_demo.move_arm(
          'left',
        [-0.4714430758247765,0.650333304630963,-0.23715665873003822,0.00014062477596432643,-0.41300602514875706,-0.4714349752352547])

        bimanual_demo.move_arm(
          'left',
          [-0.39313985588177913,0.6305118167315307,0.2497230080748888,0.00014004181736175308,-0.8802961517578444,-0.3932120366691911])

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()