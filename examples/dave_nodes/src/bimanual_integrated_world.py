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

        self.rate = rospy.Rate(1)

    def move_gripper(self, gripper='right', command='open'):
        print()
        print(f"=========={gripper} gripper will {command}...==========")

        move_group = self.move_group_hand_r
        if gripper == "left":
            move_group = self.move_group_hand_l

        joint_goal = move_group.get_current_joint_values()
        print(f"gripper current joints:{joint_goal}")
        joint_goal[0] = 0.5
        joint_goal[2] = 0.5
        if command == 'close':
            joint_goal[0] = 0.0
            joint_goal[2] = 0.0

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

        move_group = self.move_group
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1
        wpose.position.y += scale * 0.2
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y -= scale * 0.1
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0)

        return plan, fraction

def main():
    try:
        print("Setting up moveit commander")
        bimanual_demo = MoveGroupPythonInterface()
        bimanual_demo.move_gripper('left', 'close')
        bimanual_demo.move_arm(
          'left',
          [0.307825322102496, 0.36508188828997135, 0.17294171513504075, -0.22269999224694015, -0.32034474054001905, 0.5113433813642088])
        bimanual_demo.move_gripper('left', 'open')

        # bimanual_demo.move_arm(
        #   'left',
        #   [0.3491, 0.2618, 0.2269, -0.2094, -0.2967, 0.5585])
        # bimanual_demo.move_arm(
        #   'left',
        #   [0.3491, 0.1222, 0.2967, -0.2793, -0.2269, 0.6458])

        # bimanual_demo.move_gripper('left', 'close')
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()