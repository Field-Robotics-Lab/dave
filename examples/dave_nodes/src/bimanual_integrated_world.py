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
        joint_goal[0] = 0.5
        joint_goal[1] = 0.0
        joint_goal[2] = 0.5
        joint_goal[3] = 0.0
        if command == 'close':
            close = 0.17453292519943295
            # 0.1144640137963143
            joint_goal[0] = close/10.0
            joint_goal[2] = close

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
          [0.2725458466968526, 0.3199954120186353, 0.27050573169303094, -0.05612724177451451, -0.3366234745727277, 0.27816477039994353])
        bimanual_demo.move_gripper('left', 'open')
        bimanual_demo.move_arm(
          'left',
          [0.2525493202907798, -0.20498309678016832, 0.8860128543728201, -0.026684396779588802, -0.4685281195659303, 0.25894079241234275])
        bimanual_demo.move_gripper('left', 'close')
        bimanual_demo.move_arm(
          'left',
          [0.22463184903468725,0.5640109055722806,-0.010189445663893758,-0.0029447975262430984,-0.40265756418968657,0.2046361271467092])
        bimanual_demo.move_arm(
          'left',
          [-0.4396898837134099,0.5321104980211642,0.1817969793457925,0.2751617627062951,-0.5456892082990646,-0.7223044226272037])
        bimanual_demo.move_arm(
          'left',
          [-0.5076864582614361, 0.6430632572402184, -0.1964135304533572, 0.4261104261543335, -0.31872944826848815, -1.014079334617769])
        bimanual_demo.move_arm(
          'left',
          [-0.5605443400556704, 0.6194194950284646, -0.43562651809666936, -0.6993978537986612, 0.6179626637170913, 0.0903226598271187])

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()