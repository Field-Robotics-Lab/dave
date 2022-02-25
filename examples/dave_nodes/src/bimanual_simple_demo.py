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


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        # Initialize `moveit_commander` and node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)


        # Instantiate a `RobotCommander` object. Provides robot's
        # kinematic model and current joint states
        robot = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface` object. Provides a remote interface
        # for getting, setting, and updating robot's state
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).
        move_group = moveit_commander.MoveGroupCommander("arm_r")
        move_group_r = moveit_commander.MoveGroupCommander("hand_r")
        move_group_l = moveit_commander.MoveGroupCommander("hand_l")
        move_group_arm_r = moveit_commander.MoveGroupCommander("arm_r")
        move_group_arm_l = moveit_commander.MoveGroupCommander("arm_l")


        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # Subscribe to an object location via model_state 
        # TODO use to generate new arm pose
        self.object_1_sub = rospy.Subscriber('gazebo/model_states', ModelStates, self.get_state_CB)

        # Get reference frame name for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # Print the name of the end-effector link for a group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # List all groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Print the entire state of the robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.move_group_r = move_group_r
        self.move_group_l = move_group_l
        self.move_group_arm_r = move_group_arm_r
        self.move_group_arm_l = move_group_arm_l

        # Logging starting pose. There are better ways to do this.
        self.arm_l_start = move_group_arm_l.get_current_pose().pose
        self.arm_r_start = move_group_arm_r.get_current_pose().pose

        self.rate = rospy.Rate(1)

    def get_state_CB(self, _data):
        self.get_state_msg = _data

    def go_to_joint_state(self):
        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  # 1/6 of a turn
        joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def open_gripper_r(self):
        print()
        print("==========Opening right gripper...==========")

        joint_goal = self.move_group_r.get_current_joint_values()
        joint_goal[0] = 1.0

        self.move_group_r.go(joint_goal, wait=True)
        self.move_group_r.stop()

    def close_gripper_r(self):
        print()
        print("==========Closing right gripper...==========")

        joint_goal = self.move_group_r.get_current_joint_values()
        joint_goal[0] = 0.0

        self.move_group_r.go(joint_goal, wait=True)
        self.move_group_r.stop()

    def open_gripper_l(self):
        print()
        print("==========Opening left gripper...==========")

        joint_goal = self.move_group_l.get_current_joint_values()
        joint_goal[0] = 1.0

        self.move_group_l.go(joint_goal, wait=True)
        self.move_group_l.stop()

    def close_gripper_l(self):
        print()
        print("==========Closing left gripper...==========")

        joint_goal = self.move_group_l.get_current_joint_values()
        joint_goal[0] = 0.0

        self.move_group_l.go(joint_goal, wait=True)
        self.move_group_l.stop()

    def go_to_pose_goal_l(self, home_var):
        print()
        print("==========Sending left ee to goal...==========")
        current_pose = self.move_group.get_current_pose().pose

        pose_goal = geometry_msgs.msg.Pose()

        if home_var == False:
            pose_goal.orientation.w = 0.928
            pose_goal.orientation.x = 0.0198
            pose_goal.orientation.y = 0.37158
            pose_goal.orientation.z = 0.005
            pose_goal.position.x = 2.214
            pose_goal.position.y = 0.2565
            pose_goal.position.z = -1.4943
        else:
            pose_goal = self.arm_l_start

        self.move_group_arm_l.set_pose_target(pose_goal)

        # Call the planner to compute the plan and execute it.
        plan = self.move_group_arm_l.go(wait=True)
        # Call `stop()` to ensure there is no residual movement
        move_group.stop()
        # Clear your targets after planning with poses.
        move_group.clear_pose_targets()

        current_pose = self.move_group_arm_l.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def go_to_pose_goal_r(self, home_var, target_pose):
        print()
        print("==========Sending right ee to goal...==========")
        current_pose = self.move_group_arm_r.get_current_pose().pose

        pose_goal = geometry_msgs.msg.Pose()

        if home_var == False:
            pose_goal = target_pose
            pose_goal.orientation.w = 0.928
            pose_goal.orientation.x = 0.0198
            pose_goal.orientation.y = 0.37158
            pose_goal.orientation.z = 0.005
            pose_goal.position.x = 2.214
            pose_goal.position.y = -0.2565
            pose_goal.position.z = -1.4943
        else:
            pose_goal = self.arm_r_start

        self.move_group_arm_r.set_pose_target(pose_goal)

        # Call the planner to compute the plan and execute it.
        plan = self.move_group_arm_r.go(wait=True)
        # Call `stop()` to ensure there is no residual movement
        self.move_group_arm_r.stop()
        # Clear targets after planning with poses.
        self.move_group_arm_r.clear_pose_targets()

        current_pose = self.move_group_arm_r.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
       
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
            waypoints, 0.01, 0.0  
        )  

        
        return plan, fraction


    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        move_group = self.move_group

        move_group.execute(plan, wait=True)


    def pose_to_mat(self, pose):
        base_trans = tf.transformations.translation_matrix((pose.position.x,
                                                            pose.position.y,
                                                            pose.position.z))
        base_rot = tf.transformations.quaternion_matrix((pose.orientation.x,
                                                         pose.orientation.y,
                                                         pose.orientation.z,
                                                         pose.orientation.w))

        trans_mat = np.matmul(base_trans, base_rot)
        return trans_mat

    def get_target_pose(self, object_pose, robot_pose, arm_pose):
        object_mat = self.pose_to_mat(object_pose)
        robot_mat = self.pose_to_mat(robot_pose)
        arm_mat = self.pose_to_mat(arm_pose)

        # Get the arm wrt world
        a_w_T = np.matmul(robot_mat, arm_mat)
        # Get object wrt to arm
        o_a_T = np.matmul(np.linalg.inv(a_w_T), object_mat)
        # Get object wrt robot
        o_r_T = np.matmul(arm_mat, o_a_T)
        
        #  Returns as x, y, z, w
        object_quat = tf.transformations.quaternion_from_matrix(o_r_T)

        new_target_pose = geometry_msgs.msg.Pose()
        new_target_pose.position.x = o_r_T[0, 3]
        new_target_pose.position.y = o_r_T[1, 3]
        new_target_pose.position.z = o_r_T[2, 3]
        new_target_pose.orientation.x = object_quat[0]
        new_target_pose.orientation.y = object_quat[1]
        new_target_pose.orientation.z = object_quat[2]
        new_target_pose.orientation.w = object_quat[3]

        return new_target_pose

    def run_node(self):
        counter = 0
        while not rospy.is_shutdown():
            self.rate.sleep()

            target_pose = self.get_target_pose(self.get_state_msg.pose[3],
                                               self.get_state_msg.pose[-1],
                                               self.move_group_arm_l.get_current_pose().pose)

            if counter == 2:
                try:
                    print()
                    print("Target Pose")
                    print(target_pose)
                    self.go_to_pose_goal_r(False, target_pose)
                    # Open and close each gripper
                    self.open_gripper_r()
                    self.close_gripper_r()
                    self.open_gripper_l()
                    self.close_gripper_l()
                    # Go to first pose goal, set home pose as False
                    self.go_to_pose_goal_r(False)
                    self.go_to_pose_goal_l(False)
                    # Set home pose as true to return to start
                    self.go_to_pose_goal_r(True)
                    self.go_to_pose_goal_l(True)
                except:
                    print("Unable to assume pose")
                break
            counter = counter + 1
            self.rate.sleep()


def main():
    try:
        print("Setting up moveit commander")
        tutorial = MoveGroupPythonInterfaceTutorial()

        while(True):
            input("Press 'Enter' to try to pick up object")
            tutorial.run_node()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()