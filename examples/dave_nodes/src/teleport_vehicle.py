#!/usr/bin/env python3
'''
Teleport the vehicle to different locations in any world

'''

# System imports
import sys
import signal
import argparse
import yaml

# ROS/Gazebo imports
import rospy
import tf

# For GetModelState Service
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist

# Signal handler for graceful exit
def signal_handler(sig, frame):
        rospy.loginfo("Shutting down teleport_vehicle!")
        sys.exit(0)

class Poses:
    def __init__(self, name:str, robot_pose:list, camera_pose:list):
        self.name = name
        self.robot_pose = robot_pose
        self.camera_pose = camera_pose


def get_pose(pose: list):
    ''' Convert a list of x,y,z,roll,pitch,yaw into geometry_msgs/Pose '''
    pose_msg = Pose()
    print(f"Pose:{len(pose)}")
    if len(pose) < 6:
        return None
    pose_msg.position.x = pose[0]
    pose_msg.position.y = pose[1]
    pose_msg.position.z = pose[2]
    quaternion = tf.transformations.quaternion_from_euler(pose[3],\
                 pose[4], pose[5])
    pose_msg.orientation.x = quaternion[0]
    pose_msg.orientation.y = quaternion[1]
    pose_msg.orientation.z = quaternion[2]
    pose_msg.orientation.w = quaternion[3]
    return pose_msg


def teleport(places: dict):
    set_model_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    while not rospy.is_shutdown():
        print('Enter NAME PLACE:')
        user_input = input()
        entires = user_input.strip().split(' ')
        if len(entires) < 2:
          print("Invalid input. See usage")
          continue
        uuv_name = str(entires[0])
        place_name = str(entires[1])
        if place_name not in places:
            rospy.logerr("Invalid PLACE. See configured places.")
            continue
        place = places[place_name]
        rospy.wait_for_service("/gazebo/set_model_state")
        pose_msg = get_pose(place.robot_pose)
        if pose_msg is None:
            rospy.logerr("Unable to generate Pose msg for configured pose. Skipping...")
            continue
        set_model_srv(ModelState(uuv_name, pose_msg, Twist(), "world"))
        rospy.loginfo(f"Teleported UUV [{uuv_name}] to {place.name}:{place.robot_pose}")


def main(argv=sys.argv):
    parser = argparse.ArgumentParser(
        prog="teleport_vehicle",
        description="A script to teleport UUVs within underwater environments")
    parser.add_argument("-c", "--config", type=str, required=True,
                        help="Path to the config.yaml file")
    args, _ = parser.parse_known_args(argv[1:])
    config_yaml = None
    print(f"Config path: {args.config}")
    with open(args.config, "r") as f:
        config_yaml = yaml.safe_load(f)

    # Map place name to poses
    places = {}
    for place, poses in config_yaml['places'].items():
        places[place] = Poses(place, poses['robot_pose'], poses['camera_pose'])

    rospy.init_node('teleport_vehicle')
    signal.signal(signal.SIGINT, signal_handler)

    print('#####################')
    print('Usage:\n    To teleport a UUV with name NAME to place PLACE, enter NAME PLACE: ')
    print(f'    Configured places: {list(places.keys())}')
    print('#####################')

    try:
        teleport(places)
    except rospy.ROSInterruptException:
        sys.exit(0)


if __name__ == '__main__':
    main(sys.argv)
