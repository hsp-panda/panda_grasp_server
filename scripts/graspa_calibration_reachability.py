#!/usr/bin/env python

# The point of this script is to command a set of GRASPA reachability poses to the robot
# and read the reached poses either from the robot itself or an aruco detection module
# The saved data has to adhere to the GRASPA savefile fomat.


import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Pose, PoseStamped

from panda_ros_common.msg import PandaState
from panda_ros_common.srv import PandaMove, PandaHome, PandaGetState

import pyquaternion as pq
import numpy as np
import sys

tcp_pose = None

BOARD_FRAME_NAME = "graspa_board"
ROOT_FRAME_NAME = "panda_link0"
CAMERA_FRAME_NAME = "camera_link"

def get_tcp_pose(markers):

    # compute pose of the TCP according to which marker is found
    # if none is found, tcp_pose = none

    raise NotImplementedError

if __name__ == "__main__":

    rospy.init_node("graspa_reachability_calibration")

    tf_listener = tf.TransformListener(True, rospy.Duration(10))

    rospy.wait_for_service('panda_grasp_server/panda_move_pose')
    move_to_pose = rospy.ServiceProxy('panda_grasp_server/panda_move_pose', PandaMove)

    # Define 3 sets of poses that will be used for both reachability and calibration
    # These target poses are defined wrt the board reference frame

    # z facing left

    rotation_set_1 = pq.Quaternion(0.5, -0.5, -0.5, 0.5) # wxyz

    # z facing down

    rotation_set_2 = pq.Quaternion(0, 0.7071068, 0.7071068, 0)

    # z facing right

    rotation_set_3 = pq.Quaternion(0.5, 0.5, 0.5, 0.5)

    rotation_per_set = [rotation_set_1, rotation_set_2, rotation_set_3]

    # The tcp positions are the same for all the sets
    # Create a 4x4 grid of poses with the same order as in GRASPA

    x_space = -np.linspace(0, 0.544, 4)
    y_space = np.linspace(0, 0.37, 4)
    x_grid, y_grid = np.meshgrid(x_space, y_space)
    z_grid = 0.15 * np.ones(x_space.shape)
    positions_grid = np.dstack((x_grid, y_grid, z_grid))

    # Go home

    rospy.wait_for_service('panda_grasp_server/panda_home')
    try:
        move_home = rospy.ServiceProxy('panda_grasp_server/panda_home', PandaHome)
        req = PandaHomeRequest(use_joint_values=True)
        move_success = move_home(req).success
        if not move_success:
            print("Unable to go home. Quitting.")
            rospy.signal_shutdown()
            sys.exit(1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    for set_rotation in rotation_per_set:

        # Iterate over x and y to obtain poses

        for pose_idx_x in range(positions_grid.shape(0)):
            for pose_idx_y in range(positions_grid.shape(1)):

                pose_name = "Reachable_frame{}{}".format(pose_idx_y, pose_idx_x)

                # Create pose wrt graspa board frame

                pose_graspa = PoseStamped()
                pose_graspa.header.frame_id = BOARD_FRAME_NAME
                pose_graspa.header.stamp = rospy.Time.now()
                pose_graspa.pose.position.x = positions_grid[pose_idx_x, pose_idx_y][0]
                pose_graspa.pose.position.y = positions_grid[pose_idx_x, pose_idx_y][1]
                pose_graspa.pose.position.z = positions_grid[pose_idx_x, pose_idx_y][2]

                pose_graspa.pose.orientation.x = set_rotation.x
                pose_graspa.pose.orientation.y = set_rotation.y
                pose_graspa.pose.orientation.z = set_rotation.z
                pose_graspa.pose.orientation.w = set_rotation.w

                # Transform pose into robot root, when available

                pose_root = None
                rate = rospy.Rate(10)
                while not rospy.is_shutdown():
                    try:
                        pose_root = tf_listener.transformPose(ROOT_FRAME_NAME, pose_graspa)
                        break
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        rate.sleep()
                        continue

                # Go to the pose

                rospy.INFO("Moving to {}".format(pose_name))
                raw_input("Press any key to move")
                move_to_pose.wait_for_service()
                move_success = move_to_pose(pose_root).success

                # If move is successful, wait a bit and read eef pose from the grasp server

                # Read marker list and estimate eef pose from vision

                # Save pose in xml tree

        # Save xml

    # gubai

