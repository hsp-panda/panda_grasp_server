#!/usr/bin/env python

# The point of this script is to command a set of GRASPA reachability poses to the robot
# and read the reached poses either from the robot itself or an aruco detection module
# The saved data has to adhere to the GRASPA savefile fomat.

# This code is absolute trash, to be used hopefully only once


import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Pose, PoseStamped

from panda_ros_common.msg import PandaState
from panda_ros_common.srv import PandaMove, PandaMoveRequest, PandaHome, PandaHomeRequest, PandaGetState, PandaMoveWaypoints, PandaMoveWaypointsRequest
from aruco_board_detect.msg import MarkerList

import pyquaternion as pq
import numpy as np
import sys
from threading import Lock
from xml.etree import ElementTree as ET
from xml.dom import minidom
import copy

tcp_pose = None
tcp_pose_lock = Lock()

BOARD_FRAME_NAME = "graspa_board"
ROOT_FRAME_NAME = "panda_link0"
CAMERA_FRAME_NAME = "camera_link"

tf_listener = None
tf_broadcaster = None

def get_tcp_pose(markers_msg):

    # compute pose of the TCP according to which marker is found
    # if none is found, tcp_pose = none
    # if more than one is found, choose a random one
    # look for markers 42, 43, 44

    tcp_marker_offset = 0.035

    global tcp_pose_lock
    tcp_pose_lock.acquire()

    global tcp_pose
    tcp_pose = None

    # acquire list of detected ids

    marker_ids_list = [m_id.data for m_id in markers_msg.marker_ids]

    # acquire list of marker poses, one for each id

    marker_pose_list = markers_msg.marker_poses

    # if marker 42, 43 or 44 is present:

    if any(desired_tag in marker_ids_list for desired_tag in [42, 43, 44]):

        if 43 in marker_ids_list:

            marker_pose = marker_pose_list[marker_ids_list.index(43)]
            # marker_pose = Pose()

            marker_pos = np.array([marker_pose.position.x, marker_pose.position.y, marker_pose.position.z])

            marker_quat = pq.Quaternion(marker_pose.orientation.w, marker_pose.orientation.x, marker_pose.orientation.y, marker_pose.orientation.z)

            # Translate along marker z axis to get to the TCP frame origin

            marker_z_axis = marker_quat.rotation_matrix[:,2]
            tcp_pos = marker_pos - tcp_marker_offset * marker_z_axis

            # Obtain the TCP frame by rotating the marker frame

            tcp_quat = marker_quat * pq.Quaternion(axis=[0,1,0], degrees=0)

            # Construct the pose object
            tcp_pose = PoseStamped()
            tcp_pose.header.frame_id = markers_msg.header.frame_id
            tcp_pose.header.stamp = rospy.Time.now()
            tcp_pose.pose.position.x = tcp_pos[0]
            tcp_pose.pose.position.y = tcp_pos[1]
            tcp_pose.pose.position.z = tcp_pos[2]
            tcp_pose.pose.orientation.w = tcp_quat.w
            tcp_pose.pose.orientation.x = tcp_quat.x
            tcp_pose.pose.orientation.y = tcp_quat.y
            tcp_pose.pose.orientation.z = tcp_quat.z

        elif 42 in marker_ids_list:

            marker_pose = marker_pose_list[marker_ids_list.index(42)]
            # marker_pose = PoseStamped()

            marker_pos = np.array([marker_pose.position.x, marker_pose.position.y, marker_pose.position.z])

            marker_quat = pq.Quaternion(marker_pose.orientation.w, marker_pose.orientation.x, marker_pose.orientation.y, marker_pose.orientation.z)

            # Translate along marker z axis to get to the TCP frame origin

            marker_z_axis = marker_quat.rotation_matrix[:,2]
            tcp_pos = marker_pos - tcp_marker_offset * marker_z_axis

            # Obtain the TCP frame by rotating the marker frame

            tcp_quat = marker_quat * pq.Quaternion(axis=[0,1,0], degrees=90)

            # Construct the pose object
            tcp_pose = PoseStamped()
            tcp_pose.header.frame_id = markers_msg.header.frame_id
            tcp_pose.header.stamp = rospy.Time.now()
            tcp_pose.pose.position.x = tcp_pos[0]
            tcp_pose.pose.position.y = tcp_pos[1]
            tcp_pose.pose.position.z = tcp_pos[2]
            tcp_pose.pose.orientation.w = tcp_quat.w
            tcp_pose.pose.orientation.x = tcp_quat.x
            tcp_pose.pose.orientation.y = tcp_quat.y
            tcp_pose.pose.orientation.z = tcp_quat.z

        elif 44 in marker_ids_list:

            marker_pose = marker_pose_list[marker_ids_list.index(44)]
            # marker_pose = PoseStamped()

            marker_pos = np.array([marker_pose.position.x, marker_pose.position.y, marker_pose.position.z])

            marker_quat = pq.Quaternion(marker_pose.orientation.w, marker_pose.orientation.x, marker_pose.orientation.y, marker_pose.orientation.z)

            # Translate along marker z axis to get to the TCP frame origin

            marker_z_axis = marker_quat.rotation_matrix[:,2]
            tcp_pos = marker_pos - tcp_marker_offset * marker_z_axis

            # Obtain the TCP frame by rotating the marker frame

            tcp_quat = marker_quat * pq.Quaternion(axis=[0,1,0], degrees=-90)

            # Construct the pose object
            tcp_pose = PoseStamped()
            tcp_pose.header.frame_id = markers_msg.header.frame_id
            tcp_pose.header.stamp = rospy.Time.now()
            tcp_pose.pose.position.x = tcp_pos[0]
            tcp_pose.pose.position.y = tcp_pos[1]
            tcp_pose.pose.position.z = tcp_pos[2]
            tcp_pose.pose.orientation.w = tcp_quat.w
            tcp_pose.pose.orientation.x = tcp_quat.x
            tcp_pose.pose.orientation.y = tcp_quat.y
            tcp_pose.pose.orientation.z = tcp_quat.z

    tcp_pose_copy = copy.deepcopy(tcp_pose)
    tcp_pose_lock.release()

    success = False
    rate = rospy.Rate(10)
    while not success:
        try:
            now = rospy.Time.now()
            tf_listener.waitForTransform(BOARD_FRAME_NAME, tcp_pose.header.frame_id, now, rospy.Duration(3.0))
            visual_pose = tf_listener.transformPose(BOARD_FRAME_NAME, tcp_pose)
            success = True
        except (tf.ExtrapolationException):
            rate.sleep()
            continue

    tf_broadcaster.sendTransform((visual_pose.pose.position.x,visual_pose.pose.position.y,visual_pose.pose.position.z),
        (visual_pose.pose.orientation.x, visual_pose.pose.orientation.y, visual_pose.pose.orientation.z, visual_pose.pose.orientation.w),
        rospy.Time.now(),
        "tcp_estimated",
        BOARD_FRAME_NAME)

def add_outcome(stamped_pose, parent, frame_name):

    pose_position = np.array([stamped_pose.pose.position.x, stamped_pose.pose.position.y, stamped_pose.pose.position.z])
    pose_rotation = pq.Quaternion([stamped_pose.pose.orientation.w,
                                    stamped_pose.pose.orientation.x,
                                    stamped_pose.pose.orientation.y,
                                    stamped_pose.pose.orientation.z,]).rotation_matrix

    manip_object_field = ET.SubElement(parent, 'ManipulationObject')
    manip_object_field.set('name', frame_name)

    file_field = ET.SubElement(manip_object_field, 'File')
    file_field.text = 'objects/frame.xml'

    global_pose_field = ET.SubElement(manip_object_field, 'GlobalPose')

    transform_field = ET.SubElement(global_pose_field, 'Transform')

    matrix = ET.SubElement(transform_field, 'Matrix4x4')
    row1 = ET.SubElement(matrix, 'row1')
    row2 = ET.SubElement(matrix, 'row2')
    row3 = ET.SubElement(matrix, 'row3')
    row4 = ET.SubElement(matrix, 'row4')

    for col_idx in range(4):
        if col_idx == 3:
            row1.set('c'+str(col_idx+1), str(pose_position[0] * 1000.0))
            row2.set('c'+str(col_idx+1), str(pose_position[1] * 1000.0))
            row3.set('c'+str(col_idx+1), str(pose_position[2] * 1000.0))
            row4.set('c'+str(col_idx+1), str(1))
        else:
            row1.set('c'+str(col_idx+1), str(pose_rotation[0,col_idx]))
            row2.set('c'+str(col_idx+1), str(pose_rotation[1,col_idx]))
            row3.set('c'+str(col_idx+1), str(pose_rotation[2,col_idx]))
            row4.set('c'+str(col_idx+1), str(0))

    # Not sure if I should return the parent or not




if __name__ == "__main__":

    rospy.init_node("graspa_reachability_calibration")

    tf_listener = tf.TransformListener(True, rospy.Duration(10))
    tf_broadcaster = tf.TransformBroadcaster()

    rospy.wait_for_service('panda_grasp_server/panda_move_pose')
    move_to_pose = rospy.ServiceProxy('panda_grasp_server/panda_move_wp', PandaMoveWaypoints)
    get_robot_state = rospy.ServiceProxy('panda_grasp_server/panda_get_state', PandaGetState)

    markers_sub = rospy.Subscriber('aruco_board_detector/markers_data', MarkerList, get_tcp_pose)

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
    z_grid = 0.15 * np.ones(x_grid.shape)
    positions_grid = np.dstack((x_grid, y_grid, z_grid))

    # Go home

    # rospy.wait_for_service('panda_grasp_server/panda_home')
    # try:
    #     move_home = rospy.ServiceProxy('panda_grasp_server/panda_home', PandaHome)
    #     req = PandaHomeRequest(use_joint_values=True)
    #     move_success = move_home(req).success
    #     if not move_success:
    #         print("Unable to go home. Quitting.")
    #         rospy.signal_shutdown()
    #         sys.exit(1)
    # except rospy.ServiceException as e:
    #     print("Service call failed: %s"%e)

    for set_rotation in rotation_per_set:

        # For each rotation of the eef, build a different xml

        reachability_scene_root = ET.Element('Scene')
        reachability_scene_root.set('name', "Set_Poses_{}".format(int(rotation_per_set.index(set_rotation)+1)))

        calibration_scene_root = ET.Element('Scene')
        calibration_scene_root.set('name', "Set_Poses_{}".format(int(rotation_per_set.index(set_rotation)+1)))

        # Iterate over x and y to obtain poses

        for pose_idx_x in range(positions_grid.shape[0]):
            for pose_idx_y in range(positions_grid.shape[1]):

                pose_name = "Reachable_frame{}{}".format(pose_idx_y, pose_idx_x)

                # Create pose wrt graspa board frame

                pose_graspa = PoseStamped()
                pose_graspa.header.frame_id = BOARD_FRAME_NAME
                # pose_graspa.header.stamp = rospy.Time.now()
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
                while not rospy.is_shutdown() and pose_root is None:
                    try:
                        tf_listener.waitForTransform(ROOT_FRAME_NAME, CAMERA_FRAME_NAME, rospy.Time(0), rospy.Duration(3.0))
                        pose_root = tf_listener.transformPose(ROOT_FRAME_NAME, pose_graspa)
                        break
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        rate.sleep()
                        continue

                # Go to the pose
                # In this phase, it is important that the robot moves even if the trajectory is not 100% viable

                rospy.loginfo("Moving to {}".format(pose_name))
                raw_input("Press any key to move")
                move_to_pose.wait_for_service()
                # move_success = move_to_pose(pose_root).success
                target = PandaMoveWaypointsRequest()
                target.pose_waypoints.header = pose_root.header
                target.pose_waypoints.poses = [pose_root.pose]
                move_success = move_to_pose(target).success

                # If move is successful, wait a bit and read eef pose from the grasp server

                if move_success:

                    rospy.sleep(rospy.Duration(2))

                    rospy.loginfo("Retrieving TCP pose from the robot...")

                    eef_pose_direct_kin = get_robot_state().robot_state.eef_state

                    # Obtain the pose wrt the GRASPA board ref frame

                    reachability_pose = PoseStamped()
                    now = rospy.Time.now()
                    success = False
                    while not success and not rospy.is_shutdown():
                        try:
                            tf_listener.waitForTransform(ROOT_FRAME_NAME, CAMERA_FRAME_NAME, eef_pose_direct_kin.header.stamp , rospy.Duration(3.0))
                            reachability_pose = tf_listener.transformPose(BOARD_FRAME_NAME, eef_pose_direct_kin)
                            success = True
                        except (tf.ExtrapolationException):
                            rate.sleep()
                            continue

                    # Read marker list and estimate eef pose from vision

                    rospy.loginfo("Retrieving TCP pose from the vision system...")

                    success = False
                    while not success and not rospy.is_shutdown():
                        tcp_pose_lock.acquire()
                        if tcp_pose:
                            success = True
                        else:
                            tcp_pose_lock.release()
                            rate.sleep()

                    # tcp_pose is supposed a PoseStamped

                    tf_listener.waitForTransform(BOARD_FRAME_NAME, CAMERA_FRAME_NAME, tcp_pose.header.stamp, rospy.Duration(3.0))
                    visual_pose = tf_listener.transformPose(BOARD_FRAME_NAME, tcp_pose)
                    tcp_pose_lock.release()

                    # Save pose in xml tree

                    add_outcome(reachability_pose, reachability_scene_root, pose_name)
                    add_outcome(visual_pose, calibration_scene_root, pose_name)

        # Save xml

        domstring = minidom.parseString(ET.tostring(reachability_scene_root))
        filename = "reached_poses{}".format(format(int(rotation_per_set.index(set_rotation))))

        with open(filename, "w") as handle:
            handle.write(domstring.toprettyxml())

        domstring = minidom.parseString(ET.tostring(calibration_scene_root))
        filename = "cam_calibration_test_output{}".format(format(int(rotation_per_set.index(set_rotation))))

        with open(filename, "w") as handle:
            handle.write(domstring.toprettyxml())


