#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This script takes care of performing the reachability/calibration steps
# of the GRASPA benchmark. It involves the presence of the robot itself,
# the hand-mounted camera, the setup-mounted camera, the marker cube, and
# the GRASPA marker board.

# Make sure the hand-mounted camera can see the board in the robot homing
# configuration, and that the setup-mounted camera can see both the marker cube
# (while grasped) and the board for the whole procedure.

# Any transform or matrix is written with the convention
# a_T_b -> b expressed in the a reference frame

import rospy
import tf2_ros as tf2
import tf.transformations
import tf.transformations as t
from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped
from std_msgs.msg import Header

from panda_ros_common.msg import PandaState
from panda_ros_common.srv import PandaMove, PandaMoveRequest
from panda_ros_common.srv import PandaHome, PandaHomeRequest
from panda_ros_common.srv import PandaGetState
from panda_ros_common.srv import PandaMoveWaypoints, PandaMoveWaypointsRequest
from panda_ros_common.srv import PandaGripperCommand, PandaGripperCommandRequest
from topic_tools.srv import MuxSelect
from std_srvs.srv import Trigger, TriggerRequest
from aruco_board_detect.msg import MarkerList

import pyquaternion as pq
import numpy as np
import sys
from threading import Lock
from xml.etree import ElementTree as ET
from xml.dom import minidom
import copy

# Globals
tcp_pose = None
tcp_pose_lock = Lock()

BOARD_FRAME_NAME = "graspa_board"
ROOT_FRAME_NAME = "panda_link0"
MUXED_CAMERA_NAME = "camera"
HAND_CAMERA_NAME = "hand_camera"
HAND_CAMERA_LINK_NAME = HAND_CAMERA_NAME + "_link"
HAND_CAMERA_FRAME_NAME = HAND_CAMERA_LINK_NAME
SETUP_CAMERA_NAME = "setup_camera"
SETUP_CAMERA_LINK_NAME = SETUP_CAMERA_NAME + "_link"
SETUP_CAMERA_FRAME_NAME = SETUP_CAMERA_LINK_NAME
TCP_MARKERS_LIST = [42, 43, 44]
TCP_MARKER_GRIPPER_SIDE = 41
TCP_MARKER_OFFSET = [0.0, 0.0, 0.035]

tf_listener = None
tf_broadcaster = None

def get_tcp_pose(markers_msg):

    # compute pose of the TCP according to which marker is found
    # if none is found, tcp_pose = none
    # if more than one is found, choose a random one
    # look for markers 41, 42, 43, 44

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

    if any(desired_tag in marker_ids_list for desired_tag in [42, 43, 44, TCP_MARKER_GRIPPER_SIDE]):

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

        elif TCP_MARKER_GRIPPER_SIDE in marker_ids_list:

            marker_pose = marker_pose_list[marker_ids_list.index(TCP_MARKER_GRIPPER_SIDE)]
            # marker_pose = PoseStamped()

            marker_pos = np.array([marker_pose.position.x, marker_pose.position.y, marker_pose.position.z])

            marker_quat = pq.Quaternion(marker_pose.orientation.w, marker_pose.orientation.x, marker_pose.orientation.y, marker_pose.orientation.z)

            # Translate along marker z axis to get to the TCP frame origin

            marker_z_axis = marker_quat.rotation_matrix[:,2]
            tcp_pos = marker_pos - tcp_marker_offset * marker_z_axis

            # Obtain the TCP frame by rotating the marker frame

            tcp_quat = marker_quat * pq.Quaternion(axis=[0,1,0], degrees=180)

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

def switch_camera_input(new_camera_name):

    # Use services to act on the camera multiplexers
    rospy.loginfo("Switching current camera input to {}".format(new_camera_name))
    rospy.wait_for_service("/camera_info_mux/select")
    camera_info_mux = rospy.ServiceProxy("/camera_info_mux/select", MuxSelect)
    rospy.wait_for_service("/camera_image_mux/select")
    camera_image_mux = rospy.ServiceProxy("/camera_image_mux/select", MuxSelect)

    # Switch camera info
    req = MuxSelect()
    req = "/" + new_camera_name + "/color/camera_info"
    res = camera_info_mux(req)

    # Switch camera image
    req = MuxSelect()
    req = "/" + new_camera_name + "/color/image_raw"
    res = camera_image_mux(req)

    rospy.loginfo("Switched camera input")

    return

def position_difference(pos_1, pos_2):

    return np.linalg.norm(np.array(pos_1) - np.array(pos_2))

def rotation_difference_quat(quat_1, quat_2):

    acc = 0
    for idx in range(4):
        acc += quat_1[idx] * quat_2[idx]

    theta = 2 * np.arccos(np.absolute(acc))
    return theta

def stop_robot():

    # Use panda_grasp_server's stop service to
    # stop both robot and gripper

    rospy.wait_for_service("/panda_grasp_server/panda_stop")
    stop = rospy.ServiceProxy("/panda_grasp_server/panda_stop", Trigger)
    stop(TriggerRequest())

def grasp_marker_cube(move_finger_proxy):

    # Since auto grasping from the table is not really precise enough,
    # we ask the user to manually position the cube in the robot hand

    marker_cube_grasped = False

    while not marker_cube_grasped:

        stop_robot()

        req = PandaGripperCommandRequest(width=0.08)
        rospy.wait_for_service('panda_grasp_server/panda_gripper_cmd')
        res = move_finger_proxy(req)

        rospy.loginfo("Insert the marker cube between the robot fingers, with marker 41 on the hand side. Check rViz and align tcp_estimated with panda_tcp.. Press any key to continue.")
        raw_input()

        req = PandaGripperCommandRequest(width=0.06, close_grasp=True)
        rospy.wait_for_service('panda_grasp_server/panda_gripper_cmd')
        res = move_finger_proxy(req)

        if res.success:
            # Make sure the cube is oriented correctly

            # Get transformation from root to tcp_estimated
            try:
                tf_listener.waitForTransform(ROOT_FRAME_NAME, "tcp_estimated", rospy.Time(0), rospy.Duration(3.0))
                root_T_tcp_estimated = tf_listener.lookupTransform(ROOT_FRAME_NAME, "tcp_estimated", tf_listener.getLatestCommonTime(ROOT_FRAME_NAME, "tcp_estimated"))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Could not retrieve tf between {} and {}".format(ROOT_FRAME_NAME, "tcp_estimated"))
                pass

            # Get transformation from root to tcp_estimated
            try:
                tf_listener.waitForTransform(ROOT_FRAME_NAME, "panda_tcp", rospy.Time(0), rospy.Duration(3.0))
                root_T_tcp = tf_listener.lookupTransform(ROOT_FRAME_NAME, "panda_tcp", tf_listener.getLatestCommonTime(ROOT_FRAME_NAME, "panda_tcp"))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Could not retrieve tf between {} and {}".format(ROOT_FRAME_NAME, "panda_tcp"))
                pass

            if position_difference(root_T_tcp_estimated[0], root_T_tcp[0]) < 0.1 and rotation_difference_quat(root_T_tcp_estimated[1], root_T_tcp[1]) < 0.5 :
                rospy.loginfo("Marker set correctly")
                marker_cube_grasped = True
                pass
            else:
                rospy.loginfo("Marker set incorrectly. Check rViz and align tcp_estimated with panda_tcp.")
                pass

        else:
            rospy.logerr("Could not grasp the marker cube.")
            pass

    return True

def shutdown_handle():

    # Switch camera streams
    switch_camera_input(HAND_CAMERA_NAME)
    rospy.loginfo("Switched back camera")


if __name__ == "__main__":

    rospy.init_node("graspa_reachability_calibration")
    rospy.on_shutdown(shutdown_handle)

    # Set up TF listener and broadcaster
    tf_listener = tf.TransformListener(True, rospy.Duration(10))
    tf_broadcaster = tf.TransformBroadcaster()
    tf_static_broadcaster = tf2.StaticTransformBroadcaster()

    # Set up services to move the robot and retrieve status
    rospy.loginfo("Setting up service proxies to panda_grasp_server")
    rospy.wait_for_service('panda_grasp_server/panda_move_wp') #TODO is this the right one?
    move_to_pose = rospy.ServiceProxy('panda_grasp_server/panda_move_wp', PandaMoveWaypoints)
    rospy.wait_for_service('panda_grasp_server/panda_get_state')
    get_robot_state = rospy.ServiceProxy('panda_grasp_server/panda_get_state', PandaGetState)
    rospy.wait_for_service('panda_grasp_server/panda_home')
    move_home = rospy.ServiceProxy('panda_grasp_server/panda_home', PandaHome)
    rospy.wait_for_service('panda_grasp_server/panda_gripper_cmd')
    finger_move = rospy.ServiceProxy('panda_grasp_server/panda_gripper_cmd', PandaGripperCommand)
    rospy.loginfo("Service proxies are up")

    # Subscribe to detected markers data topic
    markers_sub = rospy.Subscriber('aruco_board_detector/markers_data', MarkerList, get_tcp_pose)

    # Define 3 sets of poses that will be used for both reachability and calibration
    # Poses in each set share the same orientation
    # Sets share the same positions, orientation depending upon which set
    # These target poses are defined wrt the graspa_board reference frame

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

    # Home the robot after user input
    rospy.loginfo("The robot arm will now home. Press any key to proceed.")
    raw_input()
    req = PandaHomeRequest(use_joint_values=True, home_gripper=True)
    rospy.wait_for_service('panda_grasp_server/panda_home')
    move_home(req)
    rospy.loginfo("Homing complete.")

    # Acquire initial pose (joint space).
    # This will be useful in trajectory planning
    initial_pose_joints = get_robot_state().robot_state.joints_state
    initial_pose_pose = get_robot_state().robot_state.eef_state

    rospy.loginfo("Detecting GRASPA board.")

    # We now need to compute the setup camera pose, i.e. root_T_setup_cam
    # We will compute root_T_board and setup_cam_T_board and use these
    # Compute root_T_board
    try:
        tf_listener.waitForTransform(ROOT_FRAME_NAME, BOARD_FRAME_NAME, rospy.Time(0), rospy.Duration(3.0))
        root_T_board = tf_listener.lookupTransform(ROOT_FRAME_NAME, BOARD_FRAME_NAME, tf_listener.getLatestCommonTime(ROOT_FRAME_NAME, BOARD_FRAME_NAME))
        root_T_board_mat = tf_listener.fromTranslationRotation(root_T_board[0], root_T_board[1])
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Could not retrieve tf between {} and {}".format(ROOT_FRAME_NAME, BOARD_FRAME_NAME))
        sys.exit("Quitting...")

    # Move to the center of the board
    rospy.loginfo("The robot arm will move to the center of the board. Press any key to proceed.")
    raw_input()

    move_to_pose.wait_for_service()
    target = PandaMoveWaypointsRequest()
    target.pose_waypoints.header = Header(stamp = rospy.Time.now(), frame_id = ROOT_FRAME_NAME)
    target_pose = copy.deepcopy(initial_pose_pose.pose)
    target_pose.position.x = root_T_board[0][0] + 0.18
    target_pose.position.y = root_T_board[0][1] + 0.256
    target_pose.position.z = root_T_board[0][2] + 0.2
    target.pose_waypoints.poses = [target_pose]
    move_to_pose(target)

    rospy.loginfo("Switching camera stream. Make sure the setup camera can see both the board and gripper. Press any key to proceed.")
    raw_input()

    # Switch camera streams
    switch_camera_input(SETUP_CAMERA_NAME)

    # Get transformation from setup_camera_link to graspa_board
    try:
        tf_listener.waitForTransform(SETUP_CAMERA_FRAME_NAME, BOARD_FRAME_NAME, rospy.Time(0), rospy.Duration(3.0))
        setup_cam_T_board = tf_listener.lookupTransform(SETUP_CAMERA_FRAME_NAME, BOARD_FRAME_NAME, tf_listener.getLatestCommonTime(SETUP_CAMERA_FRAME_NAME, BOARD_FRAME_NAME))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Could not retrieve tf between {} and {}".format(SETUP_CAMERA_FRAME_NAME, BOARD_FRAME_NAME))
        sys.exit("Quitting...")

    # Compute root_T_setup_cam and send it statically
    setup_cam_T_board_mat = tf_listener.fromTranslationRotation(setup_cam_T_board[0], setup_cam_T_board[1])
    root_T_setup_cam_mat = t.concatenate_matrices(root_T_board_mat, t.inverse_matrix(setup_cam_T_board_mat))

    root_T_setup_cam_stamped = TransformStamped()
    root_T_setup_cam_stamped.header = Header(0, rospy.Time.now(), ROOT_FRAME_NAME)
    root_T_setup_cam_stamped.child_frame_id = SETUP_CAMERA_FRAME_NAME
    root_T_setup_cam_stamped.transform.translation.x = t.translation_from_matrix(root_T_setup_cam_mat)[0]
    root_T_setup_cam_stamped.transform.translation.y = t.translation_from_matrix(root_T_setup_cam_mat)[1]
    root_T_setup_cam_stamped.transform.translation.z = t.translation_from_matrix(root_T_setup_cam_mat)[2]
    root_T_setup_cam_stamped.transform.rotation.x = t.quaternion_from_matrix(root_T_setup_cam_mat)[0]
    root_T_setup_cam_stamped.transform.rotation.y = t.quaternion_from_matrix(root_T_setup_cam_mat)[1]
    root_T_setup_cam_stamped.transform.rotation.z = t.quaternion_from_matrix(root_T_setup_cam_mat)[2]
    root_T_setup_cam_stamped.transform.rotation.w = t.quaternion_from_matrix(root_T_setup_cam_mat)[3]

    tf_static_broadcaster.sendTransform(root_T_setup_cam_stamped)

    # Get the marker cube and re-home
    rospy.loginfo("Picking up marker cube. Press any key to proceed.")
    raw_input()
    if not grasp_marker_cube(finger_move):
        rospy.logerr("Could not pick marker cube! Quitting.")
        sys.exit()

    rospy.loginfo("Setup ready for reachability/calibration motion routine. Press any key to proceed.")
    raw_input()

    req = PandaHomeRequest(use_joint_values=True, home_gripper=False)
    rospy.wait_for_service('panda_grasp_server/panda_home')
    move_home(req)

    for set_rotation in rotation_per_set:

        # For each rotation of the eef, build a different xml tree
        # One tree for reachability poses
        # One tree for calib poses
        reachability_scene_root = ET.Element('Scene')
        reachability_scene_root.set('name', "Set_Poses_{}".format(int(rotation_per_set.index(set_rotation)+1)))

        calibration_scene_root = ET.Element('Scene')
        calibration_scene_root.set('name', "Set_Poses_{}".format(int(rotation_per_set.index(set_rotation)+1)))

        # Iterate over x and y to obtain poses
        for pose_idx_x in range(positions_grid.shape[0]):
            for pose_idx_y in range(positions_grid.shape[1]):

                pose_name = "Reachable_frame{}{}".format(pose_idx_x, pose_idx_y)

                # Create posestamped wrt graspa board frame
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

                # panda_grasp_server accepts poses in the robot ref frame,
                # so graspa poses will have to be translated

                pose_root = None
                rate = rospy.Rate(10)
                while not rospy.is_shutdown() and pose_root is None:
                    try:
                        tf_listener.waitForTransform(ROOT_FRAME_NAME, BOARD_FRAME_NAME, rospy.Time(0), rospy.Duration(3.0))
                        pose_root = tf_listener.transformPose(ROOT_FRAME_NAME, pose_graspa)
                        break
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        rate.sleep()
                        continue

                # Go to the pose
                # In this phase, it is important that the robot moves even if the trajectory is not 100% viable

                rospy.loginfo("Moving to {}".format(pose_name))
                # raw_input("Press any key to move")
                # rospy.sleep(1)
                move_to_pose.wait_for_service()
                # move_success = move_to_pose(pose_root).success
                target = PandaMoveWaypointsRequest()
                target.pose_waypoints.header = pose_root.header
                if pose_idx_x==0 and pose_idx_y > 0:
                    target.pose_waypoints.poses = [initial_pose_pose.pose, pose_root.pose]
                else:
                    target.pose_waypoints.poses = [pose_root.pose]
                move_success = move_to_pose(target).success

                # If move is successful, wait a bit and read eef pose from the grasp server

                if move_success:

                    rospy.sleep(rospy.Duration(0.5))

                    rospy.loginfo("Retrieving TCP pose from the robot...")

                    eef_pose_direct_kin = get_robot_state().robot_state.eef_state

                    # Obtain the pose wrt the GRASPA board ref frame

                    reachability_pose = PoseStamped()
                    now = rospy.Time.now()
                    success = False
                    while not success and not rospy.is_shutdown():
                        try:
                            tf_listener.waitForTransform(ROOT_FRAME_NAME, BOARD_FRAME_NAME, eef_pose_direct_kin.header.stamp , rospy.Duration(3.0))
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

                    tf_listener.waitForTransform(BOARD_FRAME_NAME, SETUP_CAMERA_FRAME_NAME, tcp_pose.header.stamp, rospy.Duration(3.0))
                    visual_pose = tf_listener.transformPose(BOARD_FRAME_NAME, tcp_pose)
                    tcp_pose_lock.release()

                    # Save pose in xml tree

                    add_outcome(reachability_pose, reachability_scene_root, pose_name)
                    add_outcome(visual_pose, calibration_scene_root, pose_name)

        # Move the robot back to initial state

        req = PandaHomeRequest(use_joint_values=True, home_gripper=False)
        rospy.wait_for_service('panda_grasp_server/panda_home')
        move_home(req)

        # Save xml
        domstring = minidom.parseString(ET.tostring(reachability_scene_root))
        filename = "reached_poses{}.xml".format(format(int(rotation_per_set.index(set_rotation))))

        with open(filename, "w") as handle:
            handle.write(domstring.toprettyxml())

        domstring = minidom.parseString(ET.tostring(calibration_scene_root))
        filename = "cam_calibration_test_output{}.xml".format(format(int(rotation_per_set.index(set_rotation))))

        with open(filename, "w") as handle:
            handle.write(domstring.toprettyxml())


