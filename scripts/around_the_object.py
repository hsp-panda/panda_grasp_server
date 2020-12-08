#!/usr/bin/env python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import tf
import rospy
from tf.transformations import quaternion_from_matrix, euler_from_quaternion
from geometry_msgs.msg import Pose, PoseStamped
from panda_ros_common.msg import PandaState
from panda_ros_common.srv import PandaMove, PandaHome, PandaHomeRequest, PandaGetState
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pyquaternion import Quaternion

def normalize_rows(mat_to_normalize):

    return mat_to_normalize / np.linalg.norm(mat_to_normalize, axis=1, keepdims=True)

camera_image = Image()

def camera_data_callback(rgb):

    global camera_image
    camera_image = rgb

def get_trajectory(obj_center, radius=0.4,  min_theta=np.pi / 2 + np.pi/6, max_theta= np.pi, min_phi=0, max_phi=2*np.pi, theta_steps=10, phi_steps=5):
    # Organize the parameter grid
    theta = np.linspace(min_theta, max_theta, theta_steps, endpoint=False)
    phi = np.linspace(min_phi, max_phi, phi_steps, endpoint=False)
    
    # Organize the parameter grid
    theta_space = np.empty((0,))
    phi_space = np.empty((0,))
    for idx in range(phi.size):
        theta_space = np.append(theta_space, theta)
        phi_space = np.append(phi_space, phi[idx]*np.ones_like(theta))

    theta_space = np.reshape(theta_space, (1, theta_space.size))
    phi_space = np.reshape(phi_space, (1, phi_space.size))

    # Obtain pose center points
    x = radius * np.cos(theta_space) * np.cos(phi_space) + obj_center[0]
    y = radius * np.cos(theta_space) * np.sin(phi_space) + obj_center[1]
    z = radius * np.sin(theta_space) + obj_center[2]

    points = np.hstack((np.transpose(x), np.transpose(y), np.transpose(z)))
    return points, theta_space, phi_space

def get_orientations(obj_center, points, theta, phi):
    # find the x axis by looking at center of sphere
    # find the y axis by finding the derivative of x and y wrt theta
    # find the z axis by cross product

    z_ax = np.reshape(obj_center, (1, 3)) - points
    z_ax = normalize_rows(z_ax)

    y_ax_x = -np.sin(theta)
    y_ax_y = np.cos(theta)
    y_ax_z = np.zeros_like(y_ax_x)

    y_ax = np.hstack((np.transpose(y_ax_x), np.transpose(y_ax_y), np.transpose(y_ax_z)))
    y_ax = normalize_rows(y_ax)

    x_ax = np.cross(y_ax, z_ax)
    x_ax = normalize_rows(x_ax)
    return x_ax, y_ax, z_ax

def set_pose(br, position, orientation, idx):
    # position = points[i,:]
    # orientation = [x_ax[i,:], y_ax[i, :], z_ax[i, :]]

    x_ax_, y_ax_, z_ax_ = orientation
    rot_matrix = np.identity(4)
    rot_matrix[:3, :3] = np.transpose(np.stack((
        x_ax_,
        y_ax_,
        z_ax_
    )))
    

    pose_quat = quaternion_from_matrix(rot_matrix)
    pose_origin = position

    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.pose = Pose()
    pose.pose.position.x = pose_origin[0]
    pose.pose.position.y = pose_origin[1]
    pose.pose.position.z = pose_origin[2]
    pose.pose.orientation.x = pose_quat[0]
    pose.pose.orientation.y = pose_quat[1]
    pose.pose.orientation.z = pose_quat[2]
    pose.pose.orientation.w = pose_quat[3]

    br.sendTransform((pose_origin[0], pose_origin[1], pose_origin[2]),
                     pose_quat,
                     rospy.Time.now(),
                     "calib_pose" + str(idx),
                     "world"
                     )

    return pose

if __name__ == '__main__':

    rospy.init_node('calib_generator')

    # Define ranges of theta and phi
    min_theta = np.pi / 2 + np.pi / 6
    max_theta = np.pi * 3/2 - np.pi / 6
    # min_phi = np.pi / 4
    min_phi = 0
    # max_phi = np.pi / 2 - np.pi / 12
    max_phi = 2 * np.pi
    theta_steps = 1
    phi_steps = 4
    theta = np.linspace(min_theta, max_theta, theta_steps, endpoint=False)
    phi = np.linspace(min_phi, max_phi, phi_steps, endpoint=False)

    # print angles:
    print("theta points: ", theta)
    print("phi points: ", phi)

    rad = 0.3

    # Define sphere center
    center = [0.6, 0, 0.1]

    # Define working directory to save images
    workdir = '/home/icub/data/test'

    # Sample uniformly by euler angles theta and phi
    points, theta, phi = get_trajectory(center, rad, min_theta, max_theta, theta_steps=theta_steps, phi_steps=phi_steps)

    # Get camera orientations w.r.t to points
    x_ax, y_ax, z_ax = get_orientations(center, points, theta, phi)

    eef_pose = None

    br = tf.TransformBroadcaster()
    rgb_sub = rospy.Subscriber('/camera/color/image_raw', Image, camera_data_callback)

    bridge = CvBridge()

    # Obtain poses as position + quaternion
    poses = []
    poses_reached = 0

    # save positions and rotation matrices
    positions = points
    rotations = np.zeros((x_ax.shape[0], 3, 3), dtype=np.float32)


    for idx in range(x_ax.shape[0]):
        next_position = points[idx, :]
        next_orientation = [x_ax[idx, :], y_ax[idx, :], z_ax[idx, :]]
        
        # save rotation matrix
        rot_matrix = np.eye(3)
        rot_matrix[:3, :3] = np.transpose(np.stack((
            x_ax[idx, :],
            y_ax[idx, :],
            z_ax[idx, :]
        )))

        rotations[idx, :, :] = rot_matrix

        pose = set_pose(br, next_position, next_orientation, idx)
        print("Next pose: ")
        print(pose.pose)

        # print("Press any key to proceed")
        # raw_input()

        # I don't know why we go home but let me keep this

        if idx % theta_steps == 0:
            rospy.wait_for_service('panda_grasp_server/panda_home')
            try:
                move_home = rospy.ServiceProxy('panda_grasp_server/panda_home', PandaHome)
                req = PandaHomeRequest(use_joint_values=True)
                move_success = move_home(req).success
                if not move_success:
                    print("Unable to go home. Quitting.")
                    break
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)


        rospy.wait_for_service('panda_grasp_server/panda_move_pose')
        try:
            move_to_pose = rospy.ServiceProxy('panda_grasp_server/panda_move_pose', PandaMove)
            move_success = move_to_pose(pose).success
            if move_success:
                poses_reached+=1
                get_robot_state = rospy.ServiceProxy('panda_grasp_server/panda_get_state', PandaGetState)
                robot_state = get_robot_state().robot_state
                eef_pose = robot_state.eef_state.pose
                print(eef_pose)
                # we need the pose in axis-angle representation
                eef_orient = Quaternion(eef_pose.orientation.w, eef_pose.orientation.x, eef_pose.orientation.y, eef_pose.orientation.z)
                eef_orient_rotvec = eef_orient.get_axis() * eef_orient.angle
                # pose_euler = euler_from_quaternion([eef_pose.orientation.x, eef_pose.orientation.x, eef_pose.orientation.z, eef_pose.orientation.w], axes=)
                cv_image = bridge.imgmsg_to_cv2(camera_image, desired_encoding='passthrough')
                # cv2.imwrite('/home/icub/calib_data/image-{}.png'.format(poses_reached), cv_image)
                cv2.imwrite(workdir + '/image_%d.png' % poses_reached, cv_image)
            else:
                print("Pose unreachable. Proceeding with next pose")
        except rospy.ServiceException as e:
           print("Service call failed: %s"%e)

    np.save(workdir + '/rots.npy', rotations)
    np.save(workdir + '/poss.npy', positions)


    rospy.wait_for_service('panda_grasp_server/panda_home')
    try:
        move_home = rospy.ServiceProxy('panda_grasp_server/panda_home', PandaHome)
        req = PandaHomeRequest(use_joint_values=True)
        success = move_home(req)
        if not success:
            print("Unable to go home. Quitting.")
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


