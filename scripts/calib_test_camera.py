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

import os, sys

def normalize_rows(mat_to_normalize):

    return mat_to_normalize / np.linalg.norm(mat_to_normalize, axis=1, keepdims=True)

camera_image = Image()

def camera_data_callback(rgb):

    global camera_image
    camera_image = rgb



def fibonacci_sphere(samples=2):
    phi = np.pi * (3. - np.sqrt(5.))  # golden angle in radians

    i = np.linspace(0, samples, samples, endpoint=False)

    y = 1 - (i / (samples - 1)) * 2
    radius = np.sqrt(1 - y * y)
    theta = phi * i

    x = np.cos(theta) * radius
    z = np.sin(theta) * radius

    return np.stack((x, y, z), axis=1)

def fibonacci_trajectory(samples):
    pass


class ThetaPhiTrajectory:
    def __init__(self, theta_steps, phi_steps, max_theta, min_theta, max_phi, min_phi, radius, feasibles=None):
        theta = np.linspace(min_theta, max_theta, theta_steps)
        phi = np.linspace(min_phi, max_phi, phi_steps)
        
        angular_limits = np.array([min_theta, max_theta, theta_steps, min_phi, max_phi, phi_steps])

        wdir_rad = workdir + '/' + 'rad_%.2f' % rad
        if not os.path.exists(wdir_rad):
            os.mkdir(wdir_rad)
        
        np.save(wdir_rad + '/' + 'angles.npy', angular_limits)

        # Organize the parameter grid
        theta_space = np.empty((0,))
        phi_space = np.empty((0,))
        for idx in range(phi.size):
            theta_space = np.append(theta_space, theta)
            phi_space = np.append(phi_space, phi[idx]*np.ones_like(theta))

        theta_space = np.reshape(theta_space, (1, theta_space.size))
        phi_space = np.reshape(phi_space, (1, phi_space.size))

        # Define sphere center
        center = [0.45, 0.0, 0.20]

        # Obtain pose center points
        x = rad*np.cos(theta_space)*np.cos(phi_space) + center[0]
        y = rad*np.sin(theta_space)*np.cos(phi_space) + center[1]
        z = rad*np.sin(phi_space) + center[2]

        # find the x axis by looking at center of sphere
        # find the y axis by finding the derivative of x and y wrt theta
        # find the z axis by cross product

        points = np.hstack((np.transpose(x),np.transpose(y),np.transpose(z)))

        z_ax = np.reshape(center, (1,3)) - points
        z_ax = normalize_rows(z_ax)

        y_ax_x = -np.sin(theta_space)
        y_ax_y = np.cos(theta_space)
        y_ax_z = np.zeros_like(y_ax_x)

        y_ax = np.hstack((np.transpose(y_ax_x),np.transpose(y_ax_y),np.transpose(y_ax_z)))
        y_ax = normalize_rows(y_ax)

        x_ax = np.cross(y_ax, z_ax)
        x_ax = normalize_rows(x_ax)

        if feasibles is None:
            self.feasibles =  np.ones((x_ax.shape[0]),dtype=np.bool)

    def gen(self):
        for idx in range(x_ax.shape[0]):
            if not self.feasiblesp[idx]:
                continue
            rot_matrix = np.identity(4)
            rot_matrix[:3, :3] = np.transpose(np.stack((
                                                    x_ax[idx,:],
                                                    y_ax[idx,:],
                                                    z_ax[idx,:]
                                                )))


            extra_rot = np.transpose(np.stack((
                                                y_ax[idx, :],
                                                z_ax[idx, :],
                                                x_ax[idx, :]
                                                )))


            camera_shift = np.array([0.051, -0.01, 0.041]).T
            tcp_shift = np.array([0, 0, 0.105]).T


            extra_shift = np.zeros_like(camera_shift)

            extra_shift =  extra_rot.dot(camera_shift - tcp_shift )


            pose_quat = quaternion_from_matrix(rot_matrix)
            pose_origin = points[idx, :]

            # print(pose_origin)
            # print(extra_shift)

            pose = PoseStamped()
            pose.header.frame_id="world"
            pose.pose = Pose()
            pose.pose.position.x = pose_origin[0] + extra_shift[0]
            pose.pose.position.y = pose_origin[1] + extra_shift[1]
            pose.pose.position.z = pose_origin[2] + extra_shift[2]
            pose.pose.orientation.x = pose_quat[0]
            pose.pose.orientation.y = pose_quat[1]
            pose.pose.orientation.z = pose_quat[2]
            pose.pose.orientation.w = pose_quat[3]

            

            # print("Next pose: ")
            # print(pose.pose)

            yield pose


def explore_and_save(workdir, rad, theta_steps=96, phi_steps=5):
    # Define ranges of theta and phi
    # min_theta = np.pi / 2 + np.pi / 6
    min_theta = np.pi * 0 + np.pi / 3
    # max_theta = np.pi * 3/2 - np.pi / 6
    max_theta = np.pi * 2 - np.pi / 12
    if rad >= 0.3:
        min_phi = np.pi / 6
    else:
        min_phi = np.pi / 6
    
    max_phi = np.pi / 2 - np.pi / 12
    # theta_steps = 24
    # phi_steps = 3
    theta = np.linspace(min_theta, max_theta, theta_steps)
    phi = np.linspace(min_phi, max_phi, phi_steps)
    
    angular_limits = np.array([min_theta, max_theta, theta_steps, min_phi, max_phi, phi_steps])

    wdir_rad = workdir + '/' + 'rad_%.2f' % rad
    if not os.path.exists(wdir_rad):
        os.mkdir(wdir_rad)
    
    np.save(wdir_rad + '/' + 'angles.npy', angular_limits)

    # Organize the parameter grid
    theta_space = np.empty((0,))
    phi_space = np.empty((0,))
    for idx in range(phi.size):
        theta_space = np.append(theta_space, theta)
        phi_space = np.append(phi_space, phi[idx]*np.ones_like(theta))

    theta_space = np.reshape(theta_space, (1, theta_space.size))
    phi_space = np.reshape(phi_space, (1, phi_space.size))

    # Define sphere center
    center = [0.45, 0.0, 0.20]

    # Obtain pose center points
    x = rad*np.cos(theta_space)*np.cos(phi_space) + center[0]
    y = rad*np.sin(theta_space)*np.cos(phi_space) + center[1]
    z = rad*np.sin(phi_space) + center[2]

    # find the x axis by looking at center of sphere
    # find the y axis by finding the derivative of x and y wrt theta
    # find the z axis by cross product

    points = np.hstack((np.transpose(x),np.transpose(y),np.transpose(z)))

    z_ax = np.reshape(center, (1,3)) - points
    z_ax = normalize_rows(z_ax)

    y_ax_x = -np.sin(theta_space)
    y_ax_y = np.cos(theta_space)
    y_ax_z = np.zeros_like(y_ax_x)

    y_ax = np.hstack((np.transpose(y_ax_x),np.transpose(y_ax_y),np.transpose(y_ax_z)))
    y_ax = normalize_rows(y_ax)

    x_ax = np.cross(y_ax, z_ax)
    x_ax = normalize_rows(x_ax)

    eef_pose = None

    br = tf.TransformBroadcaster()
    rgb_sub = rospy.Subscriber('/camera/color/image_raw', Image, camera_data_callback)

    bridge = CvBridge()

    # Obtain poses as position + quaternion
    poses = []
    poses_reached = 0
    feasibles = np.zeros((x_ax.shape[0]),dtype=np.bool)


   


    for idx in range(x_ax.shape[0]):
        rot_matrix = np.identity(4)
        rot_matrix[:3, :3] = np.transpose(np.stack((
                                                x_ax[idx,:],
                                                y_ax[idx,:],
                                                z_ax[idx,:]
                                            )))


        extra_rot = np.transpose(np.stack((
                                            y_ax[idx, :],
                                            z_ax[idx, :],
                                            x_ax[idx, :]
                                            )))


        camera_shift = np.array([0.051, -0.01, 0.041]).T
        tcp_shift = np.array([0, 0, 0.105]).T


        extra_shift = np.zeros_like(camera_shift)

        extra_shift =  extra_rot.dot(camera_shift - tcp_shift )


        pose_quat = quaternion_from_matrix(rot_matrix)
        pose_origin = points[idx, :]

        print(pose_origin)
        print(extra_shift)

        pose = PoseStamped()
        pose.header.frame_id="world"
        pose.pose = Pose()
        pose.pose.position.x = pose_origin[0] + extra_shift[0]
        pose.pose.position.y = pose_origin[1] + extra_shift[1]
        pose.pose.position.z = pose_origin[2] + extra_shift[2]
        pose.pose.orientation.x = pose_quat[0]
        pose.pose.orientation.y = pose_quat[1]
        pose.pose.orientation.z = pose_quat[2]
        pose.pose.orientation.w = pose_quat[3]

        br.sendTransform((pose_origin[0], pose_origin[1], pose_origin[2]),
                        pose_quat,
                        rospy.Time.now(),
                        "calib_pose"+str(idx),
                        "world"
                        )

        print("Next pose: ")
        print(pose.pose)
        # print("Press any key to proceed")
        # raw_input()

        if idx%theta_steps == 0:
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
                feasibles[idx] = True
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
                cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                cv2.imwrite(wdir_rad + '/image_%d.png' % poses_reached, cv_image_rgb)

            else:
                print("Pose unreachable. Proceeding with next pose")
        except rospy.ServiceException as e:
           print("Service call failed: %s"%e)

    np.save(wdir_rad + '/' + 'feasibles.npy' , feasibles)
    np.save(wdir_rad + '/' + 'center.npy', np.array(center))

if __name__ == '__main__':

    rospy.init_node('calib_generator')
    
    workdir = '/home/icub/esafronov/data/test'
 
    radius_list = [0.3]

    rad = 0.4
    if len(sys.argv) > 1:
        workdir = sys.argv[1]

    if not os.path.exists(workdir):
        os.mkdir(workdir)

    for rad in radius_list:
        explore_and_save(workdir, rad, theta_steps=48)
 
    rospy.wait_for_service('panda_grasp_server/panda_home')
    try:
        move_home = rospy.ServiceProxy('panda_grasp_server/panda_home', PandaHome)
        req = PandaHomeRequest(use_joint_values=True)
        success = move_home(req)
        if not success:
            print("Unable to go home. Quitting.")
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Once calibration is done, the frame must be rotated from the ros conventional ref frame to the realsense one
# Input the angles from the visp calib procedure
# e_m_c_raw = np.array([0.0343389, 0.00886664, 1.54735])
# angle = np.linalg.norm(e_m_c_raw)
# axis = e_m_c_raw/angle
# e_m_c = pq.Quaternion(axis=axis, angle=angle)
# e_m_c * pq.Quaternion(axis=[1, 0, 0], degrees=90) * pq.Quaternion(axis=[0,0,1], degrees=90)

