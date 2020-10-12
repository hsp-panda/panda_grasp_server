#!/usr/bin/env python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import tf
import rospy
from tf.transformations import quaternion_from_matrix
from geometry_msgs.msg import Pose, PoseStamped
from pyquaternion import Quaternion
from panda_ros_common.msg import PandaState
from panda_ros_common.srv import PandaMove, PandaHome, PandaHomeRequest, PandaGetState
from sensor_msgs.msg import Image

def normalize_rows(mat_to_normalize):

    return mat_to_normalize / np.linalg.norm(mat_to_normalize, axis=1, keepdims=True)


if __name__ == '__main__':

    rospy.init_node('calib_generator')

    # Define ranges of theta and phi
    min_theta = np.pi / 2 + np.pi / 6
    max_theta = np.pi * 3/2 - np.pi / 6
    min_phi = np.pi / 4
    max_phi = np.pi / 2 - np.pi / 12
    theta_steps = 5
    phi_steps = 3
    theta = np.linspace(min_theta, max_theta, theta_steps)
    phi = np.linspace(min_phi, max_phi, phi_steps)
    rad = 0.5


    # Organize the parameter grid
    theta_space = np.empty((0,))
    phi_space = np.empty((0,))
    for idx in range(phi.size):
        theta_space = np.append(theta_space, theta)
        phi_space = np.append(phi_space, phi[idx]*np.ones_like(theta))

    theta_space = np.reshape(theta_space, (1, theta_space.size))
    phi_space = np.reshape(phi_space, (1, phi_space.size))

    # Define sphere center
    center = [0.6, 0, 0.15]

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

    camera_image = None
    eef_pose = None

    # Define camera callback

    def camera_data_callback(self, rgb):

        camera_image = rgb

    br = tf.TransformBroadcaster()
    rgb_sub = rospy.Subscriber('/camera/color/image_raw', Image, camera_data_callback)

    # Obtain poses as position + quaternion
    poses = []
    for idx in range(x_ax.shape[0]):
        rot_matrix = np.identity(4)
        rot_matrix[:3, :3] = np.transpose(np.stack((
                                                x_ax[idx,:],
                                                y_ax[idx,:],
                                                z_ax[idx,:]
                                            )))

        pose_quat = quaternion_from_matrix(rot_matrix)
        pose_origin = points[idx, :]

        pose = PoseStamped()
        pose.header.frame_id="world"
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
                        "calib_pose"+str(idx),
                        "world"
                        )

        print("Next pose: ")
        print(pose.pose)
        print("Press any key to proceed")
        raw_input()

        if idx%theta_steps == 0:
            rospy.wait_for_service('panda_action_server/panda_home')
            try:
                move_home = rospy.ServiceProxy('panda_action_server/panda_home', PandaHome)
                req = PandaHomeRequest(use_joint_values=True)
                success = move_home(req)
                if not success:
                    print("Unable to go home. Quitting.")
                    break
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

        rospy.wait_for_service('panda_action_server/panda_move_pose')
        try:
            move_to_pose = rospy.ServiceProxy('panda_action_server/panda_move_pose', PandaMove)
            success = move_to_pose(pose)
            if success:
                get_robot_state = rospy.ServiceProxy('panda_action_server/panda_get_state', PandaGetState)
                robot_state = get_robot_state()
                eef_pose = robot_state.eef_state.pose
                print(eef_pose)


            else:
                print("Pose unreachable. Proceeding with next pose")





        except rospy.ServiceException as e:
           print("Service call failed: %s"%e)




