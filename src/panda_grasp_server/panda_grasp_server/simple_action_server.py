#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import math
import copy
import numpy as np
import rospy
import geometry_msgs
import moveit_commander
import moveit_msgs.msg
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from tf.transformations import quaternion_from_matrix, quaternion_matrix

from panda_grasp_server.srv import PandaGrasp, PandaGraspRequest, PandaGraspResponse,
                                UserCmd,
                                PandaMove, PandaMoveRequest, PandaMoveResponse,
                                PandaMoveWaypoints, PandaMoveWaypointsRequest, PandaMoveWaypointsResponse


def all_close(goal, actual, tolerance):

    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False
    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class NodeConfig(object):

    def __init__(self):

        # Configure service names
        self._user_cmd_service_name = "~user_cmd"
        self._move_service_name = "~panda_move_pose"
        self._grasp_service_name = "~panda_grasp"
        self._home_service_name = "~panda_home"
        self._move_wp_service_name = "~panda_move_wp"
        self._set_home_service_name = "~panda_set_home_pose"

        # Configure scene parameters
        self._table_height = 0.0 # z dimension, from the robot base ref frame
        self._robot_workspace = None
        self._bench_dimensions = (0.6, 0.6, 0.6) # x y z
        self._bench_mount_point_xy = (0.2, 0.0) # x y wrt center of the bench

        # Configure speed/accel scaling factors
        self._max_velocity_scaling_factor = 0.3
        self._max_acceleration_scaling_factor = 0.3

        # Configure planner ID
        self._planner_id = "RRTkConfigDefault"

        # Enable Rviz visualization of trajectories
        self._publish_rviz = True


class PandaActionServer(object):

    def __init__(self, config):

        # Configure moveit
        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()

        self._group_name = "panda_arm"
        self._move_group = moveit_commander.MoveGroupCommander(
            self._group_name)
        self._move_group.set_end_effector_link("panda_hand")
        self._eef_link = self._move_group.get_end_effector_link()
        self._move_group_hand = moveit_commander.MoveGroupCommander(
            "hand")

        self._max_velocity_scaling_factor = config._max_velocity_scaling_factor
        self._max_acceleration_scaling_factor = config._max_acceleration_scaling_factor

        self._move_group.set_max_velocity_scaling_factor(config._max_velocity_scaling_factor)
        self._move_group.set_max_acceleration_scaling_factor(config._max_acceleration_scaling_factor)

        self._move_group.set_planner_id("RRTkConfigDefault")

        # Display trajectories in Rviz
        self._publish_rviz = config._publish_rviz
        self._display_trajectory_publisher = None
        if self._publish_rviz:
            self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                                 moveit_msgs.msg.DisplayTrajectory,
                                                                 queue_size=10)

        self._scene = moveit_commander.PlanningSceneInterface()
        self._scene.remove_world_object()

        # Configure user input server
        self._cmd_srv = rospy.Service(config._user_cmd_service_name,
                                      UserCmd,
                                      self.user_cmd)

        # Configure grasping server
        self._grasp_service = rospy.Service(config._grasp_service_name,
                                            PandaGrasp,
                                            self.do_grasp_callback)

        # Configure movement server
        self._movement_server = rospy.Service(config._move_service_name,
                                              PandaMove,
                                              self.go_to_pose_callback)

        # Configure homing server
        self._homing_server = rospy.Service(config._home_service_name,
                                            PandaHome,
                                            self.go_home_callback)

        # Configure homing pose setting server
        self._set_home_pose_server = rospy.Service(config._set_home_service_name,
                                                    PandaSetHome,
                                                    self.set_home_pose_callback)

        # Configure trajectory movement server
        self._wp_movement_server = rospy.Service(config._move_wp_service_name,
                                                PandaMoveWaypoints,
                                                self.execute_trajectory_callback)

        # Configure home pose
        self._home_pose = geometry_msgs.msg.Pose()

        self._home_pose.orientation.x = 1.0
        self._home_pose.orientation.y = 0.0
        self._home_pose.orientation.z = 0.0
        self._home_pose.orientation.w = 0.0

        self._home_pose.position.x = 0.5
        self._home_pose.position.y = 0.0
        self._home_pose.position.z = 0.6

        # Alternative home pose in joint values
        # This home pose is the same defined in the libfranka tutorials
        self._home_pose_joints = self._move_group.get_current_joint_values()
        self._home_pose_joints[0:7] = [0,
                                       -math.pi/4,
                                       0,
                                       -3*math.pi/4,
                                       0,
                                       math.pi/2,
                                       math.pi/4]

        # Add table as a collision object
        if config._table_height:
            rospy.sleep(2)
            table_pose = geometry_msgs.PoseStamped()
            table_pose.header.frame_id = self._robot.get_planning_frame()
            table.pose.position.x = 0.9
            table.pose.position.y = 0.0
            table.pose.position.z = config._table_height
            self._scene.add_box("table", table_pose, (0.8, 0.8, 0.8))

        # Add the workbench
        rospy.sleep(2)
        workbench_pose = geometry_msgs.PoseStamped()
        workbench.header.frame_id = self._robot.get_planning_frame()
        workbench_pose.pose.position.x = -config._bench_mount_point_xy[0]
        workbench_pose.pose.position.y = config._bench_mount_point_xy[1]
        workbench_pose.pose.position.z = -1.0
        self._scene.add_box("workbench", workbench_pose, (0.8, 0.8, 0.8))


    def set_home(self, pos, quat):

        self._home_pose.orientation.x = quat[0]
        self._home_pose.orientation.y = quat[1]
        self._home_pose.orientation.z = quat[2]
        self._home_pose.orientation.w = quat[3]

        self._home_pose.position.x = pos[0]
        self._home_pose.position.y = pos[1]
        self._home_pose.position.z = pos[2]

    def set_home_pose_callback(self, req):

        # Callback for setting home state of the robot

        use_joint_values = req.use_joints

        ok_msg = (req.home_joints is not None) or (req,target_pose.pose is not None)

        if ok_msg:
            if use_joint_values:
                self._home_pose_joints[0:7] = req.home_joints
            else:
                self._home_pose = req.target_pose.pose
            rospy.INFO("New homing pose set")
            return True
        else:
            return False

    def go_home(self, use_joints=False):

        # Move the robot in home pose
        # use_joints flag uses the joint config instead of pose
        if use_joints:
            self._move_group.set_joint_value_target(self._home_pose_joints)
        else:
            self._move_group.set_pose_target(self._home_pose)
        self._move_group.go(wait=True)
        self._move_group.stop()
        self._move_group.clear_pose_targets()
        self.open_gripper()

    def close_gripper(self):
        joint_goal = self._move_group_hand.get_current_joint_values()
        if joint_goal[0] > 0.03 and joint_goal[1] > 0.03:
            self.command_gripper(0.0)
        else:
            print("gripper already closed")

    def open_gripper(self):
        joint_goal = self._move_group_hand.get_current_joint_values()
        if joint_goal[0] <= 0.03 and joint_goal[1] <= 0.03:
            self.command_gripper(0.08)
        else:
            print("gripper already open")

    def command_gripper(self, gripper_width):

        joint_goal = self._move_group_hand.get_current_joint_values()
        joint_goal[0] = gripper_width/2.
        joint_goal[1] = gripper_width/2.

        self._move_group_hand.go(joint_goal, wait=True)
        self._move_group_hand.stop()
        current_joints = self._move_group_hand.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.001)

    def get_gripper_state(self):
        joint_poses = self._move_group_hand.get_current_joint_values()
        return joint_poses

    def get_joints_state(self):
        joint_poses = self._move_group.get_current_joint_values()
        return joint_poses

    def get_current_pose_EE(self):

        quaternion = [self._move_group.get_current_pose().pose.orientation.x,
                      self._move_group.get_current_pose().pose.orientation.y,
                      self._move_group.get_current_pose().pose.orientation.z,
                      self._move_group.get_current_pose().pose.orientation.w]

        position = [self._move_group.get_current_pose().pose.position.x,
                    self._move_group.get_current_pose().pose.position.y,
                    self._move_group.get_current_pose().pose.position.z]

        # print("current gripper pose is:")
        # print(position)
        # print(quaternion)
        return [position, quaternion]

    def go_to_pose_callback(self, req):

        target_pose = self._move_group.get_current_pose()
        target_pose.pose.position = req.target_pose.pose.position
        target_pose.pose.orientation = req.target_pose.pose.orientation
        self._move_group.set_pose_target(target_pose)

        plan = self._move_group.go(wait=True)

        self._move_group.stop()
        self._move_group.clear_pose_targets()

        return True

    def go_home_callback(self, req):

        use_joint_values = req.use_joint_values

        self.go_home(use_joints=use_joint_values)

        return True

    def execute_trajectory_callback(self, req):

        # Plan and execute trajectory based on an array of pose waypoints
        waypoints = []
        pose_start = self._move_group.get_current_pose().pose
        waypoints.append(pose_start)

        # Add all the waypoints from the message
        for wp in req.pose_waypoints.poses:
            waypoints.append(wp)

        # Plan
        (plan, fraction) = self._move_group.compute_cartesian_path(
                                waypoints=waypoints,
                                eef_step=0.01,
                                jump_threshold=0.0,
                                avoid_collisions=True
                                )

        # Retime trajectory according to velocity/acceleration limits
        plan = self._move_group.retime_trajectory(self._robot.get_current_state(),
                                        plan,
                                        velocity_scaling_factor=self._max_velocity_scaling_factor,
                                        acceleration_scaling_factor=self._max_acceleration_scaling_factor)

        if fraction > 0.99:
            msg = "Moving robot arm. Planned " + str(fraction) + " of the trajectory"
            print(msg)
            self._move_group.execute(plan, wait=True)
            self._move_group.stop()
            self._move_group.clear_pose_targets()
            return PandaMoveWaypointResponse(
                success=True,
                message=msg
            )
        else:
            msg = "Plan failed. Planned " + str(fraction*100) + "% of the trajectory"
            print(msg)
            self._move_group.stop()
            self._move_group.clear_pose_targets()
            return PandaMoveWaypointResponse(
                success=False,
                message=msg
            )

    # def execute_trajectory(self):

    #     # Execute trajectory around home state

    #     self.go_home_joints(None)
    #     pose_start = self._move_group.get_current_pose().pose
    #     pose_wp1 = self._move_group.get_current_pose().pose
    #     pose_wp2 = self._move_group.get_current_pose().pose
    #     pose_wp3 = self._move_group.get_current_pose().pose

    #     pose_wp1.position.x += 0.2
    #     pose_wp2.position.x += 0.2
    #     pose_wp2.position.y += 0.2
    #     pose_wp3.position.z += 0.2

    #     waypoints = []
    #     waypoints.append(pose_start)
    #     waypoints.append(pose_wp1)
    #     waypoints.append(pose_wp2)
    #     waypoints.append(pose_wp3)
    #     waypoints.append(pose_start)

    #     (plan, fraction) = self._move_group.compute_cartesian_path(
    #                                     waypoints=waypoints,
    #                                     eef_step=0.01,
    #                                     jump_threshold=0.0,
    #                                     avoid_collisions=True
    #                                     )

    #     # Retime trajectory according to velocity/acceleration limits
    #     plan = self._move_group.retime_trajectory(self._robot.get_current_state(),
    #                                                 plan,
    #                                                 velocity_scaling_factor=self._max_velocity_scaling_factor,
    #                                                 acceleration_scaling_factor=self._max_acceleration_scaling_factor)

    #     if fraction > 0.8:
    #         print("moving robot arm. Planned " + str(fraction) + " of the trajectory")
    #         self._move_group.execute(plan, wait=True)
    #         self._move_group.stop()
    #         self._move_group.clear_pose_targets()
    #         return True
    #     else:
    #         print("plan failed")
    #         self._move_group.stop()
    #         self._move_group.clear_pose_targets()
    #         return False

    def _get_pose_from_user(self):
        position = [0]*3
        quaternion = [0, 1, 0, 0]

        user_cmd = raw_input("Set desired EE home position as 'x y z':")
        user_cmd = user_cmd.split()
        if not len(user_cmd) == 3:
            user_cmd = input("Wrong input. Try again: ")
            user_cmd = user_cmd.split()
            if not len(user_cmd) == 3:
                return [], []

        for i, cmd in enumerate(user_cmd):
            position[i] = float(cmd)

        user_cmd = raw_input("Set desired EE home orientation as quaternion 'x y z w': ")
        user_cmd = user_cmd.split()
        if not len(user_cmd) == 4:
            user_cmd = input("Wrong input. Try again: ")
            user_cmd = user_cmd.split()
            if not len(user_cmd) == 4:
                return [], []

        for i, cmd in enumerate(user_cmd):
            quaternion[i] = float(cmd)

        return position, quaternion

    def do_grasp_callback(self, req):
        rospy.loginfo('%s: Executing grasp' %
                      (self._grasp_service.resolved_name))

        # move fingers in pre grasp pose
        # self.command_gripper(req.width.data)
        self.open_gripper()

        # --- define a pre-grasp point along the approach axis --- #
        p1 = quaternion_matrix([0., 0., 0., 1.])
        p1[:3, 3] = np.array([0., 0., -0.1])

        # transform grasp in matrix notation
        q_gp = req.grasp.pose.orientation
        p_gp = req.grasp.pose.position
        gp = quaternion_matrix([q_gp.x, q_gp.y, q_gp.z, q_gp.w])
        gp[:3, 3] = np.array([p_gp.x, p_gp.y, p_gp.z])

        # create pregrasp pose
        pregrasp = np.matmul(gp, p1)
        q_pregrasp = quaternion_from_matrix(pregrasp)

        pregrasp_pose = geometry_msgs.msg.Pose()

        pregrasp_pose.orientation.x = q_pregrasp[0]
        pregrasp_pose.orientation.y = q_pregrasp[1]
        pregrasp_pose.orientation.z = q_pregrasp[2]
        pregrasp_pose.orientation.w = q_pregrasp[3]

        pregrasp_pose.position.x = pregrasp[0, 3]
        pregrasp_pose.position.y = pregrasp[1, 3]
        pregrasp_pose.position.z = pregrasp[2, 3]

        print("pregrasp")

        self._move_group.set_pose_target(pregrasp_pose)
        self._move_group.go(wait=True)
        self._move_group.stop()
        self._move_group.clear_pose_targets()

        print("grasp pose ")
        print(req.grasp.pose)

        self._move_group.set_pose_target(req.grasp.pose)
        self._move_group.go(wait=True)
        self._move_group.stop()
        self._move_group.clear_pose_targets()


        # --- Execute planned trajectory to grasp pose --- #
        # self._move_group.execute(plan, wait=True)

        # --- Close fingers to try the grasp --- #
        ok = self.close_gripper()

        print("lift")
        lift_pose = req.grasp.pose
        lift_pose.position.z += 0.30

        self._move_group.set_pose_target(lift_pose)
        self._move_group.go(wait=True)
        self._move_group.stop()
        self._move_group.clear_pose_targets()

        # --- Check if grasp was successful --- #
        gripper_state = self.get_gripper_state()
        success = False if sum(gripper_state) <= 0.01 else True
        print("gripper_state ", gripper_state)
        print("Grasp success? :", success)

        # --- drop object out of workspace --- #
        print("gripper home")
        next_pose = lift_pose
        next_pose.orientation.x = 1
        next_pose.orientation.y = 0
        next_pose.orientation.z = 0
        next_pose.orientation.w = 0

        self._move_group.set_pose_target(next_pose)
        self._move_group.go(wait=True)
        self._move_group.stop()
        self._move_group.clear_pose_targets()

        print("back/right")
        next_pose.position.x = 0.4
        next_pose.position.y = -0.4

        self._move_group.set_pose_target(next_pose)
        self._move_group.go(wait=True)
        self._move_group.stop()
        self._move_group.clear_pose_targets()

        print("down")
        next_pose.position.z = 0.45

        self._move_group.set_pose_target(next_pose)
        self._move_group.go(wait=True)
        self._move_group.stop()
        self._move_group.clear_pose_targets()

        # --- Open fingers to drop the object --- #
        print("release object")
        ok = self.open_gripper()

        print("up")
        next_pose.position.z = 0.60

        self._move_group.set_pose_target(next_pose)
        self._move_group.go(wait=True)
        self._move_group.stop()
        self._move_group.clear_pose_targets()

        # --- go in home pose --- #
        print("home")
        self.go_home()

        return success

    def user_cmd_callback(self, req):
        print("Received new command from user...")

        cmd = req.cmd.data

        if cmd=="help":
            print("available commands are:")
            print("go_home\nset_home\njoints_state\npose_ee\nmove_gripper\nexecute_traj")
            return True

        elif cmd == "go_home":
            self.go_home()
            return True

        elif cmd == "set_home":
            pos, quat = self._get_pose_from_user()
            if len(pos)==3 and len(quat)==4:
                self.set_home(pos, quat)
                return True
            else:
                return False

        elif cmd == "joints_state":
            joint_states = self.get_joints_state()
            print("joint poses: ", joint_states)
            gripper_poses = self.get_gripper_state()
            print("gripper poses: ", gripper_poses)
            return True

        elif cmd == "pose_ee":
            pos, quat = self.get_current_pose_EE()
            print("current gripper pose: ")
            print(pos)
            print(quat)
            return True
        elif cmd == "move_gripper":
            user_cmd = raw_input("Set desired gripper width:")
            width = float(user_cmd)
            print("required width ", width)

            self.command_gripper(width)
            return True
        elif cmd == "execute_traj":
            print("executing trajectory")
            self.execute_trajectory()
            return True
        else:
            print("unvalid command ", cmd)
            return False


if __name__ == "__main__":

    # Initialize the ROS node.
    rospy.init_node("panda_action_server")

    # Config
    config = NodeConfig()

    # Instantiate the action server.
    grasp_planner = PandaActionServer(config)

    # Spin forever.
    rospy.spin()
