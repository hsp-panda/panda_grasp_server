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
import std_msgs.msg
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from tf.transformations import quaternion_from_matrix, quaternion_matrix, rotation_matrix
from threading import Lock
import grippers
import os
import tf
import rospkg
import graspa_utils


from panda_ros_common.msg import PandaState
from panda_ros_common.srv import (PandaGrasp, PandaGraspRequest, PandaGraspResponse,
                                    UserCmd,
                                    PandaMove, PandaMoveRequest, PandaMoveResponse,
                                    PandaMoveWaypoints, PandaMoveWaypointsRequest, PandaMoveWaypointsResponse,
                                    PandaHome, PandaHomeRequest, PandaHomeResponse,
                                    PandaSetHome, PandaSetHomeRequest, PandaSetHomeResponse,
                                    PandaGripperCommand, PandaGripperCommandRequest, PandaGripperCommandResponse,
                                    PandaGetState, PandaGetStateRequest, PandaGetStateResponse,
                                    PandaSetVelAccelScalingFactors, PandaSetVelAccelScalingFactorsRequest, PandaSetVelAccelScalingFactorsResponse
                                    )

import actionlib
from franka_gripper.msg import GraspAction, GraspActionGoal, GraspActionResult
from franka_gripper.msg import MoveAction, MoveActionGoal, MoveActionResult
from franka_gripper.msg import StopAction, StopActionGoal, StopActionResult


class NodeConfig(object):

    # Ugly but necessary dictionary container class
    # Translates string into class handle
    gripper_types = {
        "FRANKA_HAND"       : grippers.FrankaHandGripper,
        "ROBOTIQ_2F"        : grippers.Robotiq2FGripper,
        "ROBOTIQ_2F_FC"     : grippers.Robotiq2FGripperForceControlled
    }

    def __init__(self):

        # Configure service names
        self._user_cmd_service_name = rospy.get_param("~service_names/user_cmd_service", "~user_cmd")
        self._move_service_name = rospy.get_param("~service_names/panda_move_pose_service", "~panda_move_pose")
        self._grasp_service_name = rospy.get_param("~service_names/panda_grasp_service", "~panda_grasp")
        self._home_service_name = rospy.get_param("~service_names/panda_home_service", "~panda_home")
        self._move_wp_service_name = rospy.get_param("~service_names/panda_move_wp_service", "~panda_move_wp")
        self._set_home_service_name = rospy.get_param("~service_names/panda_set_home_pose_service", "~panda_set_home_pose")
        self._stop_service_name = rospy.get_param("~service_names/panda_stop_service", "~panda_stop")
        self._gripper_cmd_service_name = rospy.get_param("~service_names/panda_gripper_cmd_service", "~panda_gripper_cmd")
        self._get_robot_state_service_name = rospy.get_param("~service_names/panda_get_state_service", "~panda_get_state")
        self._set_scaling_factor_service_name = rospy.get_param("~service_names/panda_set_scaling_factors_service", "~panda_set_scaling_factors")
        self._recover_service_name = rospy.get_param("~service_names/panda_error_recover_service", "~panda_error_recover")

        # Configure scene parameters
        self._table_height = rospy.get_param("~workspace/table_height", 0.13) # z distance from upper side of the table block, from the robot base ref frame
        self._table_size = rospy.get_param("~workspace/table_size", (1.0, 2.0, 0.8)) # x y z size of table block
        self._robot_workspace = None
        self._bench_dimensions = rospy.get_param("~workspace/bench_size", (0.6, 0.6, 0.6)) # x y z
        self._bench_mount_point_xy = rospy.get_param("~workspace/bench_mount_xy", (0.1, 0.0)) # x y wrt center of the bench

        # Configure speed/accel scaling factors
        self._max_velocity_scaling_factor = rospy.get_param("~planning/max_vel_scaling_factor", 0.3)
        self._max_acceleration_scaling_factor = rospy.get_param("~planning/max_acc_scaling_factor", 0.3)

        # Configure planner ID
        self._planner_id = rospy.get_param("~planning/planner_id", "RRTkConfigDefault")

        # Configure the end effector to use in the move group
        self._eef_link_id = rospy.get_param("~planning/eef_link_id", "panda_tcp")

        # Enable Rviz visualization of trajectories
        self._publish_rviz = rospy.get_param("~planning/publish_rviz", True)

        # Choose gripper type
        self._gripper_type = rospy.get_param("~gripper/gripper_type", "FRANKA_HAND")

        # Enable force-controlled grasping
        self._enable_force_grasp = rospy.get_param("~ops_params/enable_force_grasp", False)

        # Enable GRASPA stability motion after grasp
        self._enable_graspa_stab_motion = rospy.get_param("~ops_params/enable_graspa_stab_motion", False)

        # Save grasp path
        self._save_grasp = rospy.get_param("~ops_params/enable_graspa_save_grasp", False)
        rospack = rospkg.RosPack()
        self._grasp_save_path = rospy.get_param("~ops_params/grasp_save_path", os.path.join(rospack.get_path('panda_grasp_server'), 'dumped_grasps'))

class PandaActionServer(object):

    def __init__(self, config):

        # Configure moveit
        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()

        self._group_name = "panda_arm"
        self._move_group = moveit_commander.MoveGroupCommander(
            self._group_name)
        self._move_group.set_end_effector_link(config._eef_link_id)
        self._eef_link = self._move_group.get_end_effector_link()

        self._max_velocity_scaling_factor = config._max_velocity_scaling_factor
        self._max_acceleration_scaling_factor = config._max_acceleration_scaling_factor

        self._move_group.set_max_velocity_scaling_factor(config._max_velocity_scaling_factor)
        self._move_group.set_max_acceleration_scaling_factor(config._max_acceleration_scaling_factor)

        self._move_group.set_planner_id(config._planner_id)

        # Display trajectories in Rviz
        self._publish_rviz = config._publish_rviz
        self._display_trajectory_publisher = None
        if self._publish_rviz:
            self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                                 moveit_msgs.msg.DisplayTrajectory,
                                                                 queue_size=10)

        self._scene = moveit_commander.PlanningSceneInterface()
        self._scene.remove_world_object()

        # Configure status monitor condition
        self._stop_mutex = Lock()
        self._motion_stopped = False

        # Set the force grasp flag
        self._enable_force_grasp = config._enable_force_grasp

        # Set the stability motion enable flag
        self._enable_graspa_stab_motion = config._enable_graspa_stab_motion

        # Set path to save grasps to
        self._save_grasp = config._save_grasp
        self._grasp_save_path = config._grasp_save_path

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

        # Configure motion stop server
        self._movement_stop_server = rospy.Service(config._stop_service_name,
                                                    Trigger,
                                                    self.stop_motion_callback)

        # Configure error clear server
        self._error_recover_server = rospy.Service(config._recover_service_name,
                                                   Trigger,
                                                   self.recover_error_callback)

        # Configure gripper closing server
        self._gripper_cmd_server = rospy.Service(config._gripper_cmd_service_name,
                                                    PandaGripperCommand,
                                                    self.gripper_command_callback)

        # Configure get status server
        self._get_robot_state_server = rospy.Service(config._get_robot_state_service_name,
                                                    PandaGetState,
                                                    self.get_state_callback)

        # Configure speed changing server
        self._set_vel_accel_server = rospy.Service(config._set_scaling_factor_service_name,
                                                PandaSetVelAccelScalingFactors,
                                                self.set_vel_accel_scaling_factor_callback)

        # Configure gripper action clients
        self._gripper = config.gripper_types[config._gripper_type]()
        rospy.loginfo("Gripper setup complete")

        # Configure TF transform listener
        self._tf_listener = tf.TransformListener(True, rospy.Duration(10))

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
        # This home pose is the same defined in the moveit package
        self._home_pose_joints = self._move_group.get_current_joint_values()
        self._home_pose_joints[0:7] = [0.0,
                                       0.0,
                                       0.0,
                                       -math.pi/2,
                                       0.0,
                                       math.pi/2,
                                       0.0]


        # Add table as a collision object
        if config._table_height is not None:
            rospy.sleep(2)
            table_pose = geometry_msgs.msg.PoseStamped()
            table_pose.header.frame_id = self._robot.get_planning_frame()
            table_pose.pose.position.x = 0.2 + config._table_size[0]/2
            table_pose.pose.position.y = 0.0
            table_pose.pose.position.z = config._table_height - config._table_size[2]/2
            self._scene.add_box("table", table_pose, config._table_size)

        # Add the workbench
        rospy.sleep(2)
        workbench_pose = geometry_msgs.msg.PoseStamped()
        workbench_pose.header.frame_id = self._robot.get_planning_frame()
        workbench_pose.pose.position.x = -config._bench_mount_point_xy[0]
        workbench_pose.pose.position.y = config._bench_mount_point_xy[1]
        workbench_pose.pose.position.z = -config._bench_dimensions[2]/2

        # Turn off collisions between panda_link0 and workbench
        # self._scene.add_box("workbench", workbench_pose, config._bench_dimensions)
        self._scene.attach_box('panda_link0', 'workbench', pose=workbench_pose, size=config._bench_dimensions, touch_links=['panda_link0', 'panda_link1'])

        # Not sure why editing the ACM does not work, I'll leave code here anyway
        # from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
        # from moveit_msgs.srv import GetPlanningScene
        # pub_planning_scene = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10)
        # rospy.wait_for_service('/get_planning_scene', 10.0)
        # get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        # request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        # response = get_planning_scene(request)
        # acm = response.scene.allowed_collision_matrix
        # if not 'workbench' in acm.default_entry_names:
        #     # add button to allowed collision matrix
        #     acm.default_entry_names += ['workbench']
        #     acm.default_entry_values += [True]

        #     planning_scene_diff = PlanningScene(
        #             is_diff=True,
        #             allowed_collision_matrix=acm)

        #     pub_planning_scene.publish(planning_scene_diff)
        #     rospy.sleep(1.0)

    def set_stopped_status(self, stopped=True):

        # Set the motion stopped status of the server
        # Stopped = True means robot has been stopped
        # Necessary to stop trajectory execution

        self._stop_mutex.acquire()
        self._motion_stopped = stopped
        self._stop_mutex.release()

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

        ok_msg = (req.home_joints is not None) or (req.home_pose.pose is not None)

        if ok_msg:
            if use_joint_values:
                self._home_pose_joints[0:7] = req.home_joints
            else:
                self._home_pose = req.home_pose.pose
            rospy.loginfo("New homing pose set")
            return True
        else:
            return False

    def go_home(self, use_joints=False, home_gripper=True):

        # Move the robot in home pose
        # use_joints flag uses the joint config instead of pose
        if use_joints:
            self._move_group.set_joint_value_target(self._home_pose_joints)
        else:
            self._move_group.set_pose_target(self._home_pose)
        self._move_group.go(wait=True)
        self._move_group.stop()
        self._move_group.clear_pose_targets()

        if home_gripper:
            self.open_gripper()

        return True

    def close_gripper(self):

        return self._gripper.close_gripper()

    def open_gripper(self):

        return self._gripper.open_gripper()

    def command_gripper(self, gripper_width, velocity=0.1):

        # This command just moves the fingers. Actual gripper behaviour
        # depends on what type of gripper is actually used

        return self._gripper.move_fingers(gripper_width, velocity)

    def get_gripper_state(self):
        # TODO reimplement this according to new grippers module
        # joint_poses = self._move_group_hand.get_current_joint_values()
        joint_poses = [0.0]
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

        return [position, quaternion]

    def go_to_pose(self, target_pose, message=None):

        self._move_group.clear_pose_targets()

        if not self._motion_stopped:
            self._move_group.set_pose_target(target_pose)
            plan = self._move_group.plan(target_pose)
            if plan.joint_trajectory.points:
                if message:
                    rospy.loginfo(str(message))
                move_success = self._move_group.execute(plan, wait=True)
            else:
                rospy.loginfo("Trajectory planning has failed")
                move_success = False
        else:
            rospy.loginfo("Motion is stopped. Aborting movement")
            move_success=False

        self._move_group.stop()
        self._move_group.clear_pose_targets()

        return move_success

    def go_to_pose_callback(self, req):

        target_pose = self._move_group.get_current_pose()
        target_pose.pose.position = req.target_pose.pose.position
        target_pose.pose.orientation = req.target_pose.pose.orientation

        move_success = self.go_to_pose(target_pose, message='Executing motion')

        return move_success

    def go_home_callback(self, req):

        use_joint_values = req.use_joint_values

        self.go_home(use_joints=use_joint_values, home_gripper=req.home_gripper)

        return True

    def gripper_command_callback(self, req):

        # Handle weird command
        if req.close_gripper and req.open_gripper:
            rospy.loginfo("Gripper command invalid. No operation will be performed")
            return False

        if req.close_gripper:
            return self.close_gripper()
        elif req.open_gripper:
            return self.open_gripper()
        else:
            return self.command_gripper(req.width)

    def get_state_callback(self, req):

        # Return the state of the robot joints, gripper width and eef pose
        robot_state = PandaState()

        # EEF pose
        robot_state.eef_state = self._move_group.get_current_pose()

        # Joints state
        robot_state.joints_state = self._move_group.get_current_joint_values()

        # Gripper width
        # TODO: reimplement this!
        #width = self._move_group_hand.get_current_joint_values()[0] + self._move_group_hand.get_current_joint_values()[1]
        #robot_state.gripper_state = width
        robot_state.gripper_state = 0.0

        return PandaGetStateResponse(robot_state=robot_state)

    def execute_trajectory(self, waypoints, constraints=None):

        # Plan and execute trajectory based on an array of pose waypoints

        # Plan
        (plan, fraction) = self._move_group.compute_cartesian_path(
                                waypoints=waypoints,
                                eef_step=0.01,
                                jump_threshold=0.0,
                                avoid_collisions=True,
                                path_constraints=constraints
                                )

        # Retime trajectory according to velocity/acceleration limits
        plan = self._move_group.retime_trajectory(self._robot.get_current_state(),
                                        plan,
                                        velocity_scaling_factor=self._max_velocity_scaling_factor,
                                        acceleration_scaling_factor=self._max_acceleration_scaling_factor)

        if fraction > 0.5:
            msg = "Moving robot arm. Planned " + str(fraction) + " of the trajectory"
            rospy.loginfo(msg)
            self._move_group.execute(plan, wait=True)
            self._move_group.stop()
            self._move_group.clear_pose_targets()
            return True
        else:
            msg = "Plan failed. Planned " + str(fraction*100) + "% of the trajectory"
            rospy.loginfo(msg)
            self._move_group.stop()
            self._move_group.clear_pose_targets()
            return False

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

        if fraction > 0.5:
            msg = std_msgs.msg.String("Moving robot arm. Planned " + str(fraction) + " of the trajectory")
            rospy.loginfo(msg)
            self._move_group.execute(plan, wait=True)
            self._move_group.stop()
            self._move_group.clear_pose_targets()
            return PandaMoveWaypointsResponse(
                success=True,
                message=msg
            )
        else:
            msg = std_msgs.msg.String("Plan failed. Planned " + str(fraction*100) + "% of the trajectory")
            rospy.loginfo(msg)
            self._move_group.stop()
            self._move_group.clear_pose_targets()
            return PandaMoveWaypointsResponse(
                success=False,
                message=msg
            )

    def stop_motion_callback(self, req):

        self._move_group.stop()
        self._gripper.stop_gripper()
        self.set_stopped_status(stopped=True)

        return TriggerResponse(
                    success=True,
                    message="Motion stopped"
        )

    def recover_error_callback(self, req):

        # Recover from error and reset motion status

        from franka_msgs.msg import ErrorRecoveryActionGoal

        pub = rospy.Publisher("/franka_control/error_recovery/goal", ErrorRecoveryActionGoal, queue_size=10, latch=True)
        msg = ErrorRecoveryActionGoal()
        for i in range(10):
            pub.publish(msg)
            rospy.sleep(0.1)

        self.set_stopped_status(stopped=False)

        return TriggerResponse(
                    success=True,
                    message="Recovered from error"
        )

    def set_vel_accel_scaling_factors(self, vel_scal_fac, acc_scal_fac):

        # Set max velocity and acceleration scaling factors

        self._max_velocity_scaling_factor = vel_scal_fac
        self._max_acceleration_scaling_factor = acc_scal_fac

        self._move_group.set_max_velocity_scaling_factor(vel_scal_fac)
        self._move_group.set_max_acceleration_scaling_factor(acc_scal_fac)

    def set_vel_accel_scaling_factor_callback(self, req):

        self.set_vel_accel_scaling_factors(req.new_vel_scaling_factor,
                                           req.new_accel_scaling_factor)

        return True

    def grasp(self, width, velocity=0.5, force=20):

        # Execute grasp directly with the gripper action server
        # Different behaviour according to the enable_force_grasp flag
        if self._enable_force_grasp:

            grasp_result = self._gripper.grasp_motion(width, velocity, force)

        else:

            grasp_result = self._gripper.move_fingers(width, velocity, force)

        return grasp_result

    def check_trajectory_feasibility(self, goals):

        # Check if a trajectory from the current status to the end goal
        # can be computed successfully
        # Return true or false

        robot_initial_state = self._move_group.get_current_state()
        robot_current_state = copy.deepcopy(robot_initial_state)

        for next_goal in goals:

            # Plan from current state to next goal
            self._move_group.clear_pose_targets()
            self._move_group.set_start_state(robot_current_state)
            self._move_group.set_pose_target(next_goal)
            current_plan = self._move_group.plan()

            # Return if any part of the plan is not feasible
            if not current_plan.joint_trajectory.points:
                self._move_group.clear_pose_targets()
                self._move_group.set_start_state(robot_initial_state)
                return False

            # If a plan is feasible, set final goal of previous plan a current state
            joint_val_list = list(robot_current_state.joint_state.position)
            for joint_idx, joint_val in enumerate(current_plan.joint_trajectory.points[-1].positions):
                joint_val_list[joint_idx] = joint_val
            robot_current_state.joint_state.position = tuple(joint_val_list)

        # If we got this far, the whole thing is feasible
        self._move_group.clear_pose_targets()
        self._move_group.set_start_state(robot_initial_state)
        return True

    def do_grasp_callback(self, req):

        rospy.loginfo('%s: Executing grasp' %
                      (self._grasp_service.resolved_name))

        # First of all, define a bunch of intermediate goals
        # Pregrasp
        # Lift pose
        # Drop pose

        # ---------------- Compute pregrasp pose
        # Transform grasp in homogeneous matrix notation
        target_grasp_pose_q = req.grasp.pose.orientation
        target_grasp_pose_p = req.grasp.pose.position
        target_grasp_pose = quaternion_matrix([target_grasp_pose_q.x,
                                              target_grasp_pose_q.y,
                                              target_grasp_pose_q.z,
                                              target_grasp_pose_q.w])
        target_grasp_pose[:3, 3] = np.array([target_grasp_pose_p.x,
                                             target_grasp_pose_p.y,
                                             target_grasp_pose_p.z])

        # Define a pre-grasp point along the approach axis
        approach_offset = quaternion_matrix([0., 0., 0., 1.])
        approach_offset[:3, 3] = np.array([0., 0., -0.15])

        # Create pregrasp pose
        pregrasp_pose = geometry_msgs.msg.Pose()

        pregrasp = np.matmul(target_grasp_pose, approach_offset)
        pregrasp_q = quaternion_from_matrix(pregrasp)

        pregrasp_pose.orientation.x = pregrasp_q[0]
        pregrasp_pose.orientation.y = pregrasp_q[1]
        pregrasp_pose.orientation.z = pregrasp_q[2]
        pregrasp_pose.orientation.w = pregrasp_q[3]

        pregrasp_pose.position.x = pregrasp[0, 3]
        pregrasp_pose.position.y = pregrasp[1, 3]
        pregrasp_pose.position.z = pregrasp[2, 3]

        # ---------------- Compute lift pose
        lift_pose = copy.deepcopy(req.grasp.pose)
        lift_pose.position.z += 0.20

        # ---------------- Compute dropoff pose
        drop_pose = lift_pose
        drop_pose.orientation.x = 1
        drop_pose.orientation.y = 0
        drop_pose.orientation.z = 0
        drop_pose.orientation.w = 0

        drop_pose.position.x = 0.45
        drop_pose.position.y = -0.5
        drop_pose.position.z = 0.4

        # If just testing, return such result
        if req.plan_only:
            goals = [pregrasp_pose, lift_pose, drop_pose]
            return self.check_trajectory_feasibility(goals)

        # Move fingers in pre grasp pose
        # self.command_gripper(req.width.data)
        self.open_gripper()

        if not self.go_home(use_joints=True):
            return False

        # We acquire the board pose, if needed, before moving the robot
        if self._save_grasp:
            graspa_board_pose = graspa_utils.get_GRASPA_board_pose(self._tf_listener)
        else:
            graspa_board_pose = None

        if not self.go_to_pose(pregrasp_pose, "Moving to pregrasp pose"):
            return False

        approach_waypoints = []
        n_approach_waypoints = 10
        approach_range_x = target_grasp_pose_p.x - pregrasp_pose.position.x
        approach_range_y = target_grasp_pose_p.y - pregrasp_pose.position.y
        approach_range_z = target_grasp_pose_p.z - pregrasp_pose.position.z
        for idx_waypoint in range(n_approach_waypoints + 1):
            wp = copy.deepcopy(pregrasp_pose)
            wp.position.x = pregrasp_pose.position.x + approach_range_x * idx_waypoint / n_approach_waypoints
            wp.position.y = pregrasp_pose.position.y + approach_range_y * idx_waypoint / n_approach_waypoints
            wp.position.z = pregrasp_pose.position.z + approach_range_z * idx_waypoint / n_approach_waypoints
            approach_waypoints.append(wp)

        # Try enforcing a constraint during approach
        approach_constraints = moveit_msgs.msg.Constraints()
        orient_constraint = moveit_msgs.msg.OrientationConstraint()
        orient_constraint.header.frame_id = "panda_link0"
        orient_constraint.link_name = self._eef_link
        orient_constraint.orientation = pregrasp_pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.1
        orient_constraint.absolute_y_axis_tolerance = 0.1
        orient_constraint.absolute_z_axis_tolerance = 0.1
        orient_constraint.weight = 1.0
        approach_constraints.orientation_constraints.append(orient_constraint)

        if not self.execute_trajectory(approach_waypoints, approach_constraints):

            # If there is no plan available for this grasp pose, try flipping it around the approach axis and replan
            rospy.logwarn("Target pose unreachable. Replanning with flipped pose. Press a key to proceed")
            raw_input()

            target_grasp_pose = np.dot(target_grasp_pose,
                                       quaternion_matrix([0, 0, 1, 0]))

            # Define a pre-grasp point along the approach axis
            approach_offset = quaternion_matrix([0., 0., 0., 1.])
            approach_offset[:3, 3] = np.array([0., 0., -0.15])

            # Create pregrasp pose
            pregrasp_pose = geometry_msgs.msg.Pose()

            pregrasp = np.matmul(target_grasp_pose, approach_offset)
            pregrasp_q = quaternion_from_matrix(pregrasp)

            pregrasp_pose.orientation.x = pregrasp_q[0]
            pregrasp_pose.orientation.y = pregrasp_q[1]
            pregrasp_pose.orientation.z = pregrasp_q[2]
            pregrasp_pose.orientation.w = pregrasp_q[3]

            pregrasp_pose.position.x = pregrasp[0, 3]
            pregrasp_pose.position.y = pregrasp[1, 3]
            pregrasp_pose.position.z = pregrasp[2, 3]

            # Re-create waypoints
            approach_waypoints = []
            n_approach_waypoints = 10
            approach_range_x = target_grasp_pose_p.x - pregrasp_pose.position.x
            approach_range_y = target_grasp_pose_p.y - pregrasp_pose.position.y
            approach_range_z = target_grasp_pose_p.z - pregrasp_pose.position.z
            for idx_waypoint in range(n_approach_waypoints + 1):
                wp = copy.deepcopy(pregrasp_pose)
                wp.position.x = pregrasp_pose.position.x + approach_range_x * idx_waypoint / n_approach_waypoints
                wp.position.y = pregrasp_pose.position.y + approach_range_y * idx_waypoint / n_approach_waypoints
                wp.position.z = pregrasp_pose.position.z + approach_range_z * idx_waypoint / n_approach_waypoints

                approach_waypoints.append(wp)

            # Try enforcing a constraint during approach
            approach_constraints = moveit_msgs.msg.Constraints()
            orient_constraint = moveit_msgs.msg.OrientationConstraint()
            orient_constraint.header.frame_id = "panda_link0"
            orient_constraint.link_name = "panda_tcp"
            orient_constraint.orientation = pregrasp_pose.orientation
            orient_constraint.absolute_x_axis_tolerance = 0.1
            orient_constraint.absolute_y_axis_tolerance = 0.1
            orient_constraint.absolute_z_axis_tolerance = 0.1
            orient_constraint.weight = 1.0
            approach_constraints.orientation_constraints.append(orient_constraint)

            # Re-attempt with the flipped pose
            if not self.execute_trajectory(approach_waypoints, approach_constraints):
                rospy.logwarn("Replanning failed. Grasp pose is unreachable.")
                self.go_home(use_joints=True)
                return False

            self.go_home(use_joints=True)
            return False

        # Try to grasp. In case of failure, go back to pregrasp pose and go home
        grasp_success = self.grasp(req.width.data)
        if not grasp_success:
            rospy.logwarn("Grasp failed!")
            self.open_gripper()
            self.go_to_pose(pregrasp_pose)
            self.go_home(use_joints=True)
            return False

        if not self.go_to_pose(pregrasp_pose, message="Moving back from grasping pose"):
            return False

        if not self.go_to_pose(lift_pose, message="Lifting from grasping pose"):
            return False

        # Check if grasp was successful
        # gripper_state = self.get_gripper_state()
        # grasp_success = False if sum(gripper_state) <= 0.01 else True
        # rospy.loginfo("Gripper state: " +  str(gripper_state[0] + gripper_state[1]))
        # rospy.loginfo("Grasp success? " + str(grasp_success))

        # Check stability
        if self._enable_graspa_stab_motion:
            self.evaluate_stability(lift_pose, [0.3, -0.3, 0.5])

        # Move object out of workspace

        if not self.go_to_pose(drop_pose, message="Dropping object away from workspace"):
            return False

        self.open_gripper()

        self.go_home(use_joints=True)

        if self._save_grasp:
            # Save the grasp
            save = raw_input("Save grasp? [y/N]")
            if save.lower() == 'y':
                graspa_utils.save_GRASPA_grasp(self._grasp_save_path, req.grasp, graspa_board_pose)

        return grasp_success

    def evaluate_stability(self, grasp_pose, tcp_travel, approach_rotation_angle=np.pi/4, binormal_rotation_angle=np.pi/6):

        # Evaluate stability of the grasp according to GRASPA v1.0
        # Approach and binormal rotation angles are in radians
        # tcp_travel is the 3D point to take the tcp to (while maintaining orientation) before executing rotations

        # Get initial orientation as a 4x4 homogeneous matrix
        # Rotation is the grasp rotation
        # Position of the tcp is tcp_travel
        grasp_orientation_quat = [grasp_pose.orientation.x,
                                  grasp_pose.orientation.y,
                                  grasp_pose.orientation.z,
                                  grasp_pose.orientation.w]
        grasp_orientation_m = quaternion_matrix(grasp_orientation_quat)

        evaluation_center_pose = grasp_orientation_m
        evaluation_center_pose[:3, 3] = np.array([tcp_travel[0],
                                                  tcp_travel[1],
                                                  tcp_travel[2]])

        # Compute rotations around axes in matrix form
        approach_axis = grasp_orientation_m[:3, 2]
        appr_rot_positive = rotation_matrix(angle=approach_rotation_angle,
                                            direction=approach_axis,
                                            point=np.zeros(3)
                                            )
        appr_rot_negative = rotation_matrix(angle=-approach_rotation_angle,
                                            direction=approach_axis,
                                            point=np.zeros(3)
                                            )
        binormal_axis = grasp_orientation_m[:3, 1]
        bin_rot_positive = rotation_matrix(angle=binormal_rotation_angle,
                                           direction=binormal_axis,
                                           point=np.zeros(3)
                                           )
        bin_rot_negative = rotation_matrix(angle=-binormal_rotation_angle,
                                           direction=binormal_axis,
                                           point=np.zeros(3)
                                           )


        # Obtain target poses
        appr_rot_positive_pose = np.dot(evaluation_center_pose, appr_rot_positive)
        appr_rot_negative_pose = np.dot(evaluation_center_pose, appr_rot_negative)
        bin_rot_positive_pose = np.dot(evaluation_center_pose, bin_rot_positive)
        bin_rot_negative_pose = np.dot(evaluation_center_pose, bin_rot_negative)

        def numpy_to_pose(pose_4x4):

            pose_msg = geometry_msgs.msg.Pose()

            # order as in geometry_msg/Pose.msg
            position_list = pose_4x4[:3, 3].tolist()
            orientation_list = quaternion_from_matrix(pose_4x4).tolist()

            pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = [val for val in position_list]
            pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w = [val for val in orientation_list]

            return pose_msg

        wp_1 = numpy_to_pose(appr_rot_positive_pose)
        wp_2 = numpy_to_pose(appr_rot_negative_pose)
        wp_3 = numpy_to_pose(bin_rot_positive_pose)
        wp_4 = numpy_to_pose(bin_rot_negative_pose)
        wp_start = numpy_to_pose(evaluation_center_pose)

        # Create waypoint list
        waypoints = [wp_start,
                     wp_1,
                     wp_start,
                     wp_2,
                     wp_start,
                     wp_3,
                     wp_start,
                     wp_4,
                     wp_start]

        # Move the robot
        # motion_success = self.execute_trajectory(waypoints)

        # self._move_group.set_pose_targets(waypoints)
        # self._move_group.go()

        for wp in waypoints:
            self.go_to_pose(wp)
        # self.go_to_pose(wp_start)
        # self.go_to_pose(wp_1)
        # self.go_to_pose(wp_start)
        # self.go_to_pose(wp_2)
        # self.go_to_pose(wp_start)
        # self.go_to_pose(wp_3)
        # self.go_to_pose(wp_start)
        # self.go_to_pose(wp_4)
        # self.go_to_pose(wp_start)

        return True

def main():

    # Initialize the ROS node.
    rospy.init_node("panda_grasp_server")

    # Config
    config = NodeConfig()

    # Instantiate the action server.
    grasp_planner = PandaActionServer(config)

    # Spin forever.
    rospy.spin()
