#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from abc import ABCMeta, abstractproperty, abstractmethod
import rospy
import actionlib
from franka_gripper.msg import GraspAction, GraspActionGoal, GraspActionResult
from franka_gripper.msg import MoveAction, MoveActionGoal, MoveActionResult
from franka_gripper.msg import StopAction, StopActionGoal, StopActionResult
from franka_gripper.msg import HomingAction, HomingActionGoal, HomingActionResult

class GripperInterface(object):
    """ General interface for a Gripper class. Assumes a gripper has a maximum
    and minimum finger aperture, and requires some attributes and implementation of
    methods to move the fingers.

    """

    __metaclass__ = ABCMeta

    # A bunch of values strictly related to each gripper type have to be
    # explicitly declare when implementing each class

    @abstractproperty
    def _gripper_name(self):
        pass

    @abstractproperty
    def _min_width(self):
        pass

    @abstractproperty
    def _max_width(self):
        pass

    @abstractproperty
    def _min_speed(self):
        pass

    @abstractproperty
    def _max_speed(self):
        pass

    @abstractproperty
    def _min_force(self):
        pass

    @abstractproperty
    def _max_force(self):
        pass

    # Although grippers vary a lot in structure and working principles, some
    # methods have to be common

    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def home_gripper(self):
        pass

    @abstractmethod
    def move_fingers(self, pos, speed, force):
        pass

    @abstractmethod
    def close_gripper(self):
        pass

    @abstractmethod
    def open_gripper(self):
        pass

    @abstractmethod
    def stop_gripper(self):
        pass

    @abstractmethod
    def get_gripper_status(self):
        pass

    # Some utils methods

    def is_pos_in_bounds(self, pos):
        return (pos > self._min_width) and (pos < self._max_width)

    def is_speed_in_bounds(self, speed):
        return (speed > self._min_speed) and (speed < self._max_speed)

    def is_force_in_bounds(self, force):
        return (force > self._min_force) and (force < self._max_force)


class FrankaHandGripper(GripperInterface):
    """Wrapper class for the Franka Gripper class.

    Manages the gripper via the Franka Gripper Action client.

    Implements the GripperInterface.
    """

    # Gripper parameters. These are not supposed to be changable at runtime
    _gripper_name = "Franka Hand"
    _min_width = 0.0                        # m/s
    _max_width = 0.08
    _min_speed = 0.0
    _max_speed = 0.1
    _min_force = 10                         # N
    _max_force = 70

    def _init__(self, gripper_action_namespace = "/franka_gripper"):

        # The gripper works by interfacing with different actions, which have
        # a common namespace
        homing_action_name = gripper_action_namespace + "/homing"
        move_action_name = gripper_action_namespace + "/move"
        grasp_action_name = gripper_action_namespace + "/grasp"
        stop_action_namespace = gripper_action_namespace + "/stop"

        self._inner_epsilon = rospy.get_param(gripper_action_namespace + "/default_grasp_epsilon/inner", 0.005)
        self._outer_epsilon = rospy.get_param(gripper_action_namespace + "/default_grasp_epsilon/outer", 0.005)

        # Configure the gripper action clients. Raise an exception if something
        # doesn't work
        self._homing_action_client = actionlib.SimpleActionClient(homing_action_name, HomingAction)
        self._move_action_client = actionlib.SimpleActionClient(move_action_name, MoveAction)
        self._grasp_action_client = actionlib.SimpleActionClient(grasp_action_name, GraspAction)
        self._stop_action_client = actionlib.SimpleActionClient(stop_action_namespace)
        if self._homing_action_client.wait_for_server(rospy.Duration(5)) and self._move_action_client.wait_for_server(rospy.Duration(5)) and self._grasp_action_client.wait_for_server(rospy.Duration(5)) and self._stop_action_client.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("Franka Gripper action clients connected successfully")
        else:
            raise rospy.exceptions.ROSException("Unable to connect to Franka Gripper action servers!")

    def home_gripper(self, wait=True):
        """Sends a homing goal to the action server.
        Returns action success if wait=True, True otherwise.
        """

        if wait:
            self._homing_action_client.send_goal_and_wait(HomingActionGoal())
            return self._homing_action_client.get_result().success
        else:
            self._homing_action_client.send_goal(HomingAction())
            return True

    def move_fingers(self, target_width, target_speed, target_force):
        pass

    def close_gripper(self, target_speed, target_force):
        pass

    def stop_gripper(self):
        pass

    def get_gripper_status(self):
        pass










