#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from abc import ABCMeta, abstractproperty, abstractmethod
from threading import Lock
import rospy
import actionlib

from std_msgs.msg import Header
from std_srvs.srv import Trigger

# Actions for Franka Hand
from franka_gripper.msg import GraspAction, GraspActionGoal, GraspActionResult
from franka_gripper.msg import MoveAction, MoveActionGoal, MoveActionResult
from franka_gripper.msg import StopAction, StopActionGoal, StopActionResult
from franka_gripper.msg import HomingAction, HomingActionGoal, HomingActionResult

# Actions for Robotiq 2F Gripper
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperAction, CommandRobotiqGripperActionGoal, CommandRobotiqGripperActionResult, CommandRobotiqGripperActionFeedback
# from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq

# Service from module_xela_gripper
from xela_2f_force_control.msg import ControllerStatus, LoggerCommand
from xela_2f_force_control.srv import Setpoint, SetpointRequest, SetControllerStatus, SetLoggerCommand

# Standard messages
from std_msgs.msg import Float32

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
    def home_gripper(self, wait):
        pass

    @abstractmethod
    def move_fingers(self, target_width, target_speed, target_force, wait):
        pass

    @abstractmethod
    def close_gripper(self, target_speed, target_force, wait):
        pass

    @abstractmethod
    def open_gripper(self, target_speed, wait):
        pass

    @abstractmethod
    def stop_gripper(self, wait):
        pass

    @abstractmethod
    def grasp_motion(self, target_width, target_speed, target_force, wait):
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

    def clip_pos(self, pos):
        return max(min(pos, self._max_width), self._min_width)

    def clip_speed(self, speed):
        return max(min(speed, self._max_speed), self._min_speed)

    def clip_force(self, force):
        return max(min(force, self._max_speed), self._min_force)

class FrankaHandGripper(GripperInterface):
    """Wrapper class for the Franka Gripper class.

    Manages the gripper via the Franka Gripper Action client.

    Implements the GripperInterface.
    """

    # Gripper parameters. These are not supposed to be changable at runtime
    _gripper_name = "Franka Hand"
    _min_width = 0.0                        # m
    _max_width = 0.09
    _min_speed = 0.01                       # m/s
    _max_speed = 0.1
    _min_force = 10                         # N
    _max_force = 70

    def __init__(self, gripper_action_namespace = "/franka_gripper"):

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
        self._stop_action_client = actionlib.SimpleActionClient(stop_action_namespace, StopAction)
        if self._homing_action_client.wait_for_server(rospy.Duration(5)) and self._move_action_client.wait_for_server(rospy.Duration(5)) and self._grasp_action_client.wait_for_server(rospy.Duration(5)) and self._stop_action_client.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("Franka Gripper action clients connected successfully")
        else:
            raise rospy.exceptions.ROSException("Unable to connect to Franka Gripper action servers!")

    def home_gripper(self, wait=True):
        """Sends a homing goal to the action server.
        Returns action success if wait=True, True otherwise.
        """

        if wait:
            self._homing_action_client.send_goal_and_wait(HomingActionGoal().goal)
            return self._homing_action_client.get_result().success
        else:
            self._homing_action_client.send_goal(HomingAction())
            return True

    def move_fingers(self, target_width, target_speed=_min_speed, target_force=_min_force, wait=True):

        goal = MoveActionGoal()
        goal_time = rospy.Time.now()
        goal.header.stamp = goal_time
        goal.goal_id.stamp = goal_time
        goal.goal.width = target_width
        goal.goal.speed = target_speed

        self._move_action_client.send_goal(goal.goal)

        if wait:
            self._move_action_client.wait_for_result()
            return self._move_action_client.get_result().success
        else:
            return True

    def close_gripper(self, target_speed=_min_speed, target_force=_min_force, wait=True):

        return self.move_fingers(target_width=self._min_width,
                                 target_speed=target_speed,
                                 target_force=self._min_force,
                                 wait=wait)

    def open_gripper(self, target_speed=_max_speed, wait=True):

        return self.move_fingers(target_width=self._max_width,
                                 target_speed=target_speed,
                                 target_force=self._min_force,
                                 wait=wait)

    def stop_gripper(self, wait=True):

        goal = StopActionGoal()
        goal_time = rospy.Time.now()
        goal.header.stamp = goal_time
        goal.goal_id.stamp = goal_time

        self._stop_action_client.send_goal(goal.goal)

        if wait:
            self._stop_action_client.wait_for_result()
            return self._stop_action_client.get_result().success
        else:
            return True

    def grasp_motion(self, target_width=_min_width, target_speed=_min_speed, target_force=_min_force, wait=True):

        goal = GraspActionGoal()
        goal_time = rospy.Time.now()
        goal.header.stamp = goal_time
        goal.goal_id.stamp = goal_time
        goal.goal.width = target_width
        goal.goal.speed = target_speed
        goal.goal.force = target_force
        goal.goal.epsilon.inner = self._inner_epsilon
        goal.goal.epsilon.outer = self._outer_epsilon

        self._grasp_action_client.send_goal(goal.goal)

        if wait:
            self._grasp_action_client.wait_for_result()
            return self._grasp_action_client.get_result().success
        else:
            return True

    def get_gripper_status(self):
        raise NotImplementedError

class Robotiq2FGripper(GripperInterface):
    """Wrapper class for the Robotiq 2F Gripper class.

    Manages the gripper via the action client implemented in https://github.com/hsp-panda/robotiq.
    An implementation that directly interfaces with the serial interface can be used
    via the Robotiq2FingerGripperDriver class in robotiq_2f_gripper_control.robotiq_2f_gripper_driver
    but it needs to be run on the same machine the serial is hooked up to

    Implements the GripperInterface.
    """

    # Gripper parameters. These are not supposed to be changable at runtime
    # TODO config parameter for 2F140 vs 2F85?
    _gripper_name = "Robotiq 2F"
    _min_width = 0.0                        # m
    _max_width = 0.085
    _min_speed = 0.013                      # m/s
    _max_speed = 0.1
    _min_force = 5.0                        # percentage of force (0-100)
    _max_force = 100.0

    def __init__(self, gripper_action_namespace = ""):

        # The gripper works by interfacing with a single action, with different
        # commands
        # TODO check if this is the right default namespace
        server_action_name = gripper_action_namespace + "/command_robotiq_action"

        self._epsilon = 0.002

        # Configure the gripper action client. Raise an exception if something
        # goes wrong
        self._command_feedback = CommandRobotiqGripperActionFeedback()
        self._gripper_action_client = actionlib.SimpleActionClient(server_action_name, CommandRobotiqGripperAction)
        if self._gripper_action_client.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("Robotiq 2F Gripper action client connected successfully")
        else:
            raise rospy.exceptions.ROSException("Unable to connect to Robotiq 2F Gripper action server!")

    def _handle_gripper_command_feedback(self, feedback):

        # Store the last feedback
        self._command_feedback = feedback

    def command_gripper(self, target_width, target_speed, target_force, wait):

        # Basically every movement of the 2F will use this function, so we implement
        # this first

        goal = CommandRobotiqGripperActionGoal()
        goal_time = rospy.Time.now()
        goal.header.stamp = goal_time
        goal.goal_id.stamp = goal_time
        goal.goal.position = self.clip_pos(target_width)
        goal.goal.speed = self.clip_speed(target_speed)
        goal.goal.force = self.clip_force(target_force)

        self._gripper_action_client.send_goal(goal.goal, feedback_cb=self._handle_gripper_command_feedback)

        if wait:
            self._gripper_action_client.wait_for_result(rospy.Duration(20))
            return self._gripper_action_client.get_result()
        else:
            # If the caller is not interested in waiting for result,
            # we return a feedback message
            # TODO: this might not work as intended...
            return self._command_feedback

        # Robotiq.goto(self._gripper_action_client, self.clip_pos(target_width), self.clip_speed(target_speed), self.clip_force(target_force), wait)

    def home_gripper(self, wait=True):

        return self.open_gripper(self._max_speed, wait)

    def move_fingers(self, target_width, target_speed=_min_speed, target_force=_min_force, wait=True):

        # Call the command_gripper method and interpret the result according to
        # wait or not wait
        result = self.command_gripper(target_width, target_speed, target_force, wait)
        if wait:
            # return true if close enough, false otherwise
            return (abs(result.position - result.requested_position) < self._epsilon)
        else:
            # return true if gripper is moving, false otherwise
            return result.is_moving

    def close_gripper(self, target_speed=_min_speed, target_force=_min_force, wait=True):

        # Basic gripper movement. Will return True only if the gripper reaches
        # minimum aperture without encountering resistance
        return self.move_fingers(self._min_width, target_speed, target_force, wait)

    def open_gripper(self, target_speed=_max_speed, wait=True):

        # Basic gripper movement. Will return True only if the gripper reaches
        # maximum aperture without encountering resistance
        return self.move_fingers(self._max_width, target_speed, target_force=self._min_force, wait=wait)

    def stop_gripper(self, wait=True):

        goal = CommandRobotiqGripperActionGoal()
        goal_time = rospy.Time.now()
        goal.header.stamp = goal_time
        goal.goal_id.stamp = goal_time
        goal.goal.stop = True

        self._gripper_action_client.send_goal(goal.goal, feedback_cb=self._handle_gripper_command_feedback)

        if wait:
            self._gripper_action_client.wait_for_result()
            return not self._gripper_action_client.get_result().is_moving
        else:
            return True

    def grasp_motion(self, target_width=_min_width, target_speed=_min_speed, target_force=_min_force, wait=True):

        # Moves at specified width, using a capped force. With no arguments,
        # closes the gripper and stops when some force is perceived.
        result = self.command_gripper(target_width, target_speed, target_force, wait)

        # Action is successful if an object is encountered along the movement,
        # not if width is reached!
        if wait:
            return result.obj_detected
        else:
            return True

    def get_gripper_status(self):
        raise NotImplementedError

class Robotiq2FGripperForceControlled(Robotiq2FGripper):
    """ Wrapper class for the Robotiq 2F Gripper class with an added force
    control loop on top.

    Manages the gripper force control via services implemented in https://github.com/hsp-panda/xela-force-control.
    It is assumed that the `xela-force-control` `control_node` is running.
    It can be executed using e.g. `roslaunch xela-force-control force_control`.

    Manages the gripper opening via the action client implemented in https://github.com/hsp-panda/robotiq.

    Implements the GripperInterface. Mirrors Robotiq2FGripper for the most part.
    Relies on https://github.com/hsp-panda/xela-force-control for the actual force
    control to be enabled.
    """

    # Gripper parameters. These are not supposed to be changable at runtime
    # TODO config parameter for 2F140 vs 2F85?
    _gripper_name = "Robotiq 2F"
    _min_width = 0.0                        # m
    _max_width = 0.085
    _min_speed = 0.013                      # m/s
    _max_speed = 0.1
    _min_force = 5.0                        # percentage of force (0-100)
    _max_force = 100.0

    def __init__(self, gripper_action_namespace = "", force_setpoint_service_namespace = "xela_2f_force_setpoint", force_controller_namespace = "xela_2f_force_controller"):

        # Call the superclass
        super(Robotiq2FGripperForceControlled, self).__init__(gripper_action_namespace)

        # Set service names
        setpoint_service_name = force_setpoint_service_namespace + "/generate"
        controller_status_setter_name = force_controller_namespace + "/set_controller_status"
        controller_logger_setter_name = force_controller_namespace + "/set_logger_command"
        sensor_bias_reset_name = force_controller_namespace + "/xela_sensor_hander/reset_bias"

        # Set up the service proxies
        rospy.wait_for_service(setpoint_service_name)
        self._force_setpoint_set = rospy.ServiceProxy(setpoint_service_name, Setpoint)

        rospy.wait_for_service(controller_status_setter_name)
        self._force_controller_status_set = rospy.ServiceProxy(controller_status_setter_name, SetControllerStatus)

        rospy.wait_for_service(controller_logger_setter_name)
        self._force_controller_logger_set = rospy.ServiceProxy(controller_logger_setter_name, SetLoggerCommand)

        rospy.wait_for_service(sensor_bias_reset_name)
        self._sensor_bias_reset = rospy.ServiceProxy(sensor_bias_reset_name, Trigger)

        # Add callback for the topic exposing the object contact position
        rospy.Subscriber('/xela_2f_force_controller/object_contact_position', Float32, self.object_contact_position_callback)

        # Add callback for the topic exposing the force feedback signal
        rospy.Subscriber('/xela_2f_force_controller/sensor_feedback', Float32, self.force_feedback_callback)

        # Setup a mutex to enforce safe IO within topic callbacks
        self._mutex = Lock()

        # Set a default value for the object contact position
        self._object_contact_position = -1

        # Set a default value for the force feedback
        self._force_feedback = 0.0

    def grasp_motion(self, target_width=_min_width, target_speed=_min_speed, target_force=_min_force, wait=True, duration=10.0):

        # Stop the force controller
        header = Header()
        header.stamp = rospy.Time.now()
        self.controller_service_call(ControllerStatus.CONTROLLER_STATUS_STOPPED)

        # Force the force trajectory to zero
        self.setpoint_service_call(0.0, 1.0)

        # Reset internal object contact position
        with self._mutex:
            self._object_contact_position = -1

        # Reset the sensor bias
        self.sensor_bias_reset_call()

        # Start the object contact detection mechanism
        self.controller_service_call(ControllerStatus.CONTROLLER_STATUS_FIND_CONTACT)

        # Wait for the contact to happen
        object_contact_position = -1
        while not self.get_object_contact_position() > 0:
            rospy.sleep(0.1)

        # Reset the sensor bias before starting the actual force control
        self.sensor_bias_reset_call()

        # Wait as the force controller might open the gripper a bit to release part of the contact pressure
        rospy.sleep(1.0)

        # Fetch the force trajectory parameters (and specify defaults if they do not exist)
        force_target = rospy.get_param('~robotiq2f_force_controlled/force_target', 0.0)
        force_duration = rospy.get_param('~robotiq2f_force_controlled/force_duration', 1.0)
        error_threshold = rospy.get_param('~robotiq2f_force_controlled/error_threshold', 5.0)

        # Start the force controller
        self.controller_service_call(ControllerStatus.CONTROLLER_STATUS_RUNNING)

        # Start the logger
        self.controller_logger_call(LoggerCommand.LOGGER_COMMAND_RUN)

        # Start the force trajectory generator
        self.setpoint_service_call(force_target, force_duration)

        # Wait for the force to be 'near' the target
        # FIXME: Not reliable enough
        # while abs(force_target - self.get_force_feedback()) > error_threshold:
        #     rospy.sleep(0.1)

        # Wait at least for the trajectory to complete
        rospy.sleep(force_duration + 2.0)

        return True

    def open_gripper(self, target_speed=_max_speed, wait=True):

        # Stop the logger and save data
        self.controller_logger_call(LoggerCommand.LOGGER_COMMAND_SAVE)

        # Stop the force controller
        self.controller_service_call(ControllerStatus.CONTROLLER_STATUS_STOPPED)

        # Reset setpoint generator
        self.setpoint_service_call(0.0, 1.0)

        # Open the gripper
        return super(Robotiq2FGripperForceControlled, self).open_gripper()

    def close_gripper(self, target_speed=_min_speed, target_force=_min_force, wait=True):

        # Stop the force controller
        self.controller_service_call(ControllerStatus.CONTROLLER_STATUS_STOPPED)

        rospy.logerr("Closing the gripper completely is disabled within Robotiq2FGripperForceControlled")

        return False

    def controller_service_call(self, controller_status):

        # Make a service call in order to change the force controller status
        try:
            header = Header()
            header.stamp = rospy.Time.now()
            if self._force_controller_status_set(header, ControllerStatus(controller_status)):
                return True
            else:
                rospy.logerr("Cannot call force controller service!")
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Force controller service call failed: %s"%e)
            return False

    def controller_logger_call(self, logger_command):

        # Make a service call in order to send a command to the force controller logger
        try:
            header = Header()
            header.stamp = rospy.Time.now()
            if self._force_controller_logger_set(header, LoggerCommand(logger_command)):
                return True
            else:
                rospy.logerr("Cannot call force controller service!")
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Force controller service call failed: %s"%e)
            return False

    def setpoint_service_call(self, setpoint, duration):

        # Make a service call in order to generate the desired force for the controller
        try:
            if self._force_setpoint_set(setpoint, duration):
                return True
            else:
                rospy.logerr("Cannot call force setpoint service!")
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Force setpoint service call failed: %s"%e)
            return False

    def sensor_bias_reset_call(self):

        # Make a service call in order to reset the sensor bias
        try:
            if self._sensor_bias_reset():
                return True
            else:
                rospy.logerr("Cannot call sensor bias reset service!")
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Sesor bias reset service call failed: %s"%e)
            return False

    def get_object_contact_position(self):

        object_contact_position = -1
        with self._mutex:
            object_contact_position = self._object_contact_position

        return object_contact_position

    def get_force_feedback(self):

        force_feedback = -1
        with self._mutex:
            force_feedback = self._force_feedback

        return force_feedback

    def object_contact_position_callback(self, data):

        with self._mutex:
            self._object_contact_position = data.data


    def force_feedback_callback(self, data):

        with self._mutex:
            self._force_feedback = data.data
