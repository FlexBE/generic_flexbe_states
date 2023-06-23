#!/usr/bin/env python3

# Copyright 2023 Philipp Schillinger,  Christopher Newport University
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Philipp Schillinger,  Christopher Newport University nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""
State to execute with MoveIt! a known trajectory defined in the "/robot_description_semantic"
parameter (SRDF file) BEWARE! This state performs no self-/collison planning!

Created on 10.10.2016

@author: Alberto Romay
"""

import xml.etree.ElementTree as ET

import rclpy
from rclpy.exceptions import ParameterNotDeclaredException

from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv._execute_known_trajectory import ExecuteKnownTrajectory, ExecuteKnownTrajectory_Request

from trajectory_msgs.msg import JointTrajectoryPoint

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyServiceCaller


class SrdfStateToMoveitExecute(EventState):
    """
        State to execute with MoveIt!
        a known trajectory defined in the "/robot_description_semantic" parameter (SRDF file) BEWARE!
        This state performs no self-/collison planning!

        -- config_name            string            Name of the joint configuration of interest.

        -- move_group             string            Name of the move group to be used for planning.

        -- duration             float               Duration of the execution
                                                            Default to 1 second

        -- action_topic         string              Topic on which MoveIt is listening for action calls.
                                                            Defualt to: /execute_kinematic_path

        -- robot_name             string            Optional name of the robot to be used.
                                                        If left empty, the first one found will be used
                                                          (only required if multiple robots are specified
                                                           in the same file).

        ># joint_config         float[]             Target configuration of the joints.
                                                            Same order as their corresponding
                                                                names in joint_names.

        <= reached                                  Target joint configuration has been reached.
        <= request_failed                           Failed to request the service.
        <= moveit_failed                            Failed to execute the known trajectory.

        """

    def __init__(self, config_name, move_group="", duration=1.0, wait_for_execution=True,
                 action_topic='/execute_kinematic_path', robot_name=""):
        """
                Constructor
                """
        super().__init__(outcomes=['reached', 'request_failed', 'moveit_failed', 'param_error'])

        self._config_name = config_name
        self._robot_name = robot_name
        self._move_group = move_group
        self._duration = duration
        self._wait_for_execution = wait_for_execution
        self._action_topic = action_topic
        self._client = ProxyServiceCaller({self._action_topic: ExecuteKnownTrajectory})

        self._request_failed = False
        self._moveit_failed = False
        self._success = False

        self._srdf_param = None
        try:
            self._srdf_param = self._node.get_parameter('/robot_description_semantic')
        except ParameterNotDeclaredException:
            Logger.logerr('Unable to get parameter: /robot_description_semantic')

        self._param_error = False
        self._srdf = None
        self._response = None
        self._joint_config = None
        self._joint_names = None

    def execute(self, userdata):
        """
        Execute this state
        """
        if self._param_error:
            return 'param_error'

        if self._request_failed:
            return 'request_failed'

        if self._response.error_code.val != MoveItErrorCodes.SUCCESS:
            Logger.logwarn(f"Move action failed with result error code: {str(self._response.error_code)}")
            self._moveit_failed = True
            return 'moveit_failed'

        Logger.loginfo(f"Move action succeeded: {str(self._response.error_code)}")
        self._success = True
        return 'reached'

    def on_enter(self, userdata):
        self._param_error = False
        self._request_failed = False
        self._moveit_failed = False
        self._success = False

        # Parameter check
        if self._srdf_param is None:
            self._param_error = True
            return

        try:
            self._srdf = ET.fromstring(self._srdf_param)
        except Exception:  # pylint: disable=W0703
            Logger.logwarn('Unable to parse given SRDF parameter: /robot_description_semantic')
            self._param_error = True
            return

        robot = None
        for rbt in self._srdf.iter('robot'):
            if self._robot_name == ('', rbt.attrib['name']):
                robot = rbt
                break

        if robot is None:
            Logger.logwarn(f"Did not find robot name in SRDF: {self._robot_name}")
            self._param_error = True
            return

        config = None
        for cgs in robot.iter('group_state'):
            if self._move_group in ('', cgs.attrib['group']) and cgs.attrib['name'] == self._config_name:
                config = cgs
                self._move_group = cgs.attrib['group']  # Set move group name in case it was not defined
                break

        if config is None:
            Logger.logwarn(f"Did not find config name in SRDF: {self._config_name}")
            self._param_error = True
            return

        try:
            self._joint_config = [float(j.attrib['value']) for j in config.iter('joint')]
            self._joint_names = [str(j.attrib['name']) for j in config.iter('joint')]
        except Exception as exc:  # pylint: disable=W0703
            Logger.logwarn(f"Unable to parse joint values from SRDF:\n{str(exc)}")
            self._param_error = True
            return

        # Action Initialization
        action_goal = ExecuteKnownTrajectory_Request()
        # action_goal.trajectory.joint_trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.3)
        action_goal.trajectory.joint_trajectory.joint_names = self._joint_names
        action_goal.trajectory.joint_trajectory.points = [JointTrajectoryPoint()]
        action_goal.trajectory.joint_trajectory.points[0].time_from_start = rclpy.duration.Duration(seconds=self._duration)
        action_goal.wait_for_execution = self._wait_for_execution

        action_goal.trajectory.joint_trajectory.points[0].positions = self._joint_config

        try:
            self._response = self._client.call(self._action_topic, action_goal)
            Logger.loginfo(f"Execute Known Trajectory Service requested:\n  {str(self._action_topic)}")
        except Exception as exc:  # pylint: disable=W0703
            Logger.logwarn(f"Execute Known Trajectory Service did not process request: \n  {type(exc)} - {str(exc)}")
            self._request_failed = True
