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
Simple state to look up a pre-defined joint configuration from the given joint group in a SRDF file.

Created on 18.06.2016

@author: Alberto Romay
"""

import xml.etree.ElementTree as ET

from rclpy.exceptions import ParameterNotDeclaredException

from flexbe_core import EventState, Logger


class GetJointsFromSrdfGroup(EventState):
    """
    Simple state to look up a pre-defined joint configuration from the given joint group in a SRDF file.
    This state is recommended if you only need these values without any unnecessary overhead.

    -- move_group     string         Name of the move group of interest.
                                e.g., "my_moveit_config/config/my_robot.srdf"
    -- robot_name     string         Optional name of the robot to be used.
                                If left empty, the first one found will be used
                                (only required if multiple robots are specified in the same file).

    #> joint_names string[]     List of joint values for the requested group.

    <= retrieved                 Joint values are available.
    <= param_error                 Something went wrong when accessing the SRDF file.

    """

    def __init__(self, move_group, robot_name=""):
        """
        Constructor
        """
        super().__init__(outcomes=['retrieved', 'param_error'],
                         output_keys=['joint_names'])

        self._move_group = move_group
        self._robot_name = robot_name

        # Check existence of SRDF parameter.
        # Values will only be read during runtime to allow modifications.
        self._srdf_param = None
        try:
            self._srdf_param = self._node.get_parameter('/robot_description_semantic')
        except ParameterNotDeclaredException:
            Logger.logerr('Unable to get parameter: /robot_description_semantic')

        self._param_error = False
        self._file_error = False
        self._srdf = None

    def execute(self, userdata):
        """ execute the state """

        if self._param_error:
            return 'param_error'
        robot = None
        for rbt in self._srdf.iter('robot'):
            if self._robot_name in ('', rbt.attrib['name']):
                robot = rbt
                break
        if robot is None:
            Logger.logwarn(f'Did not find robot name in SRDF: {self._robot_name}')
            self._param_error = True
            return 'param_error'

        group = None
        for grp in robot.iter('group'):
            if grp.attrib['name'] == self._move_group:
                group = grp
                break

        if group is None:
            Logger.logwarn(f'Did not find group name in SRDF: {self._move_group}')
            self._param_error = True
            return 'param_error'

        try:
            userdata.joint_names = [str(j.attrib['name']) for j in group.iter('joint')]
        except Exception as exc:  # pylint: disable=W0703
            Logger.logwarn(f'Unable to parse joint values from SRDF:\n{str(exc)}')
            self._param_error = True
            return 'param_error'

        return 'retrieved'

    def on_enter(self, userdata):
        # Parameter check
        if self._srdf_param is None:
            self._param_error = True
            return

        try:
            self._srdf = ET.fromstring(self._srdf_param)
        except Exception as exc:  # pylint: disable=W0703
            Logger.logwarn(f'Unable to parse given SRDF parameter: /robot_description_semantic\n{exc}')
            self._param_error = True
