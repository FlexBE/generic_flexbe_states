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
Simple state to look up a pre-defined joint configuration from the given SRDF file.

Created on 18.06.2016

@author: Philipp Schillinger
"""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from flexbe_core import EventState, Logger


class GetJointsFromSrdfState(EventState):
    """
    Simple state to look up a pre-defined joint configuration from the given SRDF file.
    This state is recommended if you only need these values without any unnecessary overhead.

    -- config_name     string         Name of the joint configuration of interest.
    -- srdf_file    string        Package-relative path to the SRDF file,
                                e.g., "my_moveit_config/config/my_robot.srdf"
    -- move_group     string         Optional move group name to which the config refers.
                                If left empty, the first one found will be used
                                (only required if several groups use the same configs).
    -- robot_name     string         Optional name of the robot to be used.
                                If left empty, the first one found will be used
                                (only required if multiple robots are specified in the same file).

    #> joint_values float[]     List of joint values for the requested config.

    <= retrieved                 Joint values are available.
    <= file_error                 Something went wrong when accessing the SRDF file.

    """

    def __init__(self, config_name, srdf_file, move_group="", robot_name=""):
        """
        Constructor
        """
        super().__init__(outcomes=['retrieved', 'file_error'],
                         output_keys=['joint_values'])

        self._config_name = config_name
        self._move_group = move_group
        self._robot_name = robot_name

        # Check existence of SRDF file to reduce risk of runtime failure.
        # Anyways, values will only be read during runtime to allow modifications.
        self._srdf_path = None
        try:
            pkg_name = srdf_file.split('/')[0]
            self._srdf_path = os.path.join(get_package_share_directory(pkg_name), '/'.join(srdf_file.split('/')[1:]))
            if not os.path.isfile(self._srdf_path):
                raise IOError(f"File '{self._srdf_path}' does not exist!")
        except Exception as exc:  # pylint: disable=W0703
            Logger.logwarn(f"Unable to find given SRDF file: {srdf_file}\n  {str(exc)}")

        self._file_error = False
        self._srdf = None

    def execute(self, userdata):
        if self._file_error:
            return 'file_error'

        robot = None
        for rbt in self._srdf.iter('robot'):
            if self._robot_name in ('', rbt.attrib['name']):
                robot = rbt
                break
        if robot is None:
            Logger.logwarn(f"Did not find robot name in SRDF: {self._robot_name}")
            self._file_error = True
            return 'file_error'

        config = None
        for cgs in robot.iter('group_state'):
            if self._move_group in ('', cgs.attrib['group']) and cgs.attrib['name'] == self._config_name:
                config = cgs
                break
        if config is None:
            Logger.logwarn(f"Did not find config name in SRDF: {self._config_name}")
            self._file_error = True
            return 'file_error'

        try:
            userdata.joint_values = [float(jnt.attrib['value']) for jnt in config.iter('joint')]
        except Exception as exc:  # pylint: disable=W0703
            Logger.logwarn(f"Unable to parse joint values from SRDF:\n{str(exc)}")
            self._file_error = True
            return 'file_error'

        return 'retrieved'

    def on_enter(self, userdata):
        self._file_error = False
        if self._srdf_path is None:
            self._file_error = True
            return

        try:
            self._srdf = ET.parse(self._srdf_path).getroot()
        except Exception as exc:  # pylint: disable=W0703
            Logger.logwarn(f"Unable to parse given SRDF file: {self._srdf_path}\n  {str(exc)}")
            self._file_error = True
