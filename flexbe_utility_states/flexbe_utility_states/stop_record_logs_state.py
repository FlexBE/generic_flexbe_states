#!/usr/bin/env python

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
A state that records the contents of the specified ROS topics in a bag file.

Created on Oct. 17, 2014

@author: Spyros Maniatopoulos
"""

import os
import signal

from flexbe_core import EventState, Logger


class StopRecordLogsState(EventState):
    """
    A state that records the contents of the specified ROS topics in a bag file.

    ># rosbag_process     subprocess    A system process, whose ID is used to kill it.

    <= stopped                       Indicates that a command to kill the process has been issued.
    <= failed                        Indicates that a command to kill the process failed.

    """

    def __init__(self):
        """Constructor"""
        super().__init__(outcomes=['stopped', 'failed'],
                         input_keys=['rosbag_process'])
        self._bag_process = None

    def execute(self, userdata):
        if self._bag_process is None:
            return 'stopped'

        return 'failed'

    def on_enter(self, userdata):
        """Upon entering the state, kill the process"""
        self._bag_process = None
        try:
            self._bag_process = userdata.rosbag_process
            if self._bag_process is not None:
                try:
                    os.killpg(userdata.rosbag_process.pid, signal.SIGINT)
                    self._bag_process = None
                except Exception as exc:  # pylint: disable=W0703
                    Logger.warning(f"Unable to kill process {str(self._bag_process.pid)}\n  {str(exc)}")
        except Exception as exc:  # pylint: disable=W0703
            Logger.warning(f"No bagging process userdata defined!\n  {str(exc)}")
            self._bag_process = 'undefined'  # Force failed state
