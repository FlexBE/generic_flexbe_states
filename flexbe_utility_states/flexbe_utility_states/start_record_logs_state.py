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
A state that records the contents of the specified ROS topics in a bag file.
Logging is done by creating a rosbag subprocess which is afterwards accessible using the output key rosbag_process.
This state is typically combined with a StopRecordLogsState which gets the subprocess in order to stop logging.

Created on Oct. 17, 2014

@author: Spyros Maniatopoulos
"""

import os
import signal
import subprocess

from flexbe_core import EventState, Logger


class StartRecordLogsState(EventState):
    """
    A state that records the contents of the specified ROS topics in a bag file.
    Logging is done by creating a rosbag subprocess which is afterwards accessible using the output key rosbag_process.
    This state is typically combined with a StopRecordLogsState which gets the subprocess in order to stop logging.

    -- topics_to_record string[]    A list of topics (strings) that this state will log in a bagfile (.bag) using rosbag record.
                                    They are usually specified in a config (.yaml) file, which is added as a behavior parameter.

    ># bagfile_name        string        Full path of the bagfile to be created by this logging.

    #> rosbag_process     subprocess    The process that is executing rosbag record.
                                    The ID of this subprocess object can be used to kill it later.

    <= logging                        Indicates that rosbag record was started.

    """

    def __init__(self, topics_to_record):
        """Constructor"""
        super().__init__(outcomes=['logging'],
                         input_keys=['bagfile_name'],
                         output_keys=['rosbag_process'])

        self._topics_to_record = topics_to_record
        self._bag_process = None

    def execute(self, userdata):
        """Execute this state"""

        # State has already started recording upon enter
        return 'logging'

    def on_enter(self, userdata):
        """Upon entering the state"""

        bash_command = ["/bin/bash", "--norc", "-c"]

        # Start Bagging
        bag_command = f"rosbag record -O {userdata.bagfile_name} {self._topics_to_record.replace(',', ' ')}"
        self._bag_process = subprocess.Popen(bash_command + [bag_command],
                                             stdout=subprocess.PIPE,
                                             preexec_fn=os.setsid)
        userdata.rosbag_process = self._bag_process
        Logger.loginfo('Recording topics to %s' % userdata.bagfile_name)

    def on_stop(self):
        """Kill any rosbag record processes when behavior execution is stopped"""
        try:
            if self._bag_process is not None:
                os.killpg(self._bag_process.pid, signal.SIGINT)
        except Exception as e:
            Logger.warning("Unable to kill process %s:\n%s" % (str(self._bag_process.pid), str(e)))
