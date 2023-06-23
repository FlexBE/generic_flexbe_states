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
Retrieves current values of specified joints.
Created on 06.03.2016

@author: Philipp Schillinger
"""

from flexbe_core import EventState
from flexbe_core.proxy import ProxySubscriberCached

from sensor_msgs.msg import JointState


class GetJointValuesState(EventState):
    """
    Retrieves current values of specified joints.

    -- joints        string[]    List of desired joint names.

    #> joint_values float[]     List of current joint values.

    <= retrieved                 Joint values are available.

    """

    def __init__(self, joints):
        """
        Constructor
        """
        super().__init__(outcomes=['retrieved'],
                         output_keys=['joint_values'])

        self._topic = '/joint_states'
        self._sub = ProxySubscriberCached({self._topic: JointState}, inst_id=id(self))

        self._joints = joints
        self._joint_values = []

    def execute(self, userdata):
        while self._sub.has_buffered(self._topic):
            msg = self._sub.get_from_buffer(self._topic)
            for i, jnt_name in enumerate(msg.name):
                if jnt_name in self._joints and self._joint_values[self._joints.index(jnt_name)] is None:
                    self._joint_values[self._joints.index(jnt_name)] = msg.position[i]

        if all(v is not None for v in self._joint_values):
            userdata.joint_values = self._joint_values
            return 'retrieved'

    def on_enter(self, userdata):
        self._sub.enable_buffer(self._topic)
        self._joint_values = [None] * len(self._joints)

    def on_exit(self, userdata):
        self._sub.disable_buffer(self._topic)
