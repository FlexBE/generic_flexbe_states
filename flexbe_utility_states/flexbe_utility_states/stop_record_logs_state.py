#!/usr/bin/env python

from flexbe_core import EventState, Logger
import subprocess
import signal
import os
import rclpy

"""Created on Oct. 17, 2014

@author: Spyros Maniatopoulos
"""

class StopRecordLogsState(EventState):
  """
  A state that records the contents of the specified ROS topics in a bag file.

  ># rosbag_process   subprocess  A system process, whose ID is used to kill it.

  <= stopped            Indicates that a command to kill the process has been issued.

  """
  
  def __init__(self):
    """Constructor"""
    super(StopRecordLogsState, self).__init__(outcomes=['stopped'],
                          input_keys=['rosbag_process'])

  def execute(self, userdata):
    
    return 'stopped'
  
  def on_enter(self, userdata):
    """Upon entering the state, kill the process"""

    try:
      if self._bag_process is not None:
        os.killpg(userdata.rosbag_process.pid, signal.SIGINT)
    except Exception as e:
      Logger.warning("Unable to kill process %s:\n%s" % (str(self.rosbag_process.pid), str(e)))
