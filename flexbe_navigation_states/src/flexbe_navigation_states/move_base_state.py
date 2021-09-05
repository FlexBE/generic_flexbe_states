#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import rospy

"""
Created on 11/19/2015

@author: Spyros Maniatopoulos

Updated on 09/04/2021
@github/tbazina
"""

class MoveBaseState(EventState):
    """
    Navigates a robot to a desired position and orientation using move_base.

    -- action_topic str                 move_base action topic, (/move_base)
    ># waypoint     PoseStamped         Target waypoint for navigation.

    <= arrived                          Navigation to target pose succeeded.
    <= failed                           Navigation to target pose failed.
    <= preempted                        Navigation to target pose preempted.
    """

    def __init__(self, action_topic):
        """Constructor"""

        super(MoveBaseState, self).__init__(
            outcomes = ['arrived', 'failed', 'preempted'],
            input_keys = ['waypoint']
            )

        self._action_topic = action_topic

        self._client = ProxyActionClient({self._action_topic: MoveBaseAction})

        self._arrived = False
        self._failed = False
        self._preempted = False


    def execute(self, userdata):
        """Wait for action result and return outcome accordingly"""

        if self._arrived:
            return 'arrived'
        if self._failed:
            return 'failed'
        if self._preempted:
            return 'preempted'

        if self._client.has_result(self._action_topic):
            status = self._client.get_state(self._action_topic)
            if status == GoalStatus.SUCCEEDED:
                self._arrived = True
                return 'arrived'
            elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED,
                            GoalStatus.RECALLED, GoalStatus.ABORTED]:
                Logger.logwarn('Navigation failed: %s' % str(status))
                self._failed = True
                return 'failed'
            elif status == GoalStatus.PREEMPTED:
                Logger.logwarn('Navigation preempted: %s' % str(status))
                self._preempted = True
                return 'preempted'



    def on_enter(self, userdata):
        """Create and send action goal"""

        self._arrived = False
        self._failed = False
        self._preempted = False

        # Create and populate action goal
        goal = MoveBaseGoal()

        goal.target_pose = userdata.waypoint
        goal.target_pose.header.stamp = rospy.Time.now()

        # Send the action goal for execution
        try:
            self._client.send_goal(self._action_topic, goal)
        except Exception as e:
            Logger.logwarn("Unable to send navigation action goal:\n%s" % str(e))
            self._failed = True

    def cancel_active_goals(self):
        if self._client.is_available(self._action_topic):
            if self._client.is_active(self._action_topic):
                if not self._client.has_result(self._action_topic):
                    self._client.cancel(self._action_topic)
                    Logger.loginfo('Cancelled move_base active action goal.')

    def on_exit(self, userdata):
        self.cancel_active_goals()

    def on_stop(self):
        self.cancel_active_goals()
