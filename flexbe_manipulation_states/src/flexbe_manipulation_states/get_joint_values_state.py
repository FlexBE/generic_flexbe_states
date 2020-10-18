#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

from sensor_msgs.msg import JointState

'''
Created on 06.03.2016

@author: Philipp Schillinger
'''

class GetJointValuesState(EventState):
	'''
	Retrieves current values of specified joints.
	Joints are specified by parameter list.

	-- joints				string[]	List of desired joint names.
	-- timeout				double		Timeout value (optional)
	-- joint_states_topic	string		Optional name of joint states topic
										(default: /joint_states)

	#> joint_values float[] 	List of current joint values.

	<= retrieved 				Joint values are available.
	<= timeout	 				Joint values are not available.

	'''

	def __init__(self, joints, timeout=None, joint_states_topic='/joint_states'):
		'''
		Constructor
		'''
		super(GetJointValuesState, self).__init__(outcomes=['retrieved', 'timeout'],
													output_keys=['joint_values'])

		self._topic = joint_states_topic
		self._sub = ProxySubscriberCached({self._topic: JointState})

		self._joints = joints
		self._joint_values = list()
		self._timeout = timeout


	def execute(self, userdata):
		if (self._return_code is not None):
			# Handle blocked transition or error during on_enter
			return self._return_code

		while self._sub.has_buffered(self._topic):
			msg = self._sub.get_from_buffer(self._topic)
			for i in range(len(msg.name)):
				if msg.name[i] in self._joints \
				and self._joint_values[self._joints.index(msg.name[i])] is None:
					self._joint_values[self._joints.index(msg.name[i])] = msg.position[i]

		if all(v is not None for v in self._joint_values):
			userdata.joint_values = self._joint_values
			self._return_code = 'retrieved'
			return 'retrieved'

		if (self._timeout is not None and \
			(Time.now()-self._start_time) > rospy.Duration(self._timeout) ):
			self._return_code = 'timeout'
			return 'timeout'

	def on_enter(self, userdata):
		self._sub.enable_buffer(self._topic)
		self._joint_values = [None] * len(self._joints)
		self._return_code = None
		self._start_time = rospy.Time.now()
		userdata.joint_values = None

	def on_exit(self, userdata):
		self._sub.disable_buffer(self._topic)
