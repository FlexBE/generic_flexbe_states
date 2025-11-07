#!/usr/bin/env python

import rospy
import os
import xml.etree.ElementTree as ET
from rospkg import RosPack
from flexbe_core import EventState, Logger

'''
Created on 18.06.2016

@author: Alberto Romay
'''

class GetJointsFromSrdfGroup(EventState):
	'''
	Simple state to look up a pre-defined list of joint names from a given
	joint group defined in a SRDF file.
	This state is recommended if you only need these values without any unnecessary overhead.

	-- move_group 	string 		Name of the move group of interest.
								e.g., "my_moveit_config/config/my_robot.srdf"
	-- robot_name 	string 		Optional name of the robot to be used.
								If left empty, the first one found will be used
								(only required if multiple robots are specified in the same file).
	-- srdf_name	string		Optional name of the srdf parameter
								(default: "/robot_description_semantic")

	#> joint_names string[] 	List of joint values for the requested group.

	<= retrieved 				Joint values are available.
	<= param_error 				Something went wrong when accessing the SRDF file.

	'''

	def __init__(self, move_group, robot_name="", srdf_name = "/robot_description_semantic"):
		'''
		Constructor
		'''
		super(GetJointsFromSrdfGroup, self).__init__(outcomes=['retrieved', 'param_error'],
														output_keys=['joint_names'])

		self._move_group = move_group
		self._robot_name = robot_name
		self._srdf_name  = srdf_name

		self._srdf_param = None
		if rospy.has_param(self._srdf_name):
			self._srdf_param = rospy.get_param(self._srdf_name)
		else:
			Logger.logerr('Unable to get parameter: ' + self._srdf_name)

		self._file_error = False
		self._srdf = None
		self._return_code = None

	def execute(self, userdata):

		if (self._return_code is not None):
			# Handle blocked transition or error during on_enter
			return self._return_code

		robot = None
		for r in self._srdf.iter('robot'):
			if self._robot_name == '' or self._robot_name == r.attrib['name']:
				robot = r
				break
		if robot is None:
			Logger.logwarn('Did not find robot name in SRDF: %s' % self._robot_name)
			self._return_code = 'param_error'
			return 'param_error'

		group = None
		for g in robot.iter('group'):
			if g.attrib['name'] == self._move_group:
				group = g
				break
		if group is None:
			Logger.logwarn('Did not find group name in SRDF: %s' % self._move_group)
			self._return_code = 'param_error'
			return 'param_error'

		try:
			userdata.joint_names = [str(j.attrib['name']) for j in group.iter('joint')]
		except Exception as e:
			Logger.logwarn('Unable to parse joint values from SRDF:\n%s' % str(e))
			self._return_code = 'param_error'
			return 'param_error'

		self._return_code = 'retrieved'
		return 'retrieved'


	def on_enter(self, userdata):
		# Check existence of SRDF parameter.
		# Values are read during runtime to allow modifications.
		if rospy.has_param(self._srdf_name):
			self._srdf_param = rospy.get_param(self._srdf_name)
		else:
			Logger.logerror("Unable to get parameter: "+self._srdf_name)
			self._return_code = 'param_error'
			return

		# Parse the SRDF string
		try:
			self._srdf = ET.fromstring(self._srdf_param)
		except Exception as e:
			Logger.logwarn('Unable to parse given SRDF parameter: /robot_description_semantic')
			self._return_code = 'param_error'
