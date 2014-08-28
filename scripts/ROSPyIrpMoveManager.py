#!/usr/bin/env python

import rospy
import tf
import actionlib
import math

from controller_manager_msgs.srv import *
from std_msgs.msg import *
from diagnostic_msgs.msg import *
from geometry_msgs.msg import *
from trajectory_msgs.msg import *
from control_msgs.msg import *
from cartesian_trajectory_msgs.msg import *
from force_control_msgs.msg import *
from tf.transformations import *

import PyKDL
import tf_conversions.posemath as pm

class ROSPyIrpMoveManager:
	__lastGenerator = ''
	__robot = ''
	__quaternion = ''
	__conManSwitch = ''
	__jointClient = ''
	__poseClient = ''
	__toolMoveClient = ''
	__toolConfigClient = ''
	#__conManSwitch 
	
	def __init__(self, robotName):
		if robotName != 'ot' and robotName != 'p':
			print('Nieprawidlowa nazwa robota')
			sys.exit()
		self.__robot = robotName
		self.__quaternion = self.getDownOrientedQuaternion()
		self.__lastGenerator = ''
		rospy.wait_for_service('/controller_manager/switch_controller')
		self.__conManSwitch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
		self.setClients()
		self.__forceTransformation = False
		
	def getDownOrientedQuaternion(self):
		real_angle = math.pi * 1	#180 stopni to pionowo w dol
		v_x = 0.0
		v_y = -1
		v_z = 0
		angle = 0.5 * real_angle
		_sin = math.sin(angle)
		x = _sin * v_x
		y = _sin * v_y
		z = _sin * v_z
		w = math.cos(angle)
		return Quaternion(x, y, z, w)

	def setClients(self):
		if self.__robot == 'ot':
			self.__jointClient = actionlib.SimpleActionClient('/irp6ot_arm/spline_trajectory_action_joint', FollowJointTrajectoryAction)
			self.__poseClient = actionlib.SimpleActionClient('/irp6ot_arm/pose_trajectory', CartesianTrajectoryAction)
			self.__toolMoveClient = actionlib.SimpleActionClient('/irp6ot_tfg/spline_trajectory_action_motor', FollowJointTrajectoryAction)
			self.__toolConfigClient = actionlib.SimpleActionClient('/irp6ot_arm/tool_trajectory', CartesianTrajectoryAction)
		elif self.__robot == 'p':
			self.__jointClient = actionlib.SimpleActionClient('/irp6p_arm/spline_trajectory_action_joint', FollowJointTrajectoryAction)
			self.__poseClient = actionlib.SimpleActionClient('/irp6p_arm/pose_trajectory', CartesianTrajectoryAction)
			self.__toolMoveClient = actionlib.SimpleActionClient('/irp6p_tfg/spline_trajectory_action_motor', FollowJointTrajectoryAction)
			self.__toolConfigClient = actionlib.SimpleActionClient('/irp6p_arm/tool_trajectory', CartesianTrajectoryAction)
			
	def setGenerator(self, generator):
		if generator == '':
			self.__conManSwitch([], [self.__lastGenerator], True)
			return
			
		if self.__robot == 'ot':
			if generator == 'xyz':
				gen = 'Irp6otmPoseInt'
			elif generator == 'joint':
				gen = 'Irp6otmSplineTrajectoryGeneratorJoint'
			elif generator == 'tool move':
				gen = 'Irp6ottfgSplineTrajectoryGeneratorMotor'
		elif self.__robot == 'p':
			if generator == 'xyz':
				gen = 'Irp6pmPoseInt'
			elif generator == 'joint':
				gen = 'Irp6pmSplineTrajectoryGeneratorJoint'
			elif generator == 'tool move':
				gen = 'Irp6ptfgSplineTrajectoryGeneratorMotor'
				
		if gen != self.__lastGenerator:		
			self.__conManSwitch([gen], [self.__lastGenerator], True)
			self.__lastGenerator = gen
		else:
			self.__conManSwitch([gen], [], True)
		
			
	def jointMove(self, point, duration):
		self.setGenerator('joint')
		self.__jointClient.wait_for_server()

		print 'Joint trajectory pending...'

		goal = FollowJointTrajectoryGoal()
		if self.__robot == 'ot':
			goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
		elif self.__robot == 'p':
			goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
			
		goal.trajectory.points.append(JointTrajectoryPoint(point, [], [], [], rospy.Duration(duration)))
		goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		self.__jointClient.send_goal(goal)

		self.__jointClient.wait_for_result()
		
		commandResult = self.__jointClient.get_result()
	
	def xyzMove(self, point, duration, stopOnForceDetected, zForce=0, sleepDur=1.0):
		self.setGenerator('xyz')
		self.__poseClient.wait_for_server()
	
		print 'Euclidean trajectory pending...'
		
		goal = CartesianTrajectoryGoal()
		
		if stopOnForceDetected == True:
			print '...with a stop on detected force...'
			self.switchForceTransformation(True)
			rospy.sleep(sleepDur)
			goal.wrench_constraint.force.x = 0
			goal.wrench_constraint.force.y = 0
			goal.wrench_constraint.force.z = zForce
			
		goal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(duration), Pose(point, self.__quaternion), Twist()))
		goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)
		
		self.__poseClient.send_goal(goal)

		self.__poseClient.wait_for_result()
		command_result = self.__poseClient.get_result()
		
		if stopOnForceDetected == True:
			self.switchForceTransformation(False)
		
	def toolMove(self, point, duration):
		self.setGenerator('tool move')
		self.__toolMoveClient.wait_for_server()

		print 'Tool move trajectory pending...'

		goal = FollowJointTrajectoryGoal()
		goal.trajectory.joint_names = ['joint1']
		goal.trajectory.points.append(JointTrajectoryPoint(point, [0.0], [], [], rospy.Duration(duration)))
		goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

		self.__toolMoveClient.send_goal(goal)

		self.__toolMoveClient.wait_for_result()
		command_result = self.__toolMoveClient.get_result()
		
	def toolConfig(self, point):
		self.setGenerator('')
	
		self.__toolConfigClient.wait_for_server()
  
		print 'Tool configuration changing...'
		 
		goal = CartesianTrajectoryGoal()
	  
		goal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(0.0), Pose(point, Quaternion(0.0, 0.0, 0.0, 1.0)), Twist()))
		goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.1)
	  
		self.__toolConfigClient.send_goal(goal)

		self.__toolConfigClient.wait_for_result()
		command_result = self.__toolConfigClient.get_result()
		
	def switchForceTransformation(self, flip):
		print 'Switching ForceTransformation for ' + self.__robot
		if self.__robot == 'ot':
			switched = 'Irp6otmForceTransformation'
		elif self.__robot == 'p':
			switched = 'Irp6pmForceTransformation'
		else:
			print 'ERROR! Not a proper robot name!'
			sys.exit()
		
		if self.__forceTransformation == flip:
			print 'to the same state'
			return	
		elif self.__forceTransformation == False:
			print 'on'
			self.__forceTransformation = True
			self.__conManSwitch([switched], [], True)
		else:
			print 'off'
			self.__forceTransformation = False
			self.__conManSwitch([], [switched], True)

	def finish(self):
		print 'Clearing...'
		self.__conManSwitch([], [self.__lastGenerator], True)
	
