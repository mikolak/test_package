#!/usr/bin/env python

import rospy
import tf
import actionlib

from controller_manager_msgs.srv import *
from std_msgs.msg import *
from diagnostic_msgs.msg import *
from geometry_msgs.msg import *
from trajectory_msgs.msg import *
from control_msgs.msg import *
from cartesian_trajectory_msgs.msg import *
from force_control_msgs.msg import *
from tf.transformations import *

#from PyKDL import *
import PyKDL
import tf_conversions.posemath as pm

if __name__ == '__main__':
	rospy.init_node('multi_trajectory')
	rospy.wait_for_service('/controller_manager/switch_controller')
	conmanSwitch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
	
	#
	# Motor coordinates motion
	#
	
	conmanSwitch(['Irp6ptfgSplineTrajectoryGeneratorMotor'], [], True)
	
	motor_client = actionlib.SimpleActionClient('/irp6p_tfg/spline_trajectory_action_motor', FollowJointTrajectoryAction)
	motor_client.wait_for_server()

	print 'AAAAAAAAAAAAAa'

	goal = FollowJointTrajectoryGoal()
	goal.trajectory.joint_names = ['joint1']
	goal.trajectory.points.append(JointTrajectoryPoint([-1000.0], [0.0], [], [], rospy.Duration(10.0)))
	goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

	motor_client.send_goal(goal)

	motor_client.wait_for_result()
	command_result = motor_client.get_result()
		
	conmanSwitch([], ['Irp6ptfgSplineTrajectoryGeneratorMotor'], True)	
	
	conmanSwitch(['Irp6ptfgSplineTrajectoryGeneratorMotor'], [], True)
	
	motor_client = actionlib.SimpleActionClient('/irp6p_tfg/spline_trajectory_action_motor', FollowJointTrajectoryAction)
	motor_client.wait_for_server()

	print 'BBBBEEEE'

	goal = FollowJointTrajectoryGoal()
	goal.trajectory.joint_names = ['joint1']
	goal.trajectory.points.append(JointTrajectoryPoint([1000.0], [0.0], [], [], rospy.Duration(10.0)))
	goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

	motor_client.send_goal(goal)

	motor_client.wait_for_result()
	command_result = motor_client.get_result()
		
	conmanSwitch([], ['Irp6ptfgSplineTrajectoryGeneratorMotor'], True)	
