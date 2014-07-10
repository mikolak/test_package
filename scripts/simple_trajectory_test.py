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

import PyKDL
import tf_conversions.posemath as pm

def getDownOrientedQuaternion():
  pi = 3.14
  v_x = 1
  v_y = 0
  v_z = 0
  angle = 0.5 * (0.5 * pi)
  _sin = sin(angle)
  x = sin(angle) * v_x
  y = sin(angle) * v_y
  z = sin(angle) * v_z
  w = cos(angle)
  return Quaternion(x, y, z, w)
  
if __name__ == '__main__':
	rospy.init_node('simple_trajectory_test')
  rospy.wait_for_service('/controller_manager/switch_controller')
  conManSwitch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
  
  conManSwitch(['PoseInt'], ['SplineTrajectoryGeneratorJoint'], True)
  
  pose_client = actionlib.SimpleActionClient('/irp6p_arm/pose_trajectory', CartesianTrajectoryAction)
  pose_client.wait_for_server()
  
  print 'Server\' ready'
  
  goal = CartesianTrajectoryGoal()
  
  quaternion = getDownOrientedQuaternion()
  point = Point(0.705438961242, -0.1208864692291, 1.181029263241)
  trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(3.0), Pose(point, quarternion), Twist()))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)
  
  pose_client.send_goal(goal)

  pose_client.wait_for_result()
  command_result = pose_client.get_result()
  
  print 'I\'m done for!'
  
  #conManSwitch([], ['PoseInt'], True)
  
  #tool_client = actionlib.SimpleActionClient('/irp6p_arm/tool_trajectory', CartesianTrajectoryAction)
  #tool_client.wait_for_server()
