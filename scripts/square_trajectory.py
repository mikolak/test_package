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

def getDownOrientedQuaternion():
  real_angle = math.pi * 1	#180 stopni to pionowo w dol
  v_x = 0.7
  v_y = 0.7
  v_z = 0
  angle = 0.5 * real_angle
  _sin = math.sin(angle)
  x = _sin * v_x
  y = _sin * v_y
  z = _sin * v_z
  w = math.cos(angle)
  return Quaternion(x, y, z, w)
  
if __name__ == '__main__':
  rospy.init_node('simple_trajectory_test')
  rospy.wait_for_service('/controller_manager/switch_controller')
  conManSwitch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
  
  #------------------------------------------------
  # Stawy
  #------------------------------------------------
  conManSwitch(['Irp6pmSplineTrajectoryGeneratorJoint'], [], True)
    
  client = actionlib.SimpleActionClient('/irp6p_arm/spline_trajectory_action_joint', FollowJointTrajectoryAction)
  client.wait_for_server()

  print 'Inicjacja postawy'

  goal = FollowJointTrajectoryGoal()
  goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
  #goal.trajectory.points.append(JointTrajectoryPoint([-0.10087151336609543, -1.5417429815634993, 0.019743230015841898, 1.1331041783656084, 3.658011557435151, -2.7351279214366393], [], [], [], rospy.Duration(4.0)))
  #goal.trajectory.points.append(JointTrajectoryPoint([-0.10087151336609543, -1.5417429815634993, 0.019743230015841898, 0, -1.57, 0], [], [], [], rospy.Duration(6.0)))
  # 1.5 * pi
  goal.trajectory.points.append(JointTrajectoryPoint([0, -0.5 * math.pi, 0, 0, 1.5 * math.pi, -0.5 * math.pi], [], [], [], rospy.Duration(6.0)))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

  client.send_goal(goal)

  client.wait_for_result()
  command_result = client.get_result()
  
  rospy.sleep(5.0)
  
  #--------------------------------------------------
  # Kartezjanski
  #---------------------------------------------------
  
  # WAZNE ZMIENNE OD POZYCJI
  #======================================================================
  zero_point = 0.85
  x_diff = 0.10
  height = 1.20
  sleep_duration = 1.0
  #=======================================================================
  conManSwitch(['Irp6pmPoseInt'], ['Irp6pmSplineTrajectoryGeneratorJoint'], True)
  
  pose_client = actionlib.SimpleActionClient('/irp6p_arm/pose_trajectory', CartesianTrajectoryAction)
  pose_client.wait_for_server()
  
  print 'Ustawienie pozycji poczatkowej'
  
  goal = CartesianTrajectoryGoal()
  
  quaternion = getDownOrientedQuaternion()
  point = Point(zero_point, 0, height)
  goal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(15.0), Pose(point, quaternion), Twist()))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)
  
  pose_client.send_goal(goal)

  pose_client.wait_for_result()
  command_result = pose_client.get_result()
  
  rospy.sleep(sleep_duration)
  
  #------------------------- KWADRAT ----------------------------
  print 'Zaczynam rysowac kwadrat...'
  # Wierzcholek A
  conManSwitch(['Irp6pmPoseInt'], [], True)
  
  pose_client = actionlib.SimpleActionClient('/irp6p_arm/pose_trajectory', CartesianTrajectoryAction)
  pose_client.wait_for_server()
  
  print 'Ustawienie wierzcholka A'
  
  goal = CartesianTrajectoryGoal()
  
  quaternion = getDownOrientedQuaternion()
  point = Point(zero_point - x_diff, 0.05, height)
  goal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(6.0), Pose(point, quaternion), Twist()))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)
  
  pose_client.send_goal(goal)

  pose_client.wait_for_result()
  command_result = pose_client.get_result()
  
  rospy.sleep(sleep_duration)
  
  # Wierzcholek B
  conManSwitch(['Irp6pmPoseInt'], [], True)
  
  pose_client = actionlib.SimpleActionClient('/irp6p_arm/pose_trajectory', CartesianTrajectoryAction)
  pose_client.wait_for_server()
  
  print 'Ustawienie wierzcholka B'
  
  goal = CartesianTrajectoryGoal()
  
  quaternion = getDownOrientedQuaternion()
  point = Point(zero_point - x_diff, -0.05, height)
  goal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(6.0), Pose(point, quaternion), Twist()))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)
  
  pose_client.send_goal(goal)

  pose_client.wait_for_result()
  command_result = pose_client.get_result()
  
  rospy.sleep(sleep_duration)
  
  # Wierzcholek C
  conManSwitch(['Irp6pmPoseInt'], [], True)
  
  pose_client = actionlib.SimpleActionClient('/irp6p_arm/pose_trajectory', CartesianTrajectoryAction)
  pose_client.wait_for_server()
  
  print 'Ustawienie wierzcholka C'
  
  goal = CartesianTrajectoryGoal()
  
  quaternion = getDownOrientedQuaternion()
  point = Point(zero_point + x_diff, -0.05, height)
  goal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(6.0), Pose(point, quaternion), Twist()))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)
  
  pose_client.send_goal(goal)

  pose_client.wait_for_result()
  command_result = pose_client.get_result()
  
  rospy.sleep(sleep_duration)
  
  # Wierzcholek D
  conManSwitch(['Irp6pmPoseInt'], [], True)
  
  pose_client = actionlib.SimpleActionClient('/irp6p_arm/pose_trajectory', CartesianTrajectoryAction)
  pose_client.wait_for_server()
  
  print 'Ustawienie wierzcholka D'
  
  goal = CartesianTrajectoryGoal()
  
  quaternion = getDownOrientedQuaternion()
  point = Point(zero_point + x_diff, 0.05, height)
  goal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(6.0), Pose(point, quaternion), Twist()))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)
  
  pose_client.send_goal(goal)

  pose_client.wait_for_result()
  command_result = pose_client.get_result()
  
  rospy.sleep(sleep_duration)
  
  # ----------------------------------------------------------------------------
  # Powrot do pozycji synchronizacji
  conManSwitch(['Irp6pmSplineTrajectoryGeneratorJoint'], ['Irp6pmPoseInt'], True)
    
  client = actionlib.SimpleActionClient('/irp6p_arm/spline_trajectory_action_joint', FollowJointTrajectoryAction)
  client.wait_for_server()

  print 'Powrot do pozycji synchronizacji'

  goal = FollowJointTrajectoryGoal()
  goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
  goal.trajectory.points.append(JointTrajectoryPoint([-0.10087151336609543, -1.5417429815634993, 0.019743230015841898, 1.1331041783656084, 3.658011557435151, -2.7351279214366393], [], [], [], rospy.Duration(20.0)))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

  client.send_goal(goal)

  client.wait_for_result()
  command_result = client.get_result()
  print 'Skonczylem!'
