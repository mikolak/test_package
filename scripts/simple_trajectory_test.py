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
  v_x = 0
  v_y = 0
  v_z = 0
  angle = 0.5 * 0 #(0.5 * pi)
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
  conManSwitch(['SplineTrajectoryGeneratorJoint'], [], True)
    
  client = actionlib.SimpleActionClient('/irp6p_arm/spline_trajectory_action_joint', FollowJointTrajectoryAction)
  client.wait_for_server()

  print 'Server\'s A OK!'

  goal = FollowJointTrajectoryGoal()
  goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
  #goal.trajectory.points.append(JointTrajectoryPoint([-0.10087151336609543, -1.5417429815634993, 0.019743230015841898, 1.1331041783656084, 3.658011557435151, -2.7351279214366393], [], [], [], rospy.Duration(4.0)))
  #goal.trajectory.points.append(JointTrajectoryPoint([-0.10087151336609543, -1.5417429815634993, 0.019743230015841898, 0, -1.57, 0], [], [], [], rospy.Duration(6.0)))
  # 1.5 * pi
  goal.trajectory.points.append(JointTrajectoryPoint([0, -1.57, 0, 0, 4.6, 0], [], [], [], rospy.Duration(6.0)))
  goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)

  client.send_goal(goal)

  client.wait_for_result()
  command_result = client.get_result()
  
  #--------------------------------------------------
  # Kartezjanski
  #---------------------------------------------------
  #conManSwitch(['PoseInt'], ['SplineTrajectoryGeneratorJoint'], True)
  
  #pose_client = actionlib.SimpleActionClient('/irp6p_arm/pose_trajectory', CartesianTrajectoryAction)
  #pose_client.wait_for_server()
  
  #print 'Server\'s ready'
  
  #goal = CartesianTrajectoryGoal()
  
  #quaternion = getDownOrientedQuaternion()
  #point = Point(0.65, 0, 1.15)
  #goal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(10.0), Pose(point, quaternion), Twist()))
  #goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.2)
  
  #pose_client.send_goal(goal)

  #pose_client.wait_for_result()
  #command_result = pose_client.get_result()
  
  print 'I\'m done for!'
  
  #conManSwitch([], ['PoseInt'], True)
  
  #tool_client = actionlib.SimpleActionClient('/irp6p_arm/tool_trajectory', CartesianTrajectoryAction)
  #tool_client.wait_for_server()
