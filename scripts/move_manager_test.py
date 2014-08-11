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

from ROSPyIrpMoveManager import *

if __name__ == '__main__':
	rospy.init_node('simple_trajectory_test')
	moveMan = ROSPyIrpMoveManager('ot')
	
	moveMan.jointMove([0, 0, -0.5 * math.pi, 0, 0, 1.5 * math.pi, -0.5 * math.pi], 6)
	#moveMan.xyzMove(Point(0.85, 0, 1.20), 5)
	moveMan.xyzMove(Point(0.80, 0, 1.05), 5, True)
	moveMan.finish()
