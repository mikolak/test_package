#!/usr/bin/env python

import rospy
import math

from ROSPyIrpMoveManager import *

ROBOT = 'p'
LOOKOUT_POINT = None

if __name__ == '__main__':
	rospy.init_node('initialize')
	moveMan = ROSPyIrpMoveManager(ROBOT)
	moveMan.toolConfig(Point(0.0, 0.0, 0.25))
	
	if ROBOT == 'ot':
		moveMan.jointMove([0, 0, -0.5 * math.pi, 0, 0, 1.5 * math.pi, -0.5 * math.pi], 6)
		LOOKOUT_POINT = Point(0.76, 0.0, 1.03)
	elif ROBOT == 'p':
		moveMan.jointMove([0, -0.5 * math.pi, 0, 0, 1.5 * math.pi, 0.5 * math.pi], 12)
		LOOKOUT_POINT = Point(0.765, 0.0, 1.04)	
		
	moveMan.xyzMove(LOOKOUT_POINT, 5, False)
	moveMan.finish()
	
