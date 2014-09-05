#!/usr/bin/env python

import rospy
import math

from ROSPyIrpMoveManager import *

ROBOT = 'p'

if __name__ == '__main__':
	rospy.init_node('move_manager_test')
	moveMan = ROSPyIrpMoveManager(ROBOT)
	moveMan.toolConfig(Point(0.0, 0.0, 0.25))
	# WAZNE - ot ma siedem wspolrzednych w stawach, p szesc
	#moveMan.toolMove([-1000], 5)
	if ROBOT == 'ot':
		moveMan.jointMove([0, 0, -0.5 * math.pi, 0, 0, 1.5 * math.pi, -0.5 * math.pi], 6)
	elif ROBOT == 'p':
		moveMan.jointMove([0, -0.5 * math.pi, 0, 0, 1.5 * math.pi, 0.5 * math.pi], 15)
		
	#moveMan.xyzMove(Point(0.85, 0, 1.20), 5)
	#moveMan.xyzMove(Point(0.80, 0, 0.95), 20, True)
	
	moveMan.toolConfig(Point(0.0, 0.0, 0.375))
	rospy.sleep(0.100)
	moveMan.xyzMove(Point(0.76, 0, 1.05), 20, False)
	moveMan.toolMove([-1000], 5)
	moveMan.finish()
