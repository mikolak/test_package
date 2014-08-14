#!/usr/bin/env python

import rospy
import math

from ROSPyIrpMoveManager import *

if __name__ == '__main__':
	rospy.init_node('move_manager_test')
	moveMan = ROSPyIrpMoveManager('p')
	moveMan.toolConfig(Point(0.0, 0.0, 0.25))
	# WAZNE - ot ma siedem wspolrzednych w stawach, p szesc
	moveMan.toolMove([-1000], 5)
	moveMan.jointMove([0, -0.5 * math.pi, 0, 0, 1.5 * math.pi, -0.5 * math.pi], 6)
	#moveMan.xyzMove(Point(0.85, 0, 1.20), 5)
	#moveMan.xyzMove(Point(0.80, 0, 0.95), 20, True)
	moveMan.toolConfig(Point(0.0, 0.0, 0.305))
	rospy.sleep(0.100)
	moveMan.xyzMove(Point(0.8, 0, 1.0), 20, True, 5)
	moveMan.finish()
