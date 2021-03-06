#!/usr/bin/env python

import rospy
import math

from ROSPyIrpMoveManager import *

ROBOT = 'p'

if __name__ == '__main__':
	rospy.init_node('helper')
	moveMan = ROSPyIrpMoveManager(ROBOT)
	moveMan.toolConfig(Point(0.0, 0.0, 0.375))
	moveMan.xyzMove(Point(0.76, 0.0, 1.23), 6, False, 0)
	moveMan.xyzMove(Point(0.765, 0.0, 1.04), 30, True, 1, sleepDur=2.)
	
	#moveMan.xyzMove(Point(0.795, 0.005, 1.1), 5, False, 0)
	#moveMan.xyzMove(Point(0.795, 0.005, 1.05), 20, True, 3)
	
	
	
	#moveMan.xyzMove(Point(0.8, 0.0, 1.0), 10, True, 2)
	#rospy.sleep(3.)
	#moveMan.xyzMove(Point(0.8, 0.0, 1.00), 20, True, 5)
	#moveMan.toolMove([-1000], 2)
	#moveMan.xyzMove(Point(0.8, 0, 1.005), 5, False, 0)
	#moveMan.toolMove([-300], 2)
	#rospy.sleep(5.)
	#moveMan.toolMove([-1000], 2)
	#moveMan.xyzMove(Point(0.8, 0, 1.1), 5, False, 0)
	#moveMan.toolMove([450], 2)
	#rospy.sleep(2.)
	#moveMan.toolMove([100], 2)
	
	moveMan.finish()
