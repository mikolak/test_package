#!/usr/bin/env python

import rospy
import math

from ROSPyIrpMoveManager import *

if __name__ == '__main__':
	rospy.init_node('helper')
	moveMan = ROSPyIrpMoveManager('ot')
	moveMan.xyzMove(Point(0.795, 0.005, 1.0), 5, False, 0)
	#moveMan.toolMove([-1000], 2)
	#moveMan.xyzMove(Point(0.8, 0, 1.005), 5, False, 0)
	#moveMan.toolMove([-300], 2)
	#rospy.sleep(5.)
	#moveMan.toolMove([-1000], 2)
	#moveMan.xyzMove(Point(0.8, 0, 1.1), 5, False, 0)
	#moveMan.toolMove([700], 2)
	moveMan.finish()
