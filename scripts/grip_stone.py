#!/usr/bin/env python

import rospy
import math

from ROSPyIrpMoveManager import *

GRIP_POINT = 50
RELEASE_POINT = 350
BORDER_FORCE = 4

def moveToPickUp():
	print 'Move to pick up====================================='
	moveMan.xyzMove(Point(0.8, 0, 0.95), 20, True, BORDER_FORCE)

def prepareForPickUp():
	print 'Prepare for pick up================================'
	moveMan.xyzMove(Point(0.8, 0, 1.0), 5, False, 0)
	moveMan.toolConfig(Point(0.0, 0.0, 0.305))
	moveMan.toolMove([RELEASE_POINT], 3)
	rospy.sleep(0.5)
	
def prepareToGripTheGripper():
	print 'Prepare to grip the gripper==========================='
	moveMan.xyzMove(Point(0.8, 0, 1.05), 10, False, 0)
	moveMan.toolMove([GRIP_POINT], 2)
	
def release():
	print 'Release========================================='
	moveMan.toolMove([RELEASE_POINT], 5)
	
def grip():
	print 'Grip======================================'
	moveMan.toolMove([GRIP_POINT], 3)
	
def testGrip():
	print 'Test grip===================================='
	moveMan.xyzMove(Point(0.8, 0, 1.0), 5, False, 0)
	moveMan.xyzMove(Point(0.8, 0.05, 1.0), 3, False, 0)
	moveMan.xyzMove(Point(0.8, 0.05, 0.95), 10, True, BORDER_FORCE)
	release()
	moveMan.xyzMove(Point(0.8, 0.05, 1.0), 3, False, 0)
	
if __name__ == '__main__':
	rospy.init_node('grip_stone')
	moveMan = ROSPyIrpMoveManager('p')
	
	# WAZNE - ot ma siedem wspolrzednych w stawach, p szesc
	prepareToGripTheGripper()
	prepareForPickUp()
	moveToPickUp()
	grip()
	testGrip()
	release()
	moveMan.finish()
