#!/usr/bin/env python

import rospy
import math

from ROSPyIrpMoveManager import *

GRIP_POINT = -100
RELEASE_POINT = 550
BORDER_FORCE = 5

def moveToPickUp():
	print 'Move to pick up====================================='
	moveMan.xyzMove(Point(0.8, 0, 0.95), 20, True, BORDER_FORCE)
	#print '1'
	#moveMan.xyzMove(Point(0.8, 0, 0.98), 5, True, BORDER_FORCE)
	#print '2'
	#moveMan.xyzMove(Point(0.8, 0, 0.97), 5, True, BORDER_FORCE)
	#print '3'
	#moveMan.xyzMove(Point(0.8, 0, 0.96), 5, True, BORDER_FORCE)
	#print '4'
	#moveMan.xyzMove(Point(0.8, 0, 0.95), 5, True, BORDER_FORCE)
	#print '5'
	#moveMan.xyzMove(Point(0.8, 0, 0.94), 5, True, BORDER_FORCE)

def prepareForPickUp():
	print 'Prepare for pick up================================'
	moveMan.xyzMove(Point(0.8, 0, 1), 5, False, 0)
	release()
	rospy.sleep(2.)
	moveMan.switchForceTransformation(True)
	rospy.sleep(2.)
	
def prepareToGripTheGripper():
	print 'Prepare to grip the gripper==========================='
	moveMan.xyzMove(Point(0.8, 0, 1.05), 10, False, 0)
	moveMan.toolMove([GRIP_POINT], 2)
	
def release():
	print 'Release========================================='
	moveMan.toolMove([RELEASE_POINT], 3)
	
def grip():
	print 'Grip======================================'
	moveMan.toolMove([GRIP_POINT], 3)
	
def testGrip():
	print 'Test grip===================================='
	moveMan.xyzMove(Point(0.8, 0, 1.0), 5, False, 0)
	moveMan.xyzMove(Point(0.8, 0.05, 1.0), 3, False, 0)
	moveMan.xyzMove(Point(0.8, 0.05, 0.95), 25, True, BORDER_FORCE)
	release()
	moveMan.xyzMove(Point(0.8, 0.05, 1.0), 3, False, 0)
	grip()

def releaseGripper():
	moveMan.toolMove([-100], 3)
	
if __name__ == '__main__':
	rospy.init_node('grip_stone')
	moveMan = ROSPyIrpMoveManager('ot')
	
	# WAZNE - ot ma siedem wspolrzednych w stawach, p szesc
	moveMan.toolConfig(Point(0.0, 0.0, 0.305))
	#prepareToGripTheGripper()
	prepareForPickUp()
	moveToPickUp()
	grip()
	testGrip()
	release()
	releaseGripper()
	moveMan.finish()
