#!/usr/bin/env python

import rospy
import math

from ROSPyIrpMoveManager import *

GRIP_POINT = 50
RELEASE_POINT = 750
BORDER_FORCE = 2.5

def moveToLook():
	moveMan.xyzMove(Point(0.75, 0.0, 1.03), 3, False, 0)
	
def initialize():
	moveMan.jointMove([0, 0, -0.5 * math.pi, 0, 0, 1.5 * math.pi, -0.5 * math.pi], 6)
	
def wetGripper():
	print 'Wetting the gripper=========================='
	moveMan.xyzMove(Point(0.8, -0.1, 1.), 3, False)
	moveMan.xyzMove(Point(0.8, -0.1, 0.93), 15, True, BORDER_FORCE)
	moveMan.xyzMove(Point(0.8, -0.1, 1.), 3, False)
	
def moveToPickUp():
	print 'Move to pick up====================================='
	moveMan.xyzMove(Point(0.8, 0, 0.93), 20, True, BORDER_FORCE, sleepDur=2)

def pickUp():
	print 'Picking up================================'
	grip()
	moveMan.xyzMove(Point(0.8, 0, 1.00), 1, False)
	
def prepareForPickUp():
	print 'Prepare for pick up================================'
	moveMan.xyzMove(Point(0.8, 0, 1), 3, False)
	release()
	
def prepareToGripTheGripper():
	print 'Prepare to grip the gripper==========================='
	moveMan.xyzMove(Point(0.8, 0, 1.05), 5, False)
	moveMan.toolMove([GRIP_POINT], 2)
	
def release():
	print 'Release========================================='
	moveMan.toolMove([RELEASE_POINT], 3)
	
def grip():
	print 'Grip======================================'
	moveMan.toolMove([GRIP_POINT], 3)
	
def testGrip(inc=0):
	print 'Test grip===================================='
	margin = (inc - 1) / 100
	margin = margin * 1.5
	moveMan.xyzMove(Point(0.8 + margin, 0.1, 1.00), 1, False)
	moveMan.xyzMove(Point(0.8 + margin, 0.1, 0.93), 15, True, BORDER_FORCE)
	release()
	moveMan.xyzMove(Point(0.8 + margin, 0.1, 1.0), 1, False)
	grip()
	moveMan.xyzMove(Point(0.8, 0.0, 1.0), 1, False)

def releaseGripper():
	moveMan.toolMove([-100], 3)
	
if __name__ == '__main__':
	rospy.init_node('grip_stone')
	moveMan = ROSPyIrpMoveManager('ot')
	
	initialize()
	
	# WAZNE - ot ma siedem wspolrzednych w stawach, p szesc
	moveMan.toolConfig(Point(0.0, 0.0, 0.375))
	wetGripper()
	#prepareToGripTheGripper()
	for i in range(1, 5):
		print "%d. proba" %(i)
		moveToLook()
		prepareForPickUp()
		moveToPickUp()
		pickUp()
		testGrip(i)
	
	#releaseGripper()
	moveMan.finish()
