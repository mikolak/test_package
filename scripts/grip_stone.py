#!/usr/bin/env python

import rospy
import math
import threading

from ROSPyIrpMoveManager import *

# WAZNE - ot ma siedem wspolrzednych w stawach, p szesc
	
GRIP_POINT = 50
RELEASE_POINT = 750
BORDER_FORCE = 2.5

realX = 0
realY = 0

lastX = 0.
lastY = 0.

xLock = threading.Lock()
yLock = threading.Lock()

def moveToLook():
	moveMan.xyzMove(Point(0.76, 0.0, 1.03), 3, False)
	
def initialize():
	moveMan.jointMove([0, 0, -0.5 * math.pi, 0, 0, 1.5 * math.pi, -0.5 * math.pi], 6)

def waitForReading():
	rospy.sleep(5.)
	
def countPosition():
	print 'Counting position=================================='
	scale = 50.0
	height = 500.0
	width = 400.0
	x =  getX()
	y = getY()
	
	#print "I've read: x = %f, y = %f" %(x, y)
	
	#wyliczenie korekty z powodu krzywej kamery
	#1 piksel ~ 0.023 cm
	#skrajnie na gorze 0.5 centymetra w lewo
	#skrajnie z lewej 0.2 centymetra w dol
	correctionX = (-1) * (1- y/height) * 25
	correctionY = (1 - x/width) * 10
	
	global realX
	global realY
	
	realY = ((height - (y + correctionY))/scale)/100.0
	realX = ((-(x + correctionX) + width/2.0)/scale)/100.0
	
	
	print 'Przesuniecie: x = %f, y = %f' %(realY, realX)
	
def xCallback(data):
	xLock.acquire()
	global lastX
	lastX = data.data
	xLock.release()

def yCallback(data):
	yLock.acquire()
	global lastY
	lastY = data.data
	yLock.release()		

def getX():
	xLock.acquire()
	ret = lastX
	xLock.release()
	return ret

def getY():
	yLock.acquire()
	ret = lastY
	yLock.release()
	return ret

def wetGripper():
	print 'Wetting the gripper=========================='
	moveMan.xyzMove(Point(0.8, -0.1, 1.), 3, False)
	moveMan.xyzMove(Point(0.8, -0.1, 0.93), 20, True, BORDER_FORCE, sleepDur=2)
	moveMan.xyzMove(Point(0.8, -0.1, 1.), 3, False)
	
def moveToPickUp():
	print 'Move to pick up====================================='
	#moveMan.xyzMove(Point(0.8, 0, 0.93), 20, True, BORDER_FORCE, sleepDur=2)
	moveMan.xyzMove(Point(0.76 + realY, realX, 0.93), 20, True, BORDER_FORCE, sleepDur=2)

def pickUp():
	print 'Picking up================================'
	grip()
	moveMan.xyzMove(Point(0.76 + realY, realX, 1.00), 3, False)
	
def prepareForPickUp():
	print 'Prepare for pick up================================'
	#moveMan.xyzMove(Point(0.8, 0, 1), 3, False)
	moveMan.xyzMove(Point(0.76 + realY, realX, 1), 3, False)
	print 'PORUSZAM SIE DO: X = %f, Y = %f' %(0.76 + realY, realX)
	rospy.sleep(10.)
	
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
	margin = (inc - 1.) / 100.
	margin = margin * 1.5
	moveMan.xyzMove(Point(0.76 + margin, 0.1, 1.00), 3, False)
	moveMan.xyzMove(Point(0.76 + margin, 0.1, 0.93), 20, True, BORDER_FORCE)
	release()
	moveMan.xyzMove(Point(0.76 + margin, 0.1, 1.0), 3, False)
	grip()
	moveMan.xyzMove(Point(0.76, 0.0, 1.0), 3, False)

def releaseGripper():
	moveMan.toolMove([-100], 3)
	
if __name__ == '__main__':
	rospy.init_node('grip_stone')
	moveMan = ROSPyIrpMoveManager('ot')
	
	rospy.Subscriber('/center_x', Float32, xCallback)
	rospy.Subscriber('/center_y', Float32, yCallback)
	rospy.sleep(1.)
	
	initialize()

	moveMan.toolConfig(Point(0.0, 0.0, 0.375))
	wetGripper()
	
	for i in range(1, 6):
		print "%d. proba" %(i)
		moveToLook()
		waitForReading()
		countPosition()
		prepareForPickUp()
		moveToPickUp()
		pickUp()
		testGrip(i)
	
	releaseGripper()
	moveMan.finish()
