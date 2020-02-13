#!/usr/bin/env python3
#----------------------------------------------------------------------------
# Copyright (c) 2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.
#----------------------------------------------------------------------------



import json
import time
import sys
import numpy as np
import cv2
from networktables import NetworkTables
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer

ip = '10.30.6.2'

NetworkTables.initialize(server=ip)

lower_green = np.array([0, 255, 110]) #137 240 135 - HSV, 0,90,90 - RGB
upper_green = np.array([80, 255, 200]) #143 255 148 - HSV, 86,255,255 - RGB

lower_yellow = np.array([20, 75, 100])
upper_yellow = np.array([40, 255, 255])

camWidth = 320
camHeight = 240
center_x = camWidth * .5
margin = 20

cs = CameraServer.getInstance()
cs.enableLogging()

cam1 = UsbCamera("cam1", 0)

cam1.setResolution(camWidth, camHeight)

cam1.setExposureManual(32)

cvSink = cs.getVideo(camera = cam1)

outputStream = cs.putVideo("Cam1", camWidth, camHeight)

frame = np.zeros(shape=(camHeight, camWidth, 3), dtype=np.uint8)

#Makes Raspberry Pi a listener to detect changes sent from roboRIO
def listener(table, key, value, isNew):
	print("value changed: key: '%s'; value: %s; isNew: %s" % (key, value, isNew))
	return value
def connectionListener(connected, info):
	print(info, "; Connected=%s" % connected)

sd = NetworkTables.getTable("SmartDashboard")
sd.putString('dir', 'working')

NetworkTables.addConnectionListener(connectionListener, immediateNotify = True)
sd.addEntryListener(listener, key="cam")

print(sd.getString(key="cam", defaultValue = ""))

font = cv2.FONT_HERSHEY_SIMPLEX

while(True):

	time, frame = cvSink.grabFrame(frame)
	
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
	contours = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-2] 
	
	largest_area = 0
	largest_contour = np.array([0, 0])
	sec_largest_area = 0
	sec_largest_contour = np.array([0, 0])
	
	for c in contours:
		area = cv2.contourArea(c)
		if area > largest_area:
			sec_largest_contour = largest_contour
			sec_largest_area = largest_area
			largest_contour = c
			largest_area = area
		elif area > sec_largest_area:
			sec_largest_contour = c
			sec_largest_area = area
		
	#print('largest ', largest_area)
	#print('sec largest', sec_largest_area)

	cv2.putText(frame, str(largest_area), (45, 100), font, 1, (255,255,255), 2, cv2.LINE_AA)
	cv2.putText(frame, str(sec_largest_area), (45, 150), font, 1, (255,255,255), 2, cv2.LINE_AA)

	if(largest_area > 0 and sec_largest_area > 0):
		M1 = cv2.moments(largest_contour)
		center_contour_x = int(M1['m10']/M1['m00'])
		
		M2 = cv2.moments(sec_largest_contour)
		center_contour_x_2 = int(M2['m10']/M2['m00'])
		
		center_average_x = .5 * (center_contour_x + center_contour_x_2)

		error = center_average_x - center_x

		sd.putNumber('dir', error)
		'''
		if abs(center_average_x - center_x) > margin:
			if center_average_x < center_x:
				sd.putString('dir', 'l')
				print('l')
				cv2.putText(frame, 'l', (275, 50), font, 2, (255,255,255), 2, cv2.LINE_AA)
			elif center_average_x > center_x:
				sd.putString('dir', 'r')
				print('r')
				cv2.putText(frame, 'r', (275, 50), font, 2, (255,255,255), 2, cv2.LINE_AA)
		else:
			sd.putString('dir', 'f')
			print('f')
			cv2.putText(frame, 'f', (275, 50), font, 2, (255,255,255), 2, cv2.LINE_AA)
		'''
		cv2.drawContours(frame, [largest_contour], 0, (0, 0, 255), 3)
		cv2.drawContours(frame, [sec_largest_contour], 0, (0, 0 , 255), 3)

		x,y,w,h = cv2.boundingRect(largest_contour)
		#print(x, y, w, h)
		cv2.rectangle(frame,(x,y),(x+w, y+h),(0,255,0),2)
	
	elif(largest_area > 0 and sec_largest_area == 0):
		M1 = cv2.moments(largest_contour)
		center_contour_x = int(M1['m10']/M1['m00'])
		'''
		if abs(center_contour_x - center_x) > margin:
			if center_contour_x < center_x:
				sd.putString('dir', 'l')
				print('l')
				cv2.putText(frame, 'l', (275, 50), font, 2, (255,255,255), 2, cv2.LINE_AA)
			elif center_contour_x > center_x:
				sd.putString('dir', 'r')
				print('r')
				cv2.putText(frame, 'r', (275, 50), font, 2, (255,255,255), 2, cv2.LINE_AA)
		else:
			sd.putString('dir', 'f')
			print('f')
			cv2.putText(frame, 'f', (275, 50), font, 2, (255,255,255), 2, cv2.LINE_AA)
		'''
		cv2.drawContours(frame, [largest_contour], 0, (0, 0, 255), 3)



	outputStream.putFrame(frame)
	
NetworkTables.shutdown()
cap.release()
cv2.destroyAllWindows()