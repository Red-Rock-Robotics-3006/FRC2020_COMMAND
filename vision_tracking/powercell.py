#!/usr/bin/env python3

from __future__ import division
import cv2
import numpy as np
import time

def nothing(*arg):
        pass

FRAME_WIDTH = 320
FRAME_HEIGHT = 240

# Initial HSV GUI slider values to load on program start.
#icol = (36, 202, 59, 71, 255, 255)    # Green
#icol = (18, 0, 196, 36, 255, 255)  # Yellow
#icol = (89, 0, 0, 125, 255, 255)  # Blue
#icol = (0, 100, 80, 10, 255, 255)   # Red
#icol = (104, 117, 222, 121, 255, 255)   # test
icol = (0, 0, 0, 255, 255, 255)   # New start

cv2.namedWindow('colorTest')
# Lower range colour sliders.
cv2.createTrackbar('lowHue', 'colorTest', icol[0], 255, nothing)
cv2.createTrackbar('lowSat', 'colorTest', icol[1], 255, nothing)
cv2.createTrackbar('lowVal', 'colorTest', icol[2], 255, nothing)
# Higher range colour sliders.
cv2.createTrackbar('highHue', 'colorTest', icol[3], 255, nothing)
cv2.createTrackbar('highSat', 'colorTest', icol[4], 255, nothing)
cv2.createTrackbar('highVal', 'colorTest', icol[5], 255, nothing)

# Initialize webcam. Webcam 0 or webcam 1 or ...
vidCapture = cv2.VideoCapture(1)
vidCapture.set(cv2.CAP_PROP_FRAME_WIDTH,FRAME_WIDTH)
vidCapture.set(cv2.CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT)
vidCapture.set(cv2.CAP_PROP_EXPOSURE,-8)

while True:
    timeCheck = time.time()
    # Get HSV values from the GUI sliders.
    lowHue = cv2.getTrackbarPos('lowHue', 'colorTest')
    lowSat = cv2.getTrackbarPos('lowSat', 'colorTest')
    lowVal = cv2.getTrackbarPos('lowVal', 'colorTest')
    highHue = cv2.getTrackbarPos('highHue', 'colorTest')
    highSat = cv2.getTrackbarPos('highSat', 'colorTest')
    highVal = cv2.getTrackbarPos('highVal', 'colorTest')

    # Get webcam frame
    _, frame = vidCapture.read()

    # Show the original image.
    cv2.imshow('frame', frame)

    # Convert the frame to HSV colour model.
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # HSV values to define a colour range we want to create a mask from.
    colorLow = np.array([lowHue,lowSat,lowVal])
    colorHigh = np.array([highHue,highSat,highVal])
    mask = cv2.inRange(frameHSV, colorLow, colorHigh)
    # Show the first mask
    cv2.imshow('mask-plain', mask)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]
    biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
    
    #cv2.drawContours(frame, biggest_contour, -1, (0,255,0), 3)
    x,y,w,h = cv2.boundingRect(biggest_contour)
    cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
    
    #cv2.drawContours(frame, contours, -1, (0,255,0), 3)
    
    #cv2.drawContours(frame, contours, 3, (0,255,0), 3)
    
    #cnt = contours[1]
    #cv2.drawContours(frame, [cnt], 0, (0,255,0), 3)

    # Show final output image
    cv2.imshow('colorTest', frame)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
    #print('fps - ', 1/(time.time() - timeCheck))
cv2.destroyAllWindows()
vidCapture.release()