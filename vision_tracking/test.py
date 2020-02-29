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

cs = CameraServer.getInstance()
cs.enableLogging()

usb1 = cs.startAutomaticCapture(dev=0)
usb2 = cs.startAutomaticCapture(dev=2)

cs.waitForever()