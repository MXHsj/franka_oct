#! /usr/bin/env python3
import rospy
from cv2 import cv2
import selectinwindow
from GetRealSenseData import *

realsense = GetRealSenseData()

cv2.namedWindow('RGB-depth', cv2.WINDOW_AUTOSIZE)
