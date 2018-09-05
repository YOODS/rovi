#!/usr/bin/python

import cv2
import numpy as np
import sys
import roslib
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

Count='0'
imageL=None
imageR=None

def imgCat():
  imgc=cv2.hconcat([imageL,imageR])
  if len(Count)==1: Count='0'+Count
  cv2.imwrite('calib'+Count+'.pgm',imgc)
  return

def cbL(img):
  try:
    imageL = bridge.imgmsg_to_cv2(img, "mono8")
  except CvBridgeError, e:
    print e
  if imageR is None: return
  imgCat()
  quit()

def cbR(img):
  try:
    imageR = bridge.imgmsg_to_cv2(img, "mono8")
  except CvBridgeError, e:
    print e
  if imageL is None: return
  imgCat()
  quit()

if len(sys.argv)<2:
  print 'imsave.py "filename"'
  quit()

Count=sys.argv[1]

rospy.init_node("imsave", anonymous=True)
rospy.Subscriber("/rovi/left/image_raw",Image,cbL)
rospy.Subscriber("/rovi/right/image_raw",Image,cbR)

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
