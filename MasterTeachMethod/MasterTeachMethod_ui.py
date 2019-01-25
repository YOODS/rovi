#!/usr/bin/python

import cv2
import numpy as np
import roslib
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import Transform
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../script'))
import tflib

def cb_clr_scn(f):
  print 'CLEAR SCENE'
  f=Bool()
  pub_X0.publish(f)
  return

def cb_capt(tf):
  print 'CAPTURE SCENE'
  pub_euler.publish(tf)
  f=Bool()
  pub_X1.publish(f)
  return

def cb_match(f):
  print 'MATCH SCENE'
  f=Bool()
  pub_X2.publish(f)
  return

########################################################

rospy.init_node("ui",anonymous=True)
###Input topics
rospy.Subscriber("clear_scene",Bool,cb_clr_scn)
rospy.Subscriber("capture_scene_1",Transform,cb_capt)
rospy.Subscriber("capture_scene_2",Transform,cb_capt)
rospy.Subscriber("capture_scene_3",Transform,cb_capt)
rospy.Subscriber("match_scene",Bool,cb_match)

###Output topics
pub_euler=rospy.Publisher("/robot/euler",Transform,queue_size=1)
pub_X0=rospy.Publisher("/solver/X0",Bool,queue_size=1)    #clear
pub_X1=rospy.Publisher("/solver/X1",Bool,queue_size=1)    #capure
pub_X2=rospy.Publisher("/solver/X2",Bool,queue_size=1)    #match

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
