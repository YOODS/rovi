#!/usr/bin/env python
try:
  from jsk_rviz_plugins.msg import *
except:
  import roslib;roslib.load_manifest("jsk_rviz_plugins")
  from jsk_rviz_plugins.msg import *

from std_msgs.msg import Bool,ColorRGBA,Float32
import rospy
import math

tmX0=rospy.Time()
tmX1=0
tmX2=0
tmY0=0
tmY1=0
tmY2=0

def putText():
  text.text = """YOODS
X1 %f
X2 %f
Y1 %f
Y2 %f
""" % (tmX1,tmX2-tmX1,tmY1-tmX1,tmY2-tmX1)
  text_pub.publish(text)

def cb_X1(f):
  global tmX1,tmX2,tmY1,tmY2
  tmX1=rospy.get_time()
  tmX2=tmX1
  tmY1=tmX1
  tmY2=tmX1
  putText()

def cb_X2(f):
  global tmX1,tmX2,tmY1,tmY2
  tmX2=rospy.get_time()
  putText()

def cb_Y1(f):
  global tmX1,tmX2,tmY1,tmY2
  tmY1=rospy.get_time()
  putText()

def cb_Y2(f):
  global tmX1,tmX2,tmY1,tmY2
  tmY2=rospy.get_time()
  putText()

rospy.init_node("overlay_sample")

text = OverlayText()
text.width = 300
text.height = 200
text.left = 10
text.top = 10
text.text_size = 12
text.line_width = 2
text.font = "DejaVu Sans Mono"
text.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)

text_pub = rospy.Publisher("text_sample", OverlayText, queue_size=1)
rospy.Subscriber("/solver/X1",Bool,cb_X1)
rospy.Subscriber("/solver/X2",Bool,cb_X2)
rospy.Subscriber("/solver/Y1",Bool,cb_Y1)
rospy.Subscriber("/solver/Y2",Bool,cb_Y2)

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"

