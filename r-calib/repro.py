#!/usr/bin/python

import cv2
import numpy as np
import roslib
import rospy
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Transform
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../script'))
import tflib

zLimit=100.0 #to eliminate far points
Tolerance=0.001
Rejection=2.5

def P0():
  return np.array([]).reshape((-1,3))

def np2F(p):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(p)
  return f

def cb_repro(msg):
  global Kmat
  Dmat=np.float32([0,0,0,0,0])
  P1=np.reshape(msg.data,(-1,5))
  P2=P1[np.where(P1.T[3]<10000)]
  M2=P2.T[:3].astype(np.float32)
  S2=P2.T[3:5].astype(np.float32)
#  print '====='
#  print M2
#  print '-----'
#  print S2
  ret,rvec,tvec = cv2.solvePnP(M2.T,S2.T,Kmat,Dmat)
  rmat,jacob = cv2.Rodrigues(rvec)
  RT=np.hstack((rmat,tvec))
  Nc,Nl=M2.shape
  P3=np.vstack((M2,np.ones((1,Nl))))
  xyz=np.dot(Kmat,np.dot(RT,P3))
  s=xyz[2]
  uv=xyz[:2]/s
#  print "uv",uv
  err=map(np.linalg.norm,(uv-S2).T)
  print "Mean=",np.mean(err),"Max=",np.max(err)
  return

########################################################
rospy.init_node("repro",anonymous=True)
###Get Params
if rospy.has_param('/robot/calib/bTc'):
  Kmat=np.array(rospy.get_param('/rovi/left/remap/K')).reshape((3,3))
  print Kmat

###Input topics
rospy.Subscriber("/gridboard/floats",numpy_msg(Floats),cb_repro)

###Output topics
pub_tf=rospy.Publisher("/repro/tf",Transform,queue_size=1)

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
