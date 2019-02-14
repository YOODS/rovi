#!/usr/bin/python

import numpy as np
import cv2
import roslib
import rospy
from rospy.numpy_msg import numpy_msg
from rovi.msg import Floats
from scipy import optimize

Radius=10

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def pickPoints(points,center,radius):
  return points[np.where(np.linalg.norm(points-center,axis=1)<radius)]

def fit_func(parameter,x,y,z):
  a=parameter[0]
  b=parameter[1]
  c=parameter[2]
  d=parameter[3]
  residual=(a*x+b*y+c*z+d)**2/(a**2+b**2+c**2)
  return residual

def getPlane(pnt):
  dat=pnt.T
  result=optimize.leastsq(fit_func,[1,1,1,1],args=(dat[0],dat[1],dat[2]))
  nabc=np.linalg.norm(result[0][:3])
  plane=result[0]/nabc
  return plane

def To3d(uv0,uv1,P0,P1):
  uvs=np.asarray([uv0,uv1])
  B=np.ravel([[P0[2][0]*uvs[0][0]-P0[0][0], P0[2][1]*uvs[0][0]-P0[0][1], P0[2][2]*uvs[0][0]-P0[0][2]],
     [P0[2][0]*uvs[0][1]-P0[1][0], P0[2][1]*uvs[0][1]-P0[1][1], P0[2][2]*uvs[0][1]-P0[1][2]],
     [P1[2][0]*uvs[1][0]-P1[0][0], P1[2][1]*uvs[1][0]-P1[0][1], P1[2][2]*uvs[1][0]-P1[0][2]],
     [P1[2][0]*uvs[1][1]-P1[1][0], P1[2][1]*uvs[1][1]-P1[1][1], P1[2][2]*uvs[1][1]-P1[1][2]]]).reshape(4,3)
  b=np.ravel([P0[0][3]-P0[2][3]*uvs[0][0],
     P0[1][3]-P0[2][3]*uvs[0][1],
     P1[0][3]-P1[2][3]*uvs[1][0],
     P1[1][3]-P1[2][3]*uvs[1][1]])
  BP=np.dot(B.T,B)
  Mt=np.dot(np.linalg.inv(BP),B.T)
  WP=[Mt[0][0]*b[0] + Mt[0][1]*b[1] + Mt[0][2]*b[2] + Mt[0][3]*b[3],
      Mt[1][0]*b[0] + Mt[1][1]*b[1] + Mt[1][2]*b[2] + Mt[1][3]*b[3],
      Mt[2][0]*b[0] + Mt[2][1]*b[1] + Mt[2][2]*b[2] + Mt[2][3]*b[3]]
  return np.ravel(WP)

def cb_ps(msg): #callback of ps_floats
  Pc=np.reshape(msg.data,(-1,3))
  cn0=To3d(PosL[0],PosR[0],PLmat,PRmat)
  cn1=To3d(PosL[1],PosR[1],PLmat,PRmat)
  pnt0=pickPoints(Pc,cn0,Radius)
  pnt1=pickPoints(Pc,cn1,Radius)
  print "Marker0=",cn0
  print "Marker1=",cn1
  print len(pnt0),len(pnt1)
  if len(pnt0)>100 and len(pnt1)>100:
    pl0=getPlane(pnt0-cn0)
    pl1=getPlane(pnt1-cn0)
    print pl0[3]-pl1[3]


def cb_posl(msg):
  global PosL
  PosL=np.reshape(msg.data,(-1,2))

def cb_posr(msg):
  global PosR
  PosR=np.reshape(msg.data,(-1,2))

###############################################################
rospy.init_node('p2p',anonymous=True)
pb_dist=rospy.Publisher("/p2p/dist",numpy_msg(Floats),queue_size=1)

rospy.Subscriber("/rovi/ps_floats",numpy_msg(Floats),cb_ps)
rospy.Subscriber("/rovi/left/marker/pos",numpy_msg(Floats),cb_posl)
rospy.Subscriber("/rovi/right/marker/pos",numpy_msg(Floats),cb_posr)

Qmat=np.asarray(rospy.get_param('/rovi/genpc/Q')).reshape((4,4))
PLmat=np.asarray(rospy.get_param('/rovi/left/remap/P')).reshape((3,4))
PRmat=np.asarray(rospy.get_param('/rovi/right/remap/P')).reshape((3,4))


try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
