#!/usr/bin/python

import numpy as np
import cv2
import roslib
import rospy
from rospy.numpy_msg import numpy_msg
from rovi.msg import Floats
from scipy import optimize
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

Radius=5
Goal=0.02

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def np2PC(d):  #numpy to PointCloud
  pc=PointCloud()
  pc.header.frame_id='camera'
  f=np.ravel(d)
  for e in f.reshape((-1,3)):
    p=Point32()
    p.x=e[0]
    p.y=e[1]
    p.z=e[2]
    pc.points.append(p)
  return pc

def pickPoints(points,center,radius):
  return points[np.where(np.linalg.norm(points-center,axis=1)<radius)]

def fit_func(parameter,x,y,z,w):
  a=parameter[0]
  b=parameter[1]
  c=parameter[2]
  d=parameter[3]
  e=parameter[4]
  residual=(a*x+b*y+c*z+d*w+e)**2/(a**2+b**2+c**2)
  return residual

def getPlane(pnt):
  dat=pnt.T
  result=optimize.leastsq(fit_func,[1,1,1,1,1],args=(dat[0],dat[1],dat[2],dat[3]))
  nabc=np.linalg.norm(result[0][:3])
  plane=result[0]/nabc
  return plane

def getDist(pl,pnt):
  dm=np.sqrt(np.inner(pl[:3],pl[:3]))
  return np.dot(pl,np.vstack((pnt.T,np.ones(len(pnt)))))/dm

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
  global Result
  Pc=np.reshape(msg.data,(-1,3))
  try:
    cn0=To3d(PosL[0],PosR[0],PLmat,PRmat)
    cn1=To3d(PosL[1],PosR[1],PLmat,PRmat)
  except NameError:
    cn0=np.asarray([0.0,0.0,0.0])
    cn1=np.asarray([0.0,0.0,40.0])
#  print "Marker=",cn0,cn1
  pb_red.publish(np2PC(cn0.reshape((-1,3))))
  pb_blue.publish(np2PC(cn1.reshape((-1,3))))
  pnt0=pickPoints(Pc,cn0,Radius)
  pnt1=pickPoints(Pc,cn1,Radius)
  if len(pnt0)>100 and len(pnt1)>100:
    pnt0=np.hstack((pnt0,np.zeros((len(pnt0),1))))
    pnt1=np.hstack((pnt1,np.ones((len(pnt1),1))))
    print "ponits",len(pnt0),len(pnt1)
    pnt=np.vstack((pnt0,pnt1))
    for cnt in range(5):
      pl=getPlane(pnt)
      dist=getDist(pl,pnt)
      sig=np.std(dist)
      pnt=pnt[np.where(np.abs(dist)<1.5*sig)]
    res=np.array([np.sum(pnt[:,3]==0),np.sum(pnt[:,3]==1),sig,np.abs(pl[3])])
    Result=np.vstack((Result,res))
    if len(Result)>10: Result=Result[1:]
    print "[points0] [points1] [sigma] [distance]"
    for col in Result:
      print int(col[0]),int(col[1]),('%.6f'%col[2]),('%.6f'%col[3])
  else:
    print "Too few point"

def cb_posl(msg):
  global PosL
  PosL=np.reshape(msg.data,(-1,2))

def cb_posr(msg):
  global PosR
  PosR=np.reshape(msg.data,(-1,2))

###############################################################
rospy.init_node('p2p',anonymous=True)
pb_dist=rospy.Publisher("/p2p/dist",numpy_msg(Floats),queue_size=1)
pb_red=rospy.Publisher("/p2p/red",PointCloud,queue_size=1)
pb_blue=rospy.Publisher("/p2p/blue",PointCloud,queue_size=1)

rospy.Subscriber("/rovi/ps_floats",numpy_msg(Floats),cb_ps)
rospy.Subscriber("/rovi/left/marker/pos",numpy_msg(Floats),cb_posl)
rospy.Subscriber("/rovi/right/marker/pos",numpy_msg(Floats),cb_posr)

try:
  Qmat=np.asarray(rospy.get_param('/rovi/genpc/Q')).reshape((4,4))
  PLmat=np.asarray(rospy.get_param('/rovi/left/remap/P')).reshape((3,4))
  PRmat=np.asarray(rospy.get_param('/rovi/right/remap/P')).reshape((3,4))
except KeyError:
  print "No key"

Result=np.array([],dtype=float).reshape((-1,4))

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
