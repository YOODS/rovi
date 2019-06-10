#!/usr/bin/python

import numpy as np
import math
import roslib
import rospy
from geometry_msgs.msg import Transform

def dict2tf(d):
  tf=Transform()
  tf.translation.x=d['translation']['x']
  tf.translation.y=d['translation']['y']
  tf.translation.z=d['translation']['z']
  tf.rotation.x=d['rotation']['x']
  tf.rotation.y=d['rotation']['y']
  tf.rotation.z=d['rotation']['z']
  tf.rotation.w=d['rotation']['w']
  return tf

def tf2dict(tf):
  d={'translation':{'x':0,'y':0,'x':0},'rotation':{'x':0,'y':0,'x':0,'w':0}}
  d['translation']['x']=tf.translation.x
  d['translation']['y']=tf.translation.y
  d['translation']['z']=tf.translation.z
  d['rotation']['x']=tf.rotation.x
  d['rotation']['y']=tf.rotation.y
  d['rotation']['z']=tf.rotation.z
  d['rotation']['w']=tf.rotation.w
  return d

def toRT(tf):
  x=tf.rotation.x
  y=tf.rotation.y
  z=tf.rotation.z
  w=tf.rotation.w
  tx=tf.translation.x
  ty=tf.translation.y
  tz=tf.translation.z
  xx=x*x
  yy=y*y
  zz=z*z
  ww=w*w
  return np.matrix([[xx-yy-zz+ww,2.*(x*y-w*z),2.*(x*z+w*y),tx],[2.*(x*y+w*z),yy+ww-xx-zz,2.*(y*z-w*x),ty],[2.*(x*z-w*y),2.*(y*z+w*x),zz+ww-xx-yy,tz],[ 0, 0, 0, 1]])

def fromRT(rt):
  if (rt[0,0]+rt[1,1]+rt[2,2]>0):
    s=math.sqrt(1.0+rt[0,0]+rt[1,1]+rt[2,2])*2  #s=qw*4
    qw=s/4
    qx=(rt[2,1]-rt[1,2])/s
    qy=(rt[0,2]-rt[2,0])/s
    qz=(rt[1,0]-rt[0,1])/s
  elif ((rt[0,0]>rt[1,1]) and (rt[0,0]>rt[2,2])):
    s=math.sqrt(1.0+rt[0,0]-rt[1,1]-rt[2,2])*2  #s=qx*4
    qw=(rt[2,1]-rt[1,2])/s
    qx=s/4
    qy=(rt[0,1]+rt[1,0])/s
    qz=(rt[0,2]+rt[2,0])/s
  elif (rt[1,1]>rt[2,2]):
    s=math.sqrt(1.0-rt[0,0]+rt[1,1]-rt[2,2])*2  #s=qy*4
    qw=(rt[0,2]-rt[2,0])/s
    qx=(rt[0,1]+rt[1,0])/s
    qy=s/4
    qz=(rt[1,2]+rt[2,1])/s
  else:
    s=math.sqrt(1.0-rt[0,0]-rt[1,1]+rt[2,2])*2  #s=qz*4
    qw=(rt[1,0]-rt[0,1])/s
    qx=(rt[0,2]+rt[2,0])/s
    qy=(rt[1,2]+rt[2,1])/s
    qz=s/4
  tf=Transform()
  tf.rotation.w=qw
  tf.rotation.x=qx
  tf.rotation.y=qy
  tf.rotation.z=qz
  tf.translation.x=rt[0,3]
  tf.translation.y=rt[1,3]
  tf.translation.z=rt[2,3]
  return tf

def fromRTtoVec(rt):
  if (rt[0,0]+rt[1,1]+rt[2,2]>0):
    s=math.sqrt(1.0+rt[0,0]+rt[1,1]+rt[2,2])*2  #s=qw*4
    qw=s/4
    qx=(rt[2,1]-rt[1,2])/s
    qy=(rt[0,2]-rt[2,0])/s
    qz=(rt[1,0]-rt[0,1])/s
  elif ((rt[0,0]>rt[1,1]) and (rt[0,0]>rt[2,2])):
    s=math.sqrt(1.0+rt[0,0]-rt[1,1]-rt[2,2])*2  #s=qx*4
    qw=(rt[2,1]-rt[1,2])/s
    qx=s/4
    qy=(rt[0,1]+rt[1,0])/s
    qz=(rt[0,2]+rt[2,0])/s
  elif (rt[1,1]>rt[2,2]):
    s=math.sqrt(1.0-rt[0,0]+rt[1,1]-rt[2,2])*2  #s=qy*4
    qw=(rt[0,2]-rt[2,0])/s
    qx=(rt[0,1]+rt[1,0])/s
    qy=s/4
    qz=(rt[1,2]+rt[2,1])/s
  else:
    s=math.sqrt(1.0-rt[0,0]-rt[1,1]+rt[2,2])*2  #s=qz*4
    qw=(rt[1,0]-rt[0,1])/s
    qx=(rt[0,2]+rt[2,0])/s
    qy=(rt[1,2]+rt[2,1])/s
    qz=s/4
  vec=np.array([[rt[0,3],rt[1,3],rt[2,3],qx,qy,qz,qw]])
  return vec

def fromRTtoEulerCBA(matrix44):  # RobotRTToRxyzCBA
  half_pi=np.pi/2.0
  A=B=C=0
  req_limit=1e-8
  if (abs(matrix44[2,0]+1)<=req_limit or abs(matrix44[2,0]-1)<=req_limit):
    # cosB = 0 sinB = pm1 
    A=0;
    if (abs(matrix44[2,0]-1)<=req_limit): B=-half_pi
    if (abs(matrix44[2,0]+1)<=req_limit): B=half_pi
    C=math.atan2(-matrix44[0,1],matrix44[1,1])
  else:
    B=math.asin(-1*matrix44[2,0])
    C=math.atan2(matrix44[1,0],matrix44[0,0])
    A=math.atan2(matrix44[2,1],matrix44[2,2])
  Rx=matrix44[0,3]
  Ry=matrix44[1,3]
  Rz=matrix44[2,3]
  A_degree=A/np.pi*180.
  B_degree=B/np.pi*180.
  C_degree=C/np.pi*180.
  vec=np.array([Rx,Ry,Rz,A_degree,B_degree,C_degree,0])
  return vec

def fromRTtoEulerABC(matrix44):  # RobotRTToRxyzABC
  half_pi=np.pi/2.0
  A=B=C=0
  req_limit=1e-8
  if (abs(matrix44[0,2]+1)<=req_limit or abs(matrix44[0,2]-1)<=req_limit):
    # cosB = 0 sinB = pm1 
    A=0
    if (abs(matrix44[0,2]-1)<=req_limit): B=half_pi
    if (abs(matrix44[0,2]+1)<=req_limit): B=-half_pi
    C=math.atan2(matrix44[1,0],matrix44[1,1])
  else:
    B=math.asin(matrix44[0,2])
    C=math.atan2(-matrix44[0,1], matrix44[0,0])
    A=math.atan2(-matrix44[1,2], matrix44[2,2])
  Rx=matrix44[0,3]
  Ry=matrix44[1,3]
  Rz=matrix44[2,3]
  A_degree=A/np.pi*180.
  B_degree=B/np.pi*180.
  C_degree=C/np.pi*180.
  vec=np.array([Rx,Ry,Rz,A_degree,B_degree,C_degree,0])
  return vec

def inv(tf):
  RT=toRT(tf)
  return fromRT(RT.I)

def fromVec(vec):
  tf=Transform()
  tf.translation.x=vec[0]
  tf.translation.y=vec[1]
  tf.translation.z=vec[2]
  tf.rotation.x=vec[3]
  tf.rotation.y=vec[4]
  tf.rotation.z=vec[5]
  tf.rotation.w=vec[6]
  return tf

if __name__ == '__main__':
  vec=np.array([350, 0, 415, 0.35330038553 ,0.712956175793, -0.542718550391, -0.268940335466])
  tf=fromVec(vec)
  print tf
  RT=toRT(tf)
  print fromRT(RT)
  TR=RT.I
  ft=fromRT(TR)
  print ft
