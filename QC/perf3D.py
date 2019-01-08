#!/usr/bin/python

import cv2
import numpy as np
import math
import roslib
import rospy
import yodpy2
import open3d as o3d
import copy
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_srvs.srv import Trigger,TriggerRequest
from std_msgs.msg import Bool
from std_msgs.msg import String
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../script'))
import tflib
import time

Area=np.array([[-0.15,0.15],[-0.1,0.1],[0.2,0.5]])
Mesh=0.001

def voxel(D):
 ret,pc = yodpy2.normalize(np.ravel(D))
 return pc.reshape((-1,3))

def down_sample(D):
  pcd = o3d.PointCloud()
  pcd.points = o3d.Vector3dVector(D)
  downpcd = o3d.voxel_down_sample(pcd, voxel_size = Mesh)
  P=np.asarray(downpcd.points)
  return P

def crop(P):
  W=(range(len(P)),)
  for axis,area in zip(P.T,Area):
    min=area[0]
    max=area[1]
    if max<min:
      min=area[1]
      max=area[0]
    W=np.intersect1d(W,np.where(axis<max))
    W=np.intersect1d(W,np.where(axis>min))
  print 'where',W
  P=P[W]
  return P

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def cb_ps(msg): #callback of ps_floats
  print 'capt : elapsed={:.3f}'.format((time.time()-stt)*1000)+'[msec]'
  global stt
  P=np.reshape(msg.data,(-1,3))
  print "PC",P.shape
  sttc=time.time()
  P=crop(P)
  print 'crop : elapsed={:.3f}'.format((time.time()-sttc)*1000)+'[msec]',P.shape
  print 'crop : total  ={:.3f}'.format((time.time()-stt)*1000)+'[msec]',P.shape
  sttv=time.time()
#  P=voxel(P)
  P=down_sample(P)
  print 'voxel: elapsed={:.3f}'.format((time.time()-sttv)*1000)+'[msec]',P.shape
  print 'voxel: total  ={:.3f}'.format((time.time()-stt)*1000)+'[msec]',P.shape
  pub_scf.publish(np2F(P))
  print 'total: elapsed={:.3f}'.format((time.time()-stt)*1000)+'[msec]',P.shape
  return

def P0():
  return np.array([]).reshape((-1,3))

def cb_X0(f):
  P=P0()
  pub_scf.publish(np2F(P))
  return

def cb_X1(f):
  global stt
  stt=time.time()
  genpc=None
  try:
    genpc=rospy.ServiceProxy('/rovi/pshift_genpc',Trigger)
    req=TriggerRequest()
    ret=genpc(req)      #will continue to callback cb_ps
    print "genpc result: ",ret
    print 'genpc: elapsed={:.3f}'.format((time.time()-stt)*1000)+'[msec]'
  except rospy.ServiceException, e:
    print 'Genpc proxy failed:'+e
  return

rospy.init_node("perf3D",anonymous=True)
###Input topics
rospy.Subscriber("/rovi/ps_floats",numpy_msg(Floats),cb_ps)
rospy.Subscriber("/solver/X0",Bool,cb_X0)  #Clear
rospy.Subscriber("/solver/X1",Bool,cb_X1)  #Capture
###Output topics
pub_scf=rospy.Publisher("/scene/floats",numpy_msg(Floats),queue_size=1)
###Mesh
#print "mesh",tuple(map(tuple,Area))
#yodpy2.makeMesh(area=tuple(map(tuple,Area)),mesh=Mesh)
###Genpc
stt=-1

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
