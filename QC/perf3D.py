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
from std_msgs.msg import Bool
from std_msgs.msg import String
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../script'))
import tflib

Area=np.array([[-0.15,0.15],[-0.1,0.1],[0.2,0.5]])
Mesh=0.001

def voxel(D):
 ret,pc = yodpy2.normalize(np.ravel(D))
 return pc.reshape((-1,3))

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
  P=np.reshape(msg.data,(-1,3))
#  pcd = o3d.PointCloud()
#  pcd.points = o3d.Vector3dVector(P)
#  downpcd = o3d.voxel_down_sample(pcd, voxel_size = 0.001)
#  P=np.asarray(downpcd.points)
#  P=P*1000
  print "PC",P.shape
  P=crop(P)
  print 'crop',P.shape
  P=voxel(P)
  print 'voxel',P.shape
  pub_scf.publish(np2F(P))
  return

rospy.init_node("perf3D",anonymous=True)
###Input topics
rospy.Subscriber("/rovi/ps_floats",numpy_msg(Floats),cb_ps)
###Output topics
pub_scf=rospy.Publisher("/scene/floats",numpy_msg(Floats),queue_size=1)
###Mesh
print "mesh",tuple(map(tuple,Area))
yodpy2.makeMesh(area=tuple(map(tuple,Area)),mesh=Mesh)

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
