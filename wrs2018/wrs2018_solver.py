#!/usr/bin/python

import cv2
import numpy as np
import roslib
import rospy
import yodpy2
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_srvs.srv import Trigger,TriggerRequest
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Point
from geometry_msgs.msg import Transform
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../script'))
import tflib

zLimit=100.0 #to eliminate far points
Tolerance=0.001
Rejection=2.5
Mesh=1
Area=np.array([[1.0e6,-1.0e6],[1.0e6,-1.0e6],[1.0e6,-1.0e6]])

def voxel(D):
  global Area
  chgf=False
  for axis,vox in zip(D.T,Area):
    p1=np.min(axis)
    if vox[0]>p1:
      vox[0]=p1
      chgf=True
    p2=np.max(axis)
    if vox[1]<p2:
      vox[1]=p2
      chgf=True
  area=tuple(map(tuple,Area))
  if chgf is True:
    print "Remeshing",area
    yodpy2.makeMesh(area=area,mesh=Mesh)
  ret,pc = yodpy2.normalize(np.ravel(D))
  return pc.reshape((-1,3))

def P0():
  return np.array([]).reshape((-1,3))

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def np2PC(d):  #numpy to PointCloud
  pc=PointCloud()
  pc.header.frame_id='hand'
  f=np.ravel(d)
  for e in f.reshape((-1,3)):
    p=Point32()
    p.x=e[0]
    p.y=e[1]
    p.z=e[2]
    pc.points.append(p)
  return pc

def cb_load(str):
  global modPn,modMk
  if len(str.data)==0:
    modPn=P0()
    modMk=P0()
    pub_mdf.publish(np2F(modPn))
    pub_mdk.publish(np2PC(modMk))
  else:
    P=cv2.ppf_match_3d.loadPLYSimple(str.data+'.ply',0)
    print 'load PC',P.dtype,P.shape
    modPn=np.vstack((modPn,P))
    pub_mdf.publish(np2F(modPn))
    P=cv2.ppf_match_3d.loadPLYSimple(str.data+'_m.ply',0)
    print 'load Marker',P.dtype,P.shape
    modMk=np.vstack((modMk,P))
    pub_mdk.publish(np2PC(modMk))
  return

def cb_save(str):
  global scnPn,scnMk,modPn,modMk
  modPn=np.copy(scnPn)
  modMk=np.copy(scnMk)
  scnPn=P0()
  scnMk=P0()
  pub_scf.publish(np2F(scnPn))
  pub_mdf.publish(np2F(modPn))
  pub_sck.publish(np2PC(scnMk))
  pub_mdk.publish(np2PC(modMk))
  if len(str.data)>0:
    d=modPn.astype(np.float32)
    print 'save model PC',d.dtype,d.shape
    P=cv2.ppf_match_3d.writePLY(d,str.data+'.ply')
    d=modMk.astype(np.float32)
    print 'save model Marker',d.dtype,d.shape
    P=cv2.ppf_match_3d.writePLY(d,str.data+'_m.ply')
  return

def cb_ps(msg): #callback of ps_floats
  global bTmLat,cPoLat,bTc,scnPn,scnMk
  mTb=np.linalg.inv(bTmLat)
  P=np.reshape(msg.data,(-1,3))
  n,m=P.shape
  print "PointClour In:",n
#  P=P[np.where(P.T[2]<zLimit)]
  P=voxel(P)
  print "PC(camera)",P
  n,m=P.shape
  print "PointClour Out:",n
  P=np.vstack((P.T,np.ones((1,n))))
  P=np.dot(mTb[:3],np.dot(bTc,P)).T
  print "PC",P
  scnPn=np.vstack((scnPn,P))
  pub_scf.publish(np2F(scnPn))
  P=np.vstack((cPoLat.T,[[1]]))
  P=np.dot(mTb[:3],np.dot(bTc,P)).T
  print "Marker",P
  scnMk=np.vstack((scnMk,P))
  pc=np2PC(scnMk)
  pub_sck.publish(pc)
  return

def cb_X0(f):
  global scnPn,scnMk
  print "X0:scene reset"
  scnPn=P0()
  scnMk=P0()
  pub_scf.publish(np2F(scnPn))
  pub_sck.publish(np2PC(scnMk))
  return

def cb_X1(f):
  global bTm,cPo,bTmLat,cPoLat
  bTmLat=np.copy(bTm)
  print "bTm latched",bTmLat
  cPoLat=np.copy(cPo)
  print "cPo latched",cPoLat
  genpc=None
  try:
    genpc=rospy.ServiceProxy('/rovi/pshift_genpc',Trigger)
    req=TriggerRequest()
    genpc(req)      #will continue to callback cb_ps
  except rospy.ServiceException, e:
    print 'Genpc proxy failed:'+e
  return

def cb_X2(f):
  global scnPn,scnMk,modPn,modMk
  print "X2:ICP",modPn.shape,scnPn.shape
  icp=cv2.ppf_match_3d_ICP(100,Tolerance,Rejection,8)
  mp=modPn.astype(np.float32)
  sp=scnPn.astype(np.float32)
  ret,residu,RT=icp.registerModelToScene(mp,sp)
  print "Residue=",residu
  print RT
  pub_tf.publish(tflib.fromRT(RT))
  TR=np.linalg.inv(RT)
  P=np.vstack((scnPn.T,np.ones((1,len(scnPn)))))
  scnPn=np.dot(TR[:3],P).T
  pub_scf.publish(np2F(scnPn))
  return

def cb_X3(f):
  pub_scf.publish(np2F(scnPn))
  pub_mdf.publish(np2F(modPn))
  pub_sck.publish(np2PC(scnMk))
  pub_mdk.publish(np2PC(modMk))
  return

def cb_pos(p):
  global cPo
  cPo=np.array([[p.x,p.y,p.z]])
  return

def cb_tf(tf):
  global bTm
  bTm=tflib.toRT(tf)
  return

def cb_parse(str):
  exec("global zLimit,Tolerance,Rejection\n"+str.data)
  return

########################################################

rospy.init_node("solver",anonymous=True)
###Input topics
rospy.Subscriber("/solver/load",String,cb_load)  #load ply
rospy.Subscriber("/solver/save",String,cb_save)  #save ply
rospy.Subscriber("/robot/tf",Transform,cb_tf)
rospy.Subscriber("/detector/position2",Point,cb_pos)
rospy.Subscriber("/rovi/ps_floats",numpy_msg(Floats),cb_ps)
rospy.Subscriber("/solver/X0",Bool,cb_X0)  #Clear scene
rospy.Subscriber("/solver/X1",Bool,cb_X1)  #Capture position of marker and robot
rospy.Subscriber("/solver/X2",Bool,cb_X2)  #Calc transform
rospy.Subscriber("/solver/X3",Bool,cb_X3)  #redraw
rospy.Subscriber('/solver/parse',String,cb_parse)

###Output topics
pub_tf=rospy.Publisher("/solver/tf",Transform,queue_size=1)
pub_scf=rospy.Publisher("/scene/floats",numpy_msg(Floats),queue_size=1)
pub_sck=rospy.Publisher("/scene/marker",PointCloud,queue_size=1)
pub_mdf=rospy.Publisher("/model/floats",numpy_msg(Floats),queue_size=1)
pub_mdk=rospy.Publisher("/model/marker",PointCloud,queue_size=1)

###Transform
bTc=tflib.toRT(tflib.dict2tf(rospy.get_param('/robot/calib/bTc')))  #Base to Camera
print bTc
bTmLat=np.eye(4).astype(float)  #robot RT when captured
cPoLat=np.array([0,0,0],dtype=float).reshape((-1,3))  #marker coordinate when captured
bTm=np.eye(4).astype(float) 
cPo=np.array([0,0,0],dtype=float).reshape((-1,3))

###Globals
scnPn=P0()  #Scene points
scnMk=P0()  #Scene Marker points
modPn=P0()  #Model points
modMk=P0()  #Model Marker points

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
