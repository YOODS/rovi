#!/usr/bin/python

import cv2
import numpy as np
import roslib
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.srv import compute_effector_camera_quick
from visp_hand2eye_calibration.msg import TransformArray
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../script'))
import tflib

def cb_robot(tf):
  global bTm
  bTm=tflib.toRT(tf)
  mTs=np.dot(np.dot(bTm.I,bTc),cTs)
  pb_mTs.publish(tflib.fromRT(mTs))
  return

def cb_X0(f):
  global cTsAry,mTbAry,bTmAry
  print "cbX0"
  cTsAry=TransformArray()
  mTbAry=TransformArray()
  bTmAry=TransformArray()

def cb_X1(f):
  global cTsAry,mTbAry,bTmAry
  tf=rospy.wait_for_message('/gridboard/tf',Transform)
  print "cbX1::grid",tf
  cTsAry.transforms.append(tf)
  tf=rospy.wait_for_message('/robot/tf',Transform)
  print "cbX1::robot",tf
  bTmAry.transforms.append(tf)
  mTbAry.transforms.append(tflib.inv(tf))
  return

def save(name):
  Tcsv=np.array([]).reshape((-1,21))
  for M,B,S in zip(bTmAry.transforms,mTbAry.transforms,cTsAry.transforms):
    bts=tflib.fromRTtoVec( np.dot(np.dot(tflib.toRT(M),mTc),tflib.toRT(S)) )
    mts=tflib.fromRTtoVec( np.dot(np.dot(tflib.toRT(B),bTc),tflib.toRT(S)) )
    alin=np.hstack((tflib.fromRTtoVec(tflib.toRT(M)),bts,mts))
    Tcsv=np.vstack((Tcsv,alin))
  np.savetxt(name,Tcsv)
  return

def cb_X2(f):
  global cTsAry,mTbAry,bTmAry
  global bTc,mTc
  print dir(compute_effector_camera_quick)
  rospy.wait_for_service('/compute_effector_camera_quick')
  calibrator=None
  try:  #solving as fixed camera
    calibrator=rospy.ServiceProxy('/compute_effector_camera_quick',compute_effector_camera_quick)
  except rospy.ServiceException, e:
    print 'Visp call failed:'+e
    pb_Y2(Bool())  #return false
    return
  
  req=compute_effector_camera_quick.Request()
  res=compute_effector_camera_quick.Response()

  req.camera_object=cTsAry
  req.world_effector=mTbAry
  try:  #solving as fixed camera
    calibrator(req,res)
    print "calib fixed",res.effector_camera
    rospy.set_param('/robot/calib/bTc',res.effector_camera)
    bTc=tflib.toRT(res.effector_camera)
  except rospy.ServiceException, e:
    print 'Visp call failed:'+e
    pb_Y2(Bool())  #return false
    return

  req.camera_object=cTsAry
  req.world_effector=bTmAry
  try:
    calibrator(req,res)
    print "calib handeye",res.effector_camera
    rospy.set_param('/robot/calib/mTc',res.effector_camera)
    mTc=tflib.toRT(res.effector_camera)
  except rospy.ServiceException, e:
    print 'Visp call failed:'+e
    pb_Y2(Bool())  #return false
    return

  f=Bool()
  f.data=True
  pb_Y2(f)
  save('result.txt')
  return

def cb_X3():
  Tcsv=np.array([]).reshape((-1,21))
  for M,B,S in zip(bTmAry.transforms,mTbAry.transforms,cTsAry.transforms):
    bts=tflib.fromRTtoVec( np.dot(np.dot(tflib.toRT(M),mTc),tflib.toRT(S)) )
    mts=tflib.fromRTtoVec( np.dot(np.dot(tflib.toRT(B),bTc),tflib.toRT(S)) )
    print bts,mts
    alin=np.hstack((tflib.fromRTtoVec(tflib.toRT(M)),bts,mts))
    print Tcsv.shape,alin.shape
    Tcsv=np.vstack((Tcsv,alin))
  np.savetxt('result.txt',Tcsv)
  return

###############################################################
rospy.init_node('solver',anonymous=True)

pb_bTs=rospy.Publisher('/solver/bTs',Transform,queue_size=1)
pb_mTs=rospy.Publisher('/solver/mTs',Transform,queue_size=1)
pb_Y2=rospy.Publisher('/solver/Y2',Bool,queue_size=1)    #X2 done

cb_X0(Bool())
bTm=np.eye(4,dtype=float)
cTs=np.eye(4,dtype=float)
bTc=np.eye(4,dtype=float)
mTc=np.eye(4,dtype=float)
if rospy.has_param('/robot/calib/bTc'):
  bTc=tflib.toRT(tflib.dict2tf(rospy.get_param('/robot/calib/bTc')))
if rospy.has_param('/robot/calib/mTc'):
  mTc=tflib.toRT(tflib.dict2tf(rospy.get_param('/robot/calib/mTc')))

rospy.Subscriber('/robot/tf',Transform,cb_robot)
rospy.Subscriber('/solver/X0',Bool,cb_X0)
rospy.Subscriber('/solver/X1',Bool,cb_X1)
rospy.Subscriber('/solver/X2',Bool,cb_X2)

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
