#!/usr/bin/python

import cv2
import numpy as np
import math
import roslib
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.srv import reset,resetRequest,resetResponse,compute_effector_camera_quick,compute_effector_camera_quickRequest,compute_effector_camera_quickResponse
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
  global cTsAry,bTmAry
  print "cbX0"
  cTsAry=TransformArray()
  bTmAry=TransformArray()

def cb_X1(f):
  global cTsAry,bTmAry
  tf=rospy.wait_for_message('/gridboard/tf',Transform)
  print "cbX1::grid",tf
  cTsAry.transforms.append(tf)
  tf=rospy.wait_for_message('/robot/tf',Transform)
  print "cbX1::robot",tf
  bTmAry.transforms.append(tf)
  f=Bool()
  f.data=True
  pb_Y1.publish(f)
  return

def save_input(name):
  print "save input",len(bTmAry.transforms),len(cTsAry.transforms)
  Tcsv=np.array([]).reshape((-1,14))
  for M,S in zip(bTmAry.transforms,cTsAry.transforms):
    btm=tflib.fromRTtoVec(tflib.toRT(M))
    cts=tflib.fromRTtoVec(tflib.toRT(S))
    alin=np.hstack((btm,cts))
    Tcsv=np.vstack((Tcsv,alin))
  np.savetxt(name,Tcsv)
  return

def save_result(name):
  Tcsv=np.array([]).reshape((-1,14))
  for M,S in zip(bTmAry.transforms,cTsAry.transforms):
    bts=tflib.fromRTtoVec( np.dot(np.dot(tflib.toRT(M),mTc),tflib.toRT(S)) )
    mts=tflib.fromRTtoVec( np.dot(np.dot(tflib.toRT(M).I,bTc),tflib.toRT(S)) )
    alin=np.hstack((bts,mts))
    Tcsv=np.vstack((Tcsv,alin))
  np.savetxt(name,Tcsv)
  return

def save_result_mTs(name):
  Tcsv=np.array([]).reshape((-1,7))
  for M,S in zip(bTmAry.transforms,cTsAry.transforms):
    mTb=tflib.toRT(M).I
    cTs=tflib.toRT(S)
    mts=tflib.fromRTtoVec(np.dot(np.dot(mTb,bTc),cTs))
    Tcsv=np.vstack((Tcsv,mts))
  Tn=map(np.linalg.norm,Tcsv.T[:3].T)
  print "Translation error:",max(Tn)-min(Tn)
  np.savetxt(name,Tcsv)
  return

def set_param_tf(name,tf):
  rospy.set_param(name+'/translation/x',tf.translation.x)
  rospy.set_param(name+'/translation/y',tf.translation.y)
  rospy.set_param(name+'/translation/z',tf.translation.z)
  rospy.set_param(name+'/rotation/x',tf.rotation.x)
  rospy.set_param(name+'/rotation/y',tf.rotation.y)
  rospy.set_param(name+'/rotation/z',tf.rotation.z)
  rospy.set_param(name+'/rotation/w',tf.rotation.w)
  return

def call_visp():
  global cTsAry,bTmAry
  global bTc,mTc

  creset=None
  try:
    creset=rospy.ServiceProxy('/reset',reset)
  except rospy.ServiceException, e:
    print 'Visp reset failed:'+e
    return

  calibrator=None
  try:  #solving as fixed camera
    calibrator=rospy.ServiceProxy('/compute_effector_camera_quick',compute_effector_camera_quick)
  except rospy.ServiceException, e:
    print 'Visp call failed:'+e
    return

  creset(resetRequest())
  req=compute_effector_camera_quickRequest()
  req.camera_object=cTsAry
  mTbAry=TransformArray()
  for tf in bTmAry.transforms:
    mTbAry.transforms.append(tflib.inv(tf))
  req.world_effector=mTbAry
  try:  #solving as fixed camera
    res=calibrator(req)
    print "calib fixed",res.effector_camera
    set_param_tf('/robot/calib/bTc',res.effector_camera)
    bTc=tflib.toRT(res.effector_camera)
  except rospy.ServiceException, e:
    print 'Visp call failed:'+e
    return

  creset(resetRequest())
  req=compute_effector_camera_quickRequest()
  req.camera_object=cTsAry
  req.world_effector=bTmAry
  try:
    res=calibrator(req)
    print "calib handeye",res.effector_camera
    set_param_tf('/robot/calib/mTc',res.effector_camera)
    mTc=tflib.toRT(res.effector_camera)
  except rospy.ServiceException, e:
    print 'Visp call failed:'+e
    return

  return

def cb_X2(f):
  global cTsAry,bTmAry
  global bTc,mTc
  save_input('input.txt')
  call_visp()
  save_result_mTs('result.txt')
  return

def xyz2quat(e):
  tf=Transform()
  k = math.pi / 180 * 0.5;
  cx = math.cos(e.rotation.x * k)
  cy = math.cos(e.rotation.y * k)
  cz = math.cos(e.rotation.z * k)
  sx = math.sin(e.rotation.x * k)
  sy = math.sin(e.rotation.y * k)
  sz = math.sin(e.rotation.z * k)
  tf.translation.x=e.translation.x
  tf.translation.y=e.translation.y
  tf.translation.z=e.translation.z
  tf.rotation.x = cy * cz * sx - cx * sy * sz
  tf.rotation.y = cy * sx * sz + cx * cz * sy
  tf.rotation.z = cx * cy * sz - cz * sx * sy
  tf.rotation.w = sx * sy * sz + cx * cy * cz
  return tf

def cb_X3(f):
  global cTsAry,bTmAry
  print "X3"
  Tcsv=np.loadtxt('input.txt')
  bTmAry=TransformArray()
  cTsAry=TransformArray()
  for vec in Tcsv:
#    bTmAry.transforms.append(xyz2quat(tflib.fromVec(vec[0:7])))
    bTmAry.transforms.append(tflib.fromVec(vec[0:7]))
    cTsAry.transforms.append(tflib.fromVec(vec[7:14]))
  print bTmAry.transforms[0]
  print cTsAry.transforms[0]
  call_visp()
  save_result_mTs('result.txt')
  return

###############################################################
rospy.init_node('solver',anonymous=True)

pb_bTs=rospy.Publisher('/solver/bTs',Transform,queue_size=1)
pb_mTs=rospy.Publisher('/solver/mTs',Transform,queue_size=1)
pb_Y1=rospy.Publisher('/solver/Y1',Bool,queue_size=1)    #X1 done
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

#rospy.Subscriber('/robot/tf',Transform,cb_robot)
rospy.Subscriber('/solver/X0',Bool,cb_X0)
rospy.Subscriber('/solver/X1',Bool,cb_X1)
rospy.Subscriber('/solver/X2',Bool,cb_X2)
rospy.Subscriber('/solver/X3',Bool,cb_X3)

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
