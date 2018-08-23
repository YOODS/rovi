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

def cb_X2(f):
  global cTsAry,mTbAry,bTmAry
  global bTc,mTc
  print dir(compute_effector_camera_quick)
  req1=compute_effector_camera_quick.Request()
  req1.camera_object=cTsAry
  req1.world_effector=mTbAry
  res1=compute_effector_camera_quick.Response()
  req2=compute_effector_camera_quick.Request()
  req2.camera_object=cTsAry
  req2.world_effector=bTmAry
  res2=compute_effector_camera_quick.Response()
  rospy.wait_for_service('/compute_effector_camera_quick')
  try:
    calibrator=rospy.ServiceProxy('/compute_effector_camera_quick',compute_effector_camera_quick)
    calibrator(req1,res1)
    print "calib",res1.effector_camera
    rospy.set_param('/robot/calib/bTc',res1.effector_camera)
    bTc=tflib.toRT(res1.effector_camera)
    calibrator(req2,res2)
    rospy.set_param('/robot/calib/mTc',res2.effector_camera)
    mTc=tflib.toRT(res2.effector_camera)
    pb_Y2(Bool())
  except rospy.ServiceException, e:
    print 'Service call failed:'+e
  Tcsv=np.array([])
  Tcsv.reshape(-1,21)
  for M,B,S in zip(bTmAry.transforms,mTbAry.transforms,cTsAry.transforms):
    bts=tflib.fromRTtoVec(np.dot(tflib.toRT(M),mTc),tflib.toRT(S)))
    mts=tflib.fromRTtoVec(np.dot(tflib.toRT(B),bTc),tflib.toRT(S)))
    alin=Tcsv,np.hstack((M,bts,mts))
    print alin
    Tcsv=np.vstack((Tcsv,alin))
  np.save('result.csv',Tcsv)
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
  cTb=bTc.I
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
