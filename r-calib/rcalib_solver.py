#!/usr/bin/python

import cv2
import numpy as np
import roslib
import rospy
from std_msgs.msg import Empty
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
  cTs=np.dot(np.dot(cTb,bTm),mTs)
  pb_cTs.publish(tflib.fromRT(cTs))
  return

def cb_X0(f):
  global cTsAry,mTbAry,sTcAry,bTmAry
  print "cbX0"
  cTsAry=TransformArray()
  mTbAry=TransformArray()
  sTcAry=TransformArray()
  bTmAry=TransformArray()

def cb_X1(f):
  global cTsAry,mTbAry,sTcAry,bTmAry
  tf=rospy.wait_for_message('/gridboard/tf',Transform)
  print "cbX1::grid",tf
  cTsAry.transforms.append(tf)
  sTcAry.transforms.append(tflib.inv(tf))
  tf=rospy.wait_for_message('/robot/tf',Transform)
  print "cbX1::robot",tf
  bTmAry.transforms.append(tf)
  mTbAry.transforms.append(tflib.inv(tf))
  return

def cb_X2(f):
  global cTsAry,mTbAry,sTcAry,bTmAry
  global bTc,mTs
  print dir(compute_effector_camera_quick)
  req1=compute_effector_camera_quick.Request()
  req1.camera_object=cTsAry
  req1.world_effector=mTbAry
  res=compute_effector_camera_quick.Response()
  req2=compute_effector_camera_quick.Request()
  req2.camera_object=sTcAry
  req2.world_effector=bTmAry
  res=compute_effector_camera_quick.Response()
  rospy.wait_for_service('/compute_effector_camera_quick')
  try:
    calibrator=rospy.ServiceProxy('/compute_effector_camera_quick',compute_effector_camera_quick)
    calibrator(req1,res)
    print "calib",req1.effector_camera
    rospy.set_param('/robot/calib/bTc',res.effector_camera)
    bTc=tflib.toRT(res.effector_camera)
    calibrator(req2,res)
    rospy.set_param('/robot/calib/mTs',res.effector_camera)
    mTs=tflib.toRT(res.effector_camera)
    pb_Y2(Empty())
  except rospy.ServiceException, e:
    print 'Service call failed:'+e
  return


###############################################################
rospy.init_node('solver',anonymous=True)

pb_cTs=rospy.Publisher('/solver/cTs',Transform,queue_size=1)
pb_Y2=rospy.Publisher('/solver/Y2',Empty,queue_size=1)    #X2 done

cb_X0(Empty())
bTm=np.eye(4,dtype=float)
cTs=np.eye(4,dtype=float)
bTc=np.eye(4,dtype=float)
cTb=np.eye(4,dtype=float)
mTs=np.eye(4,dtype=float)
if rospy.has_param('/robot/calib/bTc'):
  bTc=tflib.toRT(tflib.dict2tf(rospy.get_param('/robot/calib/bTc')))
  cTb=bTc.I
if rospy.has_param('/robot/calib/mTs'):
  mTs=tflib.toRT(tflib.dict2tf(rospy.get_param('/robot/calib/mTs')))

rospy.Subscriber('/robot/tf',Transform,cb_robot)
rospy.Subscriber('solver/X0',Empty,cb_X0)
rospy.Subscriber('solver/X1',Empty,cb_X1)
rospy.Subscriber('solver/X2',Empty,cb_X2)

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
