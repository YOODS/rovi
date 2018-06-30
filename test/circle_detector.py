#!/usr/bin/python

import sys
import cv2
import numpy as np
#from scipy import optimize
import roslib
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from rovi.srv import Detect2D,Detect2DResponse
from rovi.srv import Dialog,DialogResponse
from cv_bridge import CvBridge, CvBridgeError

Roi=[33, 33, 66, 66]
Radius=30

def fit_func(parameter,x,y):
  a=parameter[0]
  b=parameter[1]
  r=parameter[2]
  residual=(x-a)**2+(y-b)**2-r**2
  return residual

def gp(dat,radius):
  x=0.0
  y=0.0
  N=len(dat)
  for p in dat:
    q=p[0]
    x+=q[0]
    y+=q[1]
  x=x/N
  y=y/N
  err=0
  rad=0
  for p in dat:
    q=p[0]
    d=np.sqrt((x-q[0])**2+(y-q[1])**2);
    rad+=d
    err+=np.abs(d-radius)
  rad=rad/N
  err=err/N
#  print N,x,y,rad,err
  return x,y,rad,err

def callback(req):
  try:
    img1 = bridge.imgmsg_to_cv2(req.img, "mono8")
  except CvBridgeError, e:
    print e
  imHeight = img1.shape[0]
  imWidth = img1.shape[1]
  imXo = int(imWidth*Roi[0]/100)
  imYo = int(imHeight*Roi[1]/100)
  img2 = img1[imYo:int(imHeight*Roi[3]/100),imXo:int(imWidth*Roi[2]/100)]
#  img2 = cv2.fastNlMeansDenoising(img2,None,9,13)
#  img2 = cv2.bilateralFilter(img2, 15, 20, 20)
  ret,img3 = cv2.threshold(img2,0,255,cv2.THRESH_BINARY|cv2.THRESH_OTSU)
  img4,contours,hierarchy=cv2.findContours(img3,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

  detCont=[]
  detParam=[0,0,0]
  detErr=1000
  for c in contours:
    cx,cy,rad,err=gp(c,Radius)
    if detErr>err:
      detCont=c
      detParam=[cx,cy,rad]
      detErr=err

# tcont=detCont.transpose()
# result=optimize.leastsq(fit_func,detParam,args=(tcont[0][0],tcont[1][0]))
# detParam=[result[0][0],result[0][1],result[0][2]]
  detParam[0]+=imXo
  detParam[1]+=imYo

  print detParam,detErr
  
  img0=cv2.cvtColor(img1,cv2.COLOR_GRAY2BGR)
  cv2.drawContours(img0,[detCont+[imXo,imYo]],-1,(0,255,0),3)
  text = "R=" + str(int(detParam[2]*100)/100)
  font = cv2.FONT_HERSHEY_PLAIN
  cv2.putText(img0,text,(int(detParam[0]),int(detParam[1])),font,3,(255,255,0),2,cv2.LINE_AA)

  try:
    image_pub.publish(bridge.cv2_to_imgmsg(img0, "bgr8"))
  except CvBridgeError, e:
    print e

  scene=[]
  pnt=Point()
  pnt.x=detParam[0]
  pnt.y=detParam[1]
  pnt.z=detErr
  scene.append(pnt)
  model=[]
  return Detect2DResponse(scene,model)

bridge = CvBridge()
image_pub = rospy.Publisher("/circle_detector/image",Image,queue_size=1)
image_svr = rospy.Service("/circle_detector/do", Detect2D, callback)
rospy.init_node("circle_detector", anonymous=True)
try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
