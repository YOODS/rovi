#!/usr/bin/python

import numpy as np
import cv2
import roslib
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rovi.msg import Floats

MaxPI=4.5
MaxSide=3
MinApex=50

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def pickCircles(contours):
  cs=[]
  ps=[]
  for c in contours:
    M=cv2.moments(c)
    alen=cv2.arcLength(c,True)
    area=M['m00'] #cv2.contourArea(c)
    if len(c)>MinApex and area>0.0:
      pi=alen**2/4/area
      if pi<MaxPI and alen/len(c)<MaxSide:
        cs.append(c)
        ps.append(pi)
  if len(cs)>=2:
    idx=np.asarray(ps).argsort()
    return [cs[i] for i in idx]
  else: return cs

def cb_image(rosimg):
  global sb_im
  sb_im.unregister()
  try:
    img0=bridge.imgmsg_to_cv2(rosimg, "mono8")
  except CvBridgeError, e:
    print e
    return
  img1=cv2.bilateralFilter(img0,9,75,75)
  ret,img2=cv2.threshold(img1,0,255,cv2.THRESH_BINARY|cv2.THRESH_OTSU)
  img3,cont1,hierarchy=cv2.findContours(img2,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
  imgc=cv2.cvtColor(img0,cv2.COLOR_GRAY2BGR)
  c2=pickCircles(cont1)
#  cv2.drawContours(imgc,cont1,-1,(255,0,0),2)
  if len(c2)>=2:
    e0=cv2.fitEllipse(c2[0])
    e1=cv2.fitEllipse(c2[1])
    d=np.array(e1[0])-np.array(e0[0])
    if np.abs(d[0])>np.abs(d[1]):
      if d[0]>0:
        cen=np.vstack((e0[0],e1[0]))
        cv2.ellipse(imgc,e0,(255,0,0),4)
        cv2.ellipse(imgc,e1,(0,170,255),4)
      else:
        cen=np.vstack((e1[0],e0[0]))
        cv2.ellipse(imgc,e1,(255,0,0),4)
        cv2.ellipse(imgc,e0,(0,170,255),4)
    else:
      if d[1]>0:
        cen=np.vstack((e0[0],e1[0]))
        cv2.ellipse(imgc,e0,(255,0,0),4)
        cv2.ellipse(imgc,e1,(0,170,255),4)
      else:
        cen=np.vstack((e1[0],e0[0]))
        cv2.ellipse(imgc,e1,(255,0,0),4)
        cv2.ellipse(imgc,e0,(0,170,255),4)
    pb_mk.publish(np2F(cen))
  pb_im.publish(bridge.cv2_to_imgmsg(imgc,"rgb8"))
  sb_im=rospy.Subscriber('image_rect',Image,cb_image)    

###############################################################
rospy.init_node('marker',anonymous=True)
bridge=CvBridge()
pb_im=rospy.Publisher('marker/image',Image,queue_size=1)
sb_im=rospy.Subscriber('image_rect',Image,cb_image)
pb_mk=rospy.Publisher("marker/pos",numpy_msg(Floats),queue_size=1)

#rospy.Subscriber("/rovi/ps_floats",numpy_msg(Floats),cb_ps)

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
