#!/usr/bin/python

import os
import cv2
import numpy as np
import roslib
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

Roi=[0,0,100,100]
Range=30000
Font=1

def cb_image(rosimg):
  global sb_im
  sb_im.unregister()
  try:
    img0 = bridge.imgmsg_to_cv2(rosimg, "mono8")
  except CvBridgeError, e:
    print e
    return
  imHeight = img0.shape[0]
  imWidth = img0.shape[1]
  imXo = int(imWidth*Roi[0]/100)
  imYo = int(imHeight*Roi[1]/100)
  imXe = int(imWidth*Roi[2]/100)
  imYe = int(imHeight*Roi[3]/100)
  img1 = img0[imYo:imYe,imXo:imXe]

  hist,bins = np.histogram(np.ravel(img1),256,[0,256])

  poly=np.vstack(([bins[:256]],[hist]))
  poly[1]/=Range/255
  poly[1]=256-poly[1]
  polt=poly.T.reshape((-1,1,2)).astype(int)

  imgc=cv2.cvtColor(img0,cv2.COLOR_GRAY2BGR)
  cv2.rectangle(imgc, (imXo,imYo), (imXe,imYe), (0,0,128), 1)
  imgh=np.zeros((256,256,3), np.uint8)
  cv2.polylines(imgh,[polt],False,(0,255,255))
  iave=int(np.sum(hist*bins[:256])/(imXe-imXo)/(imYe-imYo))
  bmax=np.max(hist[1:iave])
  wmax=np.max(hist[iave:256])
  bidx=np.where(hist==bmax)[0]
  widx=np.where(hist==wmax)[0]
  font = cv2.FONT_HERSHEY_PLAIN
  text = "I="+str(iave)
  ty=int(Font*12)
#  cv2.putText(imgc,text,(imXo,imYo+ty),font,Font,(255,0,0),1,cv2.LINE_AA)
  cv2.putText(imgh,text,(100,ty),font,Font,(255,0,0),1,cv2.LINE_AA)
  text = "B="+str(bmax)+str(bidx)
  ty+=int(Font*12)
#  cv2.putText(imgc,text,(imXo,imYo+ty),font,Font,(255,0,0),1,cv2.LINE_AA)
  cv2.putText(imgh,text,(100,ty),font,Font,(255,0,0),1,cv2.LINE_AA)
  text="W="+str(wmax)+str(widx)
  ty+=int(Font*12)
  cv2.putText(imgc,text,(imXo,imYo+ty),font,Font,(255,0,0),1,cv2.LINE_AA)
  cv2.putText(imgh,text,(100,ty),font,Font,(255,0,0),1,cv2.LINE_AA)
  pb_img.publish(bridge.cv2_to_imgmsg(imgc, "rgb8"))
  pb_plot.publish(bridge.cv2_to_imgmsg(imgh, "rgb8"))
  sb_im=rospy.Subscriber('image_raw',Image,cb_image)
  return

def cb_parse(str):
  exec("global Range,Font,Roi\n"+str.data)
  print "Range=",Range
  return

###############################################################
rospy.init_node('hist',anonymous=True)
bridge=CvBridge()
pb_img=rospy.Publisher('hist/image',Image,queue_size=1)
pb_plot=rospy.Publisher('hist/plot',Image,queue_size=1)
sb_im=rospy.Subscriber('image_raw',Image,cb_image)
sb_param=rospy.Subscriber('hist/param',String,cb_parse)

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
