#!/usr/bin/python

import cv2
import numpy as np
import roslib
import rospy
from geometry_msgs.msg import Transform


rospy.init_node("solver",anonymous=True)
Pmat=rospy.get_param('/rovi/left/remap/P')
Pmat=np.reshape(Pmat,(3,4))
Kmat=rospy.get_param('/rovi/left/remap/K')
Kmat=np.reshape(Kmat,(3,3))

print Pmat
print Kmat

cameraMatrix, rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ, eulerAngles=cv2.decomposeProjectionMatrix(Pmat,Kmat)

print "R=",rotMatrix
print "T=",transVect
print "C=",cameraMatrix