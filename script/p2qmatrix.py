#!/usr/bin/python

import numpy as np
import roslib
import rospy
import sys

try:
  unit=sys.argv[1]
except Exception as e:
	unit="m"

rospy.init_node('p2qmatrix',anonymous=True)

P1=rospy.get_param('/rovi/left/remap/P')
P2=rospy.get_param('/rovi/right/remap/P')
if unit is "m":
  if np.abs(P2[3])>1:
    P2[3]=P2[3]*0.001
    P2[7]=P2[7]*0.001
    P2[11]=P2[11]*0.001
else:
  if np.abs(P2[3])<1:
    P2[3]=P2[3]*1000
    P2[7]=P2[7]*1000
    P2[11]=P2[11]*1000

Q=np.zeros(16)
Q[0]=Q[5]=1
Q[3]=-P1[2] #-cx1
Q[7]=-P1[6] #-cy1
Q[11]=P1[0] #f
Tx=P2[3]/P1[0]
Q[14]=-1/Tx
Q[15]=(P1[2]-P2[2])/Tx #(cx1-cx2)/Tx
Q=Q.tolist()
print type(Q)
rospy.set_param('/rovi/genpc/Q',Q)
