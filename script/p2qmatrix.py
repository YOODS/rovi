#!/usr/bin/python

import numpy as np
import roslib
import rospy

rospy.init_node('p2qmatrix',anonymous=True)

P1=rospy.get_param('/rovi/left/remap/P')
P2=rospy.get_param('/rovi/right/remap/P')
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
