#!/usr/bin/python

import cv2
import numpy as np

print(cv2.__version__)

Kmat=np.float32([[1000, 0, 320,],[0, 1000, 240],[0, 0, 1]])
Dmat=np.float32([0, 0, 0, 0, 0])

model=np.float32([[0,0,0],[10,0,0],[0,10,0],[10,10,0]])
scene=np.float32([[320,240],[320+50,240],[320,240+50],[320+50,240+50]])
ret,rvec,tvec=cv2.solvePnP(model, scene, Kmat, Dmat)
print(tvec)
print(rvec)
scene=np.float32([[320+100,240],[320+150,240],[320+100,240+50],[320+150,240+50]])
ret, rvec, tvec = cv2.solvePnP(model, scene, Kmat, Dmat)
print(tvec)
print(rvec)

Kmat=np.float32([[2.42106055e+03,0,640],[0,2.42106055e+03,512],[0,0,1]])
model=np.float32([[0,0,0],[0,-25,0],[-25,0,0],[-25,-25,0]])
scene=np.float32([[668.911, 425.551],[846.419, 426.391],[669.597, 608.408],[847.121, 606.861]])
ret, rvec, tvec = cv2.solvePnP(model, scene, Kmat, Dmat)
print(tvec)
print(rvec)
