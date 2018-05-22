#!/usr/bin/python

import cv2
import numpy as np

#imgL = cv2.imread('left1.pgm',0)
#imgR = cv2.imread('right1.pgm',0)
imgL = cv2.imread('left2.pgm',0)
imgR = cv2.imread('right2.pgm',0)

#stereo=cv2.StereoBM_create(numDisparities=16, blockSize=15)

window_size = 1
stereo=cv2.StereoSGBM_create(
	minDisparity = 0,
	numDisparities = 16,
	blockSize = 3,
	P1 = 8*3*window_size**2,
	P2 = 32*3*window_size**2,
	disp12MaxDiff = 1,
	uniquenessRatio = 1,
	speckleWindowSize = 3,
	speckleRange = 1
)

disparity = stereo.compute(imgL,imgR)
print disparity

min,max,minLoc,maxLoc=cv2.minMaxLoc(disparity)
mat=((disparity-min)*255/(max-min)).astype(np.uint8)
cv2.imshow('depth',mat)
cv2.waitKey(0)
