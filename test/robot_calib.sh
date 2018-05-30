#!/bin/bash
############################################################################################
# Test program of Robot(hand eye) Calibraion
# Hand-eye calibration ends to calcurate the transform to the camera co-ordinate on the robot-end
############################################################################################
DataDir=/home/ca/robotcalib

rosparam set /gridboard/unitleng 60.0
rosparam set /gridboard/unitmm 5.0
rosparam set /gridboard/K [2.42106055e+03,0.0,640,0.0,2.42106055e+03,512,0.0,0.0,1.0]

############################################################################################
#Mandatry Nodes
############################################################################################
if ! rosnode list | grep /grid_node
then
	rosrun rovi grid_node /gridboard/K &
fi

if ! rosnode list | grep /calibrator
then
	rosrun visp_hand2eye_calibration visp_hand2eye_calibration_calibrator
fi

if ! rosnode list | grep /robot_calib
then
	rosrun rovi robot_calib.js &
fi

############################################################################################
#If pose of your robot is shown in ABC degree, its pose should be given thru the node /xyz2q
############################################################################################
if ! rosnode list | grep /xyz2q
then
	rosrun rovi xyz2q.js /robot_calib/pose &
fi

############################################################################################
#The nodes below are needed for off-line testing, or replaying from saved images and poses.
#Their location is given as variable DataDir in this file
############################################################################################
if ! rosnode list | grep /tools_node
then
	rosrun rovi tools_node &
fi

if ! rosnode list | grep /imsep
then
	rosrun rovi imsep.js /tools/image &
fi

############################################################################################
#Try reading data.
#The poses of robot and object will be stored in the node /robot_calib
############################################################################################
cd $DataDir
rosservice call /tools/cd $DataDir
cat robot_xyzabc.txt | for src in calib[01][0-9].pgm
do
	echo "---reading "$src "---"
	read line
	echo $line
	rosservice call /xyz2q "'""$line""'" &
	sleep 1
	rosservice call /tools/imread $src
done

############################################################################################
#The call below calcurates the transform
############################################################################################
rosservice call /robot_calib/reload
