#!/bin/bash
#
# Test program of Robot Calibraion
#
DataDir=/home/ca/robotcalib

rosparam set /gridboard/unitleng 60.0
rosparam set /gridboard/unitmm 5.0
rosparam set /gridboard/K [2.42106055e+03,0.0,640,0.0,2.42106055e+03,512,0.0,0.0,1.0]

if ! rosnode list | grep /grid_node
then
	rosrun rovi grid_node /gridboard/K &
fi

if ! rosnode list | grep /tools_node
then
	rosrun rovi tools_node &
fi

if ! rosnode list | grep /robot_calib
then
	rosrun rovi robot_calib.js &
fi

if ! rosnode list | grep /imsep
then
	rosrun rovi imsep.js /tools/image &
fi

if ! rosnode list | grep /xyz2q
then
	rosrun rovi xyz2q.js /robot_calib/pose &
fi

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

rosservice call /robot_calib/reload
