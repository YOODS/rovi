#!/bin/bash

export NODE_PATH=/usr/lib/node_modules
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
export PYTHONPATH=/usr/local/lib/python2.7/dist-packages:$PYTHONPATH

res=$1
if [ "$res" = "" ]
then
  res=sxga
fi
roscd rovi
ROS_NAMESPACE=/rovi rosparam load yaml/ycam3$res.yaml
IPADDS=$(ROS_NAMESPACE=/rovi rosparam get camera/address)

while ! script/gvload.js $res $IPADDS
do
  echo '--------'
done

roslaunch launch/ycam3s.launch
pkill camnode
echo -n 'y' | rosnode cleanup
