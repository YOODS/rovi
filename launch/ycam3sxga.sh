#!/bin/bash

export NODE_PATH=/usr/lib/node_modules
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
export PYTHONPATH=/usr/local/lib/python2.7/dist-packages:$PYTHONPATH

roscd rovi
ROS_NAMESPACE=/rovi rosparam load yaml/ycam3sxga.yaml
script/gvloadSXGA.js
roslaunch launch/ycam3s.launch
pkill camnode
echo -n 'y' | rosnode cleanup
