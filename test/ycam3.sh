#!/bin/bash
#
# Test program of YCAM-III
#

ROVI_HOME=/home/ca/catkin_ws/src/rovi

ROS_NAMESPACE=/rovi/left rosparam load $ROVI_HOME/yaml/ycam3vga.yaml

if ! rosnode list | grep /remap_node
then
	ROS_NAMESPACE=/rovi/left rosrun rovi remap_node &
	ROS_NAMESPACE=/rovi/right rosrun rovi remap_node &
fi

if ! rosnode list | grep /genpc_node
then
	ROS_NAMESPACE=/rovi rosrun rovi genpc_node &
fi
