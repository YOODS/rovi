#!/bin/bash

export NODE_PATH=/usr/lib/node_modules
export PYTHONPATH=/usr/local/lib/python2.7/dist-packages:$PYTHONPATH
source /opt/ros/kinetic/setup.bash
cd ~/*/src/rovi
cd ../../devel
source setup.bash
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
export QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms

$1 $2 $3 $4 $5 $6 $7 $8

sleep 3 

