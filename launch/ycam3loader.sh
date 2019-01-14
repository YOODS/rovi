#!/bin/bash

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

export ROS_NAMESPACE=/rovi

res=$1
if [ "$res" = "" ]
then
  res=sxga
fi
roscd rovi
rosparam load yaml/ycam3$res.yaml
IPADDS=$(rosparam get camera/address)

while :
do
  guid=$(script/gvgetid.js $IPADDS)
  if [ "$guid" != "" ]
  then
    break
  fi
  echo 'ycam3loader::No device'
done

echo "[ycam3loader]" $guid $IPADDS
rosparam set camera/guid "$guid"
script/gvload.js $res "$guid"

roslaunch launch/ycam3s.launch
pkill camnode
echo -n 'y' | rosnode cleanup
