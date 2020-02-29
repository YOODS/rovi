#!/bin/bash

source /opt/ros/kinetic/setup.bash
cd ~/*/src/rovi
cd ../../
source devel/setup.bash

export ROS_NAMESPACE=/rovi

res=$1
if [ "$res" = "" ]
then
  res=sxga
fi
unit=$2
if [ "$unit" = "" ]
then
  unit=m
fi

roscd rovi
rosparam load yaml/ycam3$res.yaml
IPADDS=$(rosparam get camera/address)

while :
do
  id_adds=$(script/gvgetid.js $IPADDS)
  adds=${id_adds#*@@@}
  if [ "$adds" != "" ]
  then
    rosparam set camera/address $adds
    guid=${id_adds%@@@*}
    if [ "$guid" != "" ]
    then
      break
    fi
  fi
  sleep 2
  echo 'ycam3loader::No device'
done

echo "[ycam3loader]" $guid $IPADDS
rosparam set camera/guid "$guid"
script/gvload.js $res "$guid"
script/p2qmatrix.py $unit

roslaunch launch/ycam3s.launch
pkill camnode
echo -n 'y' | rosnode cleanup
