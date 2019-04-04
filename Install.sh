#!/bin/bash

#installing aravis library
cd ~
wget http://ftp.gnome.org/pub/GNOME/sources/aravis/0.6/aravis-0.6.0.tar.xz
tar xvf aravis-0.6.0.tar.xz
cd aravis-0.6
./configure
make
sudo make install

#installing ros::camera_aravis
roscd rovi
cd ../
git clone https://github.com/YOODS/camera_aravis.git

#installing Eigen
roscd rovi
wget http://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz
tar xvzf 3.3.4.tar.gz
mkdir include
mv eigen-eigen-5a0156e40feb/Eigen/ include
rm -rf eigen-eigen-5a0156e40feb/ 3.3.4.tar.gz

#installing nodejs packages
npm install rosnodejs
npm install js-yaml
npm install mathjs
npm install shm-typed-array

#patching rosnodejs
cd ~
git clone https://github.com/RethinkRobotics-opensource/rosnodejs
cd ~/node_modules/rosnodejs
rm -rf dist
cp -a ~/rosnodejs/src/ dist

#installing python package
pip install scipy --user
pip install open3d-python --user

#coping files
roscd rovi
sudo cp launch/rovirun.sh /usr/local/bin
cp script/tflib ../../devel/lib/python2*/dist-packages

#build
roscd rovi
cd ../../
catkin_make
