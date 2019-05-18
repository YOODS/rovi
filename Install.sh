#!/bin/bash

source /opt/ros/kinetic/setup.bash
cd ~/*/src/rovi
cd ../../
source devel/setup.bash

#installing aravis library
cd ~
wget http://ftp.gnome.org/pub/GNOME/sources/aravis/0.6/aravis-0.6.0.tar.xz
tar xvf aravis-0.6.0.tar.xz
cd aravis-0.6.0
./configure
make
sudo make install
if ! grep /usr/local/lib /etc/ld.so.conf
then
    echo >>/etc/ld.so.conf
    echo /usr/local/lib >>/etc/ld.so.conf
    sudo ldconfig
fi

#installing ros::camera_aravis
cd ~/*/src/rovi
cd ../
git clone https://github.com/YOODS/camera_aravis.git

#installing Eigen
cd ~/*/src/rovi
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
pip install pip==9.0.3
pip install scipy --user
pip install wheel
pip install ipython==5.7 --user
pip install ipykernel==4.10 --user
pip install open3d-python --user

#coping files
cd ~/*/src/rovi
sudo cp launch/rovirun.sh /usr/local/bin
cp script/tflib.py ../../devel/lib/python2*/dist-packages

#build
cd ~/*/src/rovi
cd ../../
catkin_make


