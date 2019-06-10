#!/bin/bash

source /opt/ros/kinetic/setup.bash
cd ~/*/src/rovi
cd ../../
source devel/setup.bash

#arrange sysctl.conf
if ! grep "net.core.rmem_max.*67108864" /etc/sysctl.conf
then
    echo "
#Added by rovi/Install.sh
net.ipv4.tcp_tw_recycle = 1
net.ipv4.tcp_fin_timeout = 10
net.core.rmem_max = 67108864
net.core.rmem_default = 5000000
net.core.netdev_max_backlog = 1000000
net.core.netdev_budget = 600
" | sudo tee -a /etc/sysctl.conf
fi

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
    echo "
#Added by rovi/Install.sh
/usr/local/lib
" | sudo tee -a /etc/ld.so.conf
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
cd ~/
npm install rosnodejs
npm install js-yaml
npm install mathjs
npm install shm-typed-array

#patching rosnodejs
cd ~/
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

#checkout rovi_cropper
cd ~/*/src/rovi
cd ..
git clone https://github.com/YOODS/rovi_cropper.git

#checkout rqt_param
cd ~/*/src/rovi
cd ..
git clone -b devel https://github.com/YOODS/rqt_param_manager.git

#build
cd ~/*/src/rovi
cd ../../
catkin_make


