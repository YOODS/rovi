#!/bin/bash

CATKIN_WS=${PWD%src*}

source /opt/ros/melodic/setup.bash
source $CATKIN_WS/devel/setup.bash

#installing aravis library
sudo apt-get install automake intltool
sudo apt-get install libgstreamer*-dev
cd ~
wget http://ftp.gnome.org/pub/GNOME/sources/aravis/0.6/aravis-0.6.4.tar.xz
tar xvf aravis-0.6.4.tar.xz
cd aravis-0.6.4
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

#installing Eigen
sudo apt-get install libeigen3-dev

#installing nodejs and packages
cd ~
curl -sL https://deb.nodesource.com/setup_10.x | sudo -E bash -
sudo apt-get install nodejs
npm install rosnodejs
npm install js-yaml
npm install mathjs
npm install terminate --save
npm install ping

#installing python package
sudo apt install python-pip
pip install pip==9.0.3
pip install numpy==1.15.0 --user
pip install scipy --user
pip install wheel --user
pip install ipython==5.7 --user
pip install ipykernel==4.10 --user
pip install open3d-python --user
