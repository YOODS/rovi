# 1.First of all
I will explain YCAM D control software **RoVI** from environment construction to execution in this document.

# 2. System requirements
## CPU
### Intel series
- Core type CPU 2 core or more (4 cores recommended)

### ARM series
-
## NIC
- Jumbo packet conformance

## Momory
- 4 G or more

### OS
- Ubuntu 16.xx
- LinuxMint 18.x

## ROS
- Noetic

## Nodejs
- Ver 8 or higher

# 3. Setting

## GigE interface setting
- Address setting
The factory IP address of YCAM3D is * 192.168.222.10 *. The interface on the PC side is also adjusted accordingly. For example, when setting the PC side to * 192.168.222.100 *, the main settings are as follows.

<table>
<tr> <th> Item <td> Value <td> Remarks
<tr >> <td> Address <td> 192.168.222.100
<tr >> <td> Netmask <td> 24
<tr >> <td> Gateway <td> <td> No setting required
<tr >> <td> MTU <td> 9000 <td> Because it corresponds to Jumbo packet
</table>

- Confirmation  
Connect YCAM3D and confirm that ping passes from PC to YCAM3D IP address.

~~~
ping 192.168.222.10
~~~

- Communication buffer etc. setting
Add the following to /etc/sysctl.conf.

~~~
net.ipv4.tcp_tw_recycle = 1
net.ipv4.tcp_fin_timeout = 10
net.core.rmem_max = 67108864
net.core.rmem_default = 5000000
net.core.netdev_max_backlog = 1000000
net.core.netdev_budget = 600
~~~

You shoud use sudo for editing the file.

~~~
sudo vi /etc/sysctl.conf
~~~

# Installing the ROS package
## Installation of camera_aravis (GigE Vision Camera driver)

### Installing libaravis
- Preparation

~~~
sudo apt-get install automake intltool
sudo apt-get install libgstreamer*-dev
~~~

- Download source

~~~
wget http://ftp.gnome.org/pub/GNOME/sources/aravis/0.6/aravis-0.6.0.tar.xz
~~~

- Build

~~~
tar xvf aravis-0.6.0.tar.xz
cd aravis-0.6
./configure
make
sudo make install
~~~

- Operation check

~~~
arv-tool-0.6
~~~

Normally, it is OK if the ID of the device is displayed as below.

~~~
YOODS Co,LTD.-YCAM3D-III- (192.168.222.10)
~~~

When the following error occurs

~~~
arv-tool-0.6: error while loading shared libraries: libaravis-0.6.so.0: cannot open shared object file: No such file or direcory
~~~

in /etc/ld.so.conf

~~~
/usr/local/lib
~~~

Add the line and do the following

~~~
sudo ldconfig
~~~

### Build camera_aravis
- Source build
Check out the source

~~~
cd ~/catkin_ws/src
git clone https://github.com/YOODS/camera_aravis.git
~~~

Continue build

~~~
cd ~/catkin_ws
catkin_make
~~~

## Install RoVI

### Check out the RoVI source

~~~
cd ~/catkin_ws/src
git clone --depth 1 https://github.com/YOODS/rovi.git
~~~

Confirm ramiel branch

~~~
roscd rovi
git branch
~~~

Next install the required software.

### Install Eigen

~~~
cd ~/catkin_ws/src/rovi
wget http://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz
tar xvzf 3.3.4.tar.gz
mkdir include
mv eigen-eigen-5a0156e40feb/Eigen/ include
rm -rf eigen-eigen-5a0156e40feb/ 3.3.4.tar.gz

cd ~/catkin_ws
catkin_make
~~~

### Installing Nodejs and packages

Ver 8 or higher is required. If it is not installed, install it as follows (Ver9).

~~~
cd ~
curl -sL https://deb.nodesource.com/setup_9.x | sudo -E bash -
sudo apt-get install nodejs
~~~

- Installing the Nodejs package
Required package
<table>
<tr> <td> Package name <td> Installation method <td> Remarks
<tr> <td> rosnodejs <td> npm install rosnodejs <td> There are additional processing after installation
<tr> <td> js-yaml <td> npm install js-yaml<td>
<tr> <td> mathjs <td> npm install mathjs<td>
<tr><td>shm-typed-array<td>npm install shm-typed-array
</table>

- Additional processing after rosnodejs installation
* Rosnodejs installed above does not include the functions required by RoVI (synchronous invocation function of ROS Service).
Since the latest Git source of rosnodejs contains the functions,
overwrite with the latest Git source as follows. *

~~~
cd ~
git clone https://github.com/RethinkRobotics-opensource/rosnodejs
cd ~/node_modules/rosnodejs
rm -rf dist
cp -a ~/rosnodejs/src/ dist
~~~

### Installing the Python package
Required package
<table>
<tr> <td> Package name <td> Installation method
<tr><td>Scipy<td>pip install scipy --user
<tr><td>Open3D<td>pip install open3d-python --user
</table>

### Build

~~~
cd ~/catkin_ws
catkin_make
~~~

------

# RoVI execution procedure

## Start-up
SXGA mode

~~~
roslaunch rovi ycam3sxga.launch
~~~

VGA mode

~~~
roslaunch rovi ycam3vga.launch
~~~

## test

Start up softwares like Rviz etc.

~~~
roscd rovi/QC
roslaunch perf3D.launch
~~~

Call 3D imaging service.

~~~
rosservice call /rovi/pshift_genpc
~~~

Or issue to topic for 3D imaging.

~~~
rostopic pub -1 /rovi/X1 std_msgs/Bool True
~~~

You can call it by right click on */rovi/X1* from rqt and choose **publish once**.

------
# Topics
## To publish
<table>
<tr> <th> Name <th> Type <th> Description
<tr> <td>/rovi/ps_floats <td>Numpy <td>3D data Numpy format
<tr> <td>/rovi/ps_pc <td>PointCloud <td>3D data PointCloud format
<tr> <td>/rovi/left/image_raw <td>Image <td>Left camera raw image
<tr> <td>/rovi/left/image_rect <td>Image <td>Left camera rectify image
<tr> <td>/rovi/right/image_raw <td>Image <td>Right camera raw image
<tr> <td>/rovi/right/image_rect <td>Image <td>Right camera rectify image
<tr> <td>/rovi/ycam_ctrl/errlog <td>Strinfg <td>Error log
<tr> <td>/rovi/ycam_ctrl/stat <td>Bool <td>System state
</table>

## To subscribe
<table>
<tr> <th> Name <th> Type <th> Description
<tr> <td>/rovi/X1 <td>Bool <td>imaging trigger
</table>

# Parameters
Parameter file is less than yaml/
<table>
<tr> <th> Name <th> Description
<tr> <td>/rovi/camera/address <td>
<tr> <td>/rovi/genpc/Q <td>
<tr> <td>/rovi/left/remap/D <td>
<tr> <td>/rovi/left/remap/K <td>
<tr> <td>/rovi/left/remap/Kn <td>
<tr> <td>/rovi/left/remap/P <td>
<tr> <td>/rovi/left/remap/R <td>
<tr> <td>/rovi/left/remap/height <td>
<tr> <td>/rovi/left/remap/width <td>
<tr> <td>/rovi/live/camera/AcquisitionFrameRate <td>Fixed to 28
<tr> <td>/rovi/live/camera/ExposureTime <td>
<tr> <td>/rovi/live/camera/Gain <td>
<tr> <td>/rovi/live/camera/GainAnalog <td>
<tr> <td>/rovi/live/camera/SoftwareTriggerRate <td>
<tr> <td>/rovi/pshift_genpc/calc/brightness <td>
<tr> <td>/rovi/pshift_genpc/calc/bw_diff <td>
<tr> <td>/rovi/pshift_genpc/calc/darkness <td>
<tr> <td>/rovi/pshift_genpc/calc/ls_points <td>
<tr> <td>/rovi/pshift_genpc/calc/max_para <td>
</table>