# World Robot Summit 2018


## 事前準備
~~~
cd ~/catkin_ws/src
git clone https://github.com/YOODS/vs060_moveit_config
cd ..
catkin_make
~~~

## 起動方法
~~~
roslaunch rovi wrs2018.launch
~~~

## 動作確認
~~~
~/catkin_ws/src/rovi/r_coord_publisher/test_r_coord_publisher.js
~~~
により、RViz上のロボットが動くことを確認できる。
