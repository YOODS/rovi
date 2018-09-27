# World Robot Summit 2018


## 事前準備
~~~
cd ~/catkin_ws/src

# ここで、YOODS社内の
# Nextcloud\NEDO\UnitTest\robot_model\
# を
# ~/catkin_ws/src/robot_model/
# として配置する

cd ..
catkin_make
~~~

## 起動方法
~~~
roslaunch rovi r_coord_publisher.launch
# ここでロボットコントローラの電源を入れる。
# その後、ロボットコントローラの電源OFF, ONは自由に行っていいが、
# その間、上記r_coord_publisher.launchはずっと立ち上げておくこと。
# もしもr_coord_publisher.launchを落とした場合は、再度立ち上げて、
# ロボットコントローラの電源をOFF->ONすること。
# (ロボット側CURR.prgが切断検知をうまくしてくれないことに対する手順回避。)
roslaunch rovi wrs2018.launch
~~~

## 動作確認
~~~
~/catkin_ws/src/rovi/r_coord_publisher/test_r_coord_publisher.js
~~~
により、RViz上のロボットが動くことを確認できる。

~~~
rosservice call /rovi/pshift_genpc
~~~
で位相シフト撮影して生成された点群がRViz上に重ね描きされることも確認できる。  
(ただしその位置は適当に固定で座標変換している。)
