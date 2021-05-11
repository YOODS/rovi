# 1.はじめに
YCAM3Dの制御ソフトウェア**RoVI**の環境構築から実行までの手順を説明します。

# 2.システム要件
## CPU
### Intel系
- Core系CPU 2コア以上(4コア推奨)
### ARM系
- 
## NIC
- Jumboパケット適合
## Momory
- 4G以上
## OS
- Ubuntu 16.xx
- LinuxMint 18.x
  - ROSインストール時、/etc/apt/sources.list.d/ros-latest.listにubuntuのコードを書かないとエラーになることに注意(以下のxenialの部分)
  
    deb http://packages.ros.org/ros/ubuntu xenial main

## ROS
- Kinetic
## Nodejs
- Ver8以上

# インストール  
## RoVIリポジトリのチェックアウト  

~~~
cd ~/catkin_ws/src
git clone -b devel https://github.com/YOODS/rovi.git

## その他必要なライブラリのインストール  
シェルスクリプトInstall.shにてインストールします。
~~~
./Install.sh
~~~

## ビルド
ROSの通常の手順どおりにビルドします。
~~~
cd ~/catkin_ws
catkin_make
~~~

------

# 実行

## 起動  
！ROSのソースディレクトリが **~/catkin_ws/** 以外のときは、以下のrovi/launch/ycam3loader.shの4行目を変更します。
~~~
#!/bin/bash

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

export ROS_NAMESPACE=/rovi
~~~

1.3M pixelモード
~~~
roslaunch rovi ycam3sxga.launch
~~~
VGAモード
~~~
roslaunch rovi ycam3vga.launch
~~~

## テスト
撮像は、サービスコール
~~~
rosservice call /rovi/pshift_genpc
~~~
またはトピックに発行
~~~
rostopic pub -1 /rovi/X1 std_msgs/Bool True
~~~
にて行います。  

撮影結果点群がトピック(/rovi/ps_pc)に出力されます(下表参照)。

------
# Topics
## To publish
<table>
<tr><th>Name<th>Type<th>Description
<tr><td>/rovi/ps_floats<td>Numpy<td>3DデータNumpy形式
<tr><td>/rovi/ps_pc<td>PointCloud<td>3DデータPointCloud形式
<tr><td>/rovi/left/image_raw<td>Image<td>左カメラraw画像
<tr><td>/rovi/left/image_rect<td>Image<td>左カメラrectify画像
<tr><td>/rovi/right/image_raw<td>Image<td>右カメラraw画像
<tr><td>/rovi/right/image_rect<td>Image<td>右カメラrectify画像
<tr><td>/rovi/ycam_ctrl/stat<td>Bool<td>システム状態
</table>

## To subscribe
<table>
<tr><th>Name<th>Type<th>Description
<tr><td>/rovi/X1<td>Bool<td>撮像トリガ
</table>

# Parameters
パラメータファイルはyaml/以下
<table>
<tr><th>Name<th>Description
<tr><td>/rovi/camera/address<td>
<tr><td>/rovi/genpc/Q<td>
<tr><td>/rovi/left/remap/D<td>
<tr><td>/rovi/left/remap/K<td>
<tr><td>/rovi/left/remap/Kn<td>
<tr><td>/rovi/left/remap/P<td>
<tr><td>/rovi/left/remap/R<td>
<tr><td>/rovi/left/remap/height<td>
<tr><td>/rovi/left/remap/width<td>
<tr><td>/rovi/live/camera/AcquisitionFrameRate<td>28に固定
<tr><td>/rovi/live/camera/ExposureTime<td>
<tr><td>/rovi/live/camera/Gain<td>
<tr><td>/rovi/live/camera/GainAnalog<td>
<tr><td>/rovi/live/camera/SoftwareTriggerRate<td>
<tr><td>/rovi/pshift_genpc/calc/brightness<td>
<tr><td>/rovi/pshift_genpc/calc/bw_diff<td>
<tr><td>/rovi/pshift_genpc/calc/darkness<td>
<tr><td>/rovi/pshift_genpc/calc/ls_points<td>
<tr><td>/rovi/pshift_genpc/calc/max_parallax<td>
<tr><td>/rovi/pshift_genpc/calc/max_ph_diff<td>
<tr><td>/rovi/pshift_genpc/calc/max_step<td>
<tr><td>/rovi/pshift_genpc/calc/min_parallax<td>
<tr><td>/rovi/pshift_genpc/calc/right_dup_cnt<td>
<tr><td>/rovi/pshift_genpc/calc/search_div<td>
<tr><td>/rovi/pshift_genpc/calc/step_diff
<tr><td>/rovi/pshift_genpc/camera/ExposureTime<td>
<tr><td>/rovi/pshift_genpc/camera/Gain<td>
<tr><td>/rovi/pshift_genpc/projector/ExposureTime<td>
<tr><td>/rovi/pshift_genpc/projector/Intensity<td>
<tr><td>/rovi/pshift_genpc/projector/Interval<td>
<tr><td>/rovi/right/remap/D<td>
<tr><td>/rovi/right/remap/K<td>
<tr><td>/rovi/right/remap/Kn<td>
<tr><td>/rovi/right/remap/P<td>
<tr><td>/rovi/right/remap/R<td>
<tr><td>/rovi/right/remap/height<td>
<tr><td>/rovi/right/remap/width<td>
</table>
