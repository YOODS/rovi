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

# 3.設定

## 環境変数を追加  
bashrcに以下を追加
1. ハードウェアアクセラレーションをOFFにする（Intelのグラフィックチップを使っている場合に実施した方が良いみたい）
~~~
export LIBGL_ALWAYS_SOFTWARE=1
~~~
2. rvizが重くなる現象への対応
~~~
export ORGE_RTT_MODE=Copy
~~~
## GigEインターフェース設定
- アドレス設定  
YCAM3Dの出荷時IPアドレスは*192.168.222.10*となっています。PC側のインタフェースもそれに合わせます。例えばPC側を*192.168.222.100*とした時の主な設定は以下のようになります。

<table>
<tr><th>項目<td>値<td>備考
<tr>><td>アドレス<td>192.168.222.100
<tr>><td>ネットマスク<td>24
<tr>><td>ゲートウェイ<td><td>設定不要
<tr>><td>MTU<td>9000<td>Jumboパケット対応のため
</table>

- 確認  
YCAM3Dを接続し、PCからYCAM3DのIPアドレスへpingが通ることを確認します。

~~~
ping 192.168.222.10
~~~
- 通信バッファ等設定  
/etc/sysctl.confに以下を追加します。
~~~
net.ipv4.tcp_tw_recycle = 1
net.ipv4.tcp_fin_timeout = 10
net.core.rmem_max = 67108864
net.core.rmem_default = 5000000
net.core.netdev_max_backlog = 1000000
net.core.netdev_budget = 600
~~~
通常アカウントでは編集権限がないので
~~~
sudo vi /etc/sysctl.conf
~~~
などで行う。

# ROSパッケージのインストール
## camera_aravis (GigE Vision Cameraドライバ) のインストール  
### libaravisのインストール  
- 準備
~~~
sudo apt-get install automake intltool
sudo apt-get install libgstreamer*-dev
~~~
- ソースをダウンロード
~~~
wget http://ftp.gnome.org/pub/GNOME/sources/aravis/0.6/aravis-0.6.0.tar.xz
~~~
- ビルド
~~~
tar xvf aravis-0.6.0.tar.xz
cd aravis-0.6
./configure
make
sudo make install
~~~

- 動作確認
~~~
arv-tool-0.6
~~~
正常では、以下のようにデバイスのIDが表示されればOKです。
~~~
YOODS Co,LTD.-YCAM3D-III- (192.168.222.10)
~~~
以下のエラーになるとき
~~~
arv-tool-0.6: error while loading shared libraries: libaravis-0.6.so.0: cannot open shared object file: No such file or direcory
~~~
/etc/ld.so.confに
~~~
/usr/local/lib
~~~
の行を追加して以下を実行
~~~
sudo ldconfig
~~~
### camera_aravisのビルド  
- ソースビルド  
ソースをチェックアウト
~~~
cd ~/catkin_ws/src
git clone https://github.com/YOODS/camera_aravis.git
~~~
つづいてビルド
~~~
cd ~/catkin_ws
catkin_make
~~~

## RoVIのインストール

### RoVIソースのチェックアウト 
~~~
cd ~/catkin_ws/src
git clone -b israfel https://github.com/YOODS/rovi.git
~~~
次に必要なソフトウェアをインストールします。

### Eigenのインストール

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

### Nodejsとパッケージのインストール

Ver8以上が必要です。インストールされていないときは以下にてインストールします (Ver9)。 
~~~
cd ~
curl -sL https://deb.nodesource.com/setup_9.x | sudo -E bash -
sudo apt-get install nodejs
~~~

- Nodejsパッケージのインストール  
必要なパッケージ
<table>
<tr><td>パッケージ名<td>備考
<tr><td>rosnodejs<td>インストール後、追加の処理があります
<tr><td>js-yaml<td>
<tr><td>mathjs<td>
<tr><td>shm-typed-array<td>
<tr><td>terminate<td>
</table>

インストール
~~~
npm install rosnodejs
npm install js-yaml
npm install mathjs
npm install shm-typed-array
npm install terminate
~~~

- rosnodejsインストール後の追加の処理  
*上記でインストールされるrosnodejsは、
RoVIで必要とする機能(ROS Serviceの同期呼び出し機能)が含まれていない。  
rosnodejsの最新Gitソースにはその機能が含まれているので、
以下のようにして、最新Gitソースで上書きする。*

~~~
cd ~
git clone https://github.com/RethinkRobotics-opensource/rosnodejs
cd ~/node_modules/rosnodejs
rm -rf dist
cp -a ~/rosnodejs/src/ dist
~~~

### Pythonパッケージのインストール
必要なパッケージ
<table>
<tr><td>パッケージ名<td>備考
<tr><td>Scipy<td>
<tr><td>Open3D<td>
</table>

インストール
~~~
sudo apt install python-pip
pip install pip==9.0.3 --user
pip install scipy --user
pip install open3d-python --user
~~~

### ビルド

~~~
cd ~/catkin_ws
catkin_make
~~~

------

# RoVIの実行手順

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
Rvizなど一式起動します
~~~
roscd rovi/QC
roslaunch perf3D.launch
~~~
撮像は、サービスコール
~~~
rosservice call /rovi/pshift_genpc
~~~
またはトピックに発行
~~~
rostopic pub -1 /rovi/X1 std_msgs/Bool True
~~~
にて行います。


rqtから*/rovi/X1*で右クリックし**publish once**を選んでもよい

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
<tr><td>/rovi/ycam_ctrl/errlog<td>Strinfg<td>Errorログ
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
