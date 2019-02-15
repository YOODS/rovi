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
## ROS
- Kinetic
## Nodejs
- Ver8以上

# 3.設定

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

### Eigen
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

### Nodejs

Ver8以上が必要です。インストールされていないときは以下にてインストールします (Ver9)。 
~~~
cd ~
curl -sL https://deb.nodesource.com/setup_9.x | sudo -E bash -
sudo apt-get install nodejs
~~~
- Nodejsパッケージのインストール  
必要なパッケージ
<table>
<tr><td>パッケージ名<td>インストール方法<td>備考
<tr><td>rosnodejs<td>npm install rosnodejs<td>インストール後、追加の処理があります
<tr><td>js-yaml<td>npm install js-yaml
<tr><td>mathjs<td>npm install mathjs
<tr><td>shm-typed-array<td>npm install shm-typed-array
</table>

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

### Python
必要なパッケージ
<table>
<tr><td>パッケージ名<td>インストール方法
<tr><td>Scipy<td>pip install scipy --user
<tr><td>Open3D<td>pip install python-open3d --user
</table>

### RoVIのソースビルド  
ソースをチェックアウト
~~~
cd ~/catkin_ws/src
git clone https://github.com/YOODS/rovi.git
~~~
ramielブランチを確認
~~~
roscd rovi
git branch
~~~
つづいてビルド
~~~
cd ~/catkin_ws
catkin_make
~~~

------

# RoVIの実行手順

## Launch
1.3M pixelモード
~~~
roslaunch rovi ycam3sxga
~~~
VGAモード
~~~
roslaunch rovi ycam3vga
~~~

## Test
Rvizなど一式起動します
~~~
roscd rovi/QC
roslaunch perf3D.launch
~~~
rqtから*/rovi/X1*で右クリックし**public once**を選ぶと撮像する

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
<tr><td>/rovi/pshift_genpc/projector/Intencity<td>
<tr><td>/rovi/pshift_genpc/projector/Interval<td>
<tr><td>/rovi/right/remap/D<td>
<tr><td>/rovi/right/remap/K<td>
<tr><td>/rovi/right/remap/Kn<td>
<tr><td>/rovi/right/remap/P<td>
<tr><td>/rovi/right/remap/R<td>
<tr><td>/rovi/right/remap/height<td>
<tr><td>/rovi/right/remap/width<td>
</table>
