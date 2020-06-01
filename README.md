# RoVIについて

RoVIは3Dビジョンセンサーを組み込んだロボットのアプリケーションを開発するための、ソフトウェア群を提供します。
## 構成
下図にRoVIのソフトウェア構成を示します。コア部(I/O,Base)とUtilityをプラットフォーム層として、これを利用し用途に合わせてApplicationを開発します。基本的な機能はプラットフォーム層を利用できるので、アプリケーションの開発L/Tが短縮出来ます。  
またApplication実装例として、MTM(Master Teaching Method)パッケージも公開予定です(https://github.com/YOODS/MTM)。
<img src="img/fig1.png" width="500px" >
   
## このRepositoryについて  
このRepositoryはYOODS社が提供するYCAM3Dを制御するソフトウェアです。

## ツール・ライブラリの追加  
以下のソフトウェアが未だインストールされていなければ追加します。
### ビルド環境
  - g++
  - git
  - automake
  - intltool
  - libgsreamer*-dev
~~~
sudo apt install g++ git automake intltool libgstreamer*-dev
~~~

### Nodejs  
Node8以上。以下はNode9のインストール方法です。
~~~
cd ~
curl -sL https://deb.nodesource.com/setup_9.x | sudo -E bash -
sudo apt-get install nodejs
~~~

### Pythonパッケージャ  
pipがなければまずpipをインストール
~~~
sudo apt install python-pip python-dev
~~~
pipのversionは9.0.1以上が必要になる。低い場合はアップデートする。
~~~
pip install pip==9.0.3 --user
~~~

### ROSアカウントの初期設定  
.bashrcの最後に以下を入れる(Linux環境。Winは違うかも・・・)
~~~
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
export NODE_PATH=/usr/lib/node_modules
export PYTHONPATH=/usr/local/lib/python2.7/dist-packages:$PYTHONPATH
~~~
反映させるため、一旦ターミナルを閉じ、新しいターミナルを開く。

## RoVIのインストール  

ROSのソースディレクトリ(~/catkin_ws/srcなど)に、israfelブランチをcheckoutします。
~~~
git clone -b israfel --depth 1 https://github.com/YOODS/rovi.git
~~~
GetingStarted.mdに記載のとおりです。この記載内容をInstall.shにて一括処理することも可能です(歓迎スクリプトの間違い指摘)。
~~~
cd rovi
./Install.sh
~~~

## 起動

1. SXGAモード  
~~~
roslaunch rovi ycam3sxga.launch
~~~

2. VGAモード  
~~~
roslaunch rovi ycam3vga.launch
~~~
長さの単位をmmとする場合は以下のlaunchを使います。

3. SXGAモード(mm単位)  
~~~
roslaunch rovi ycam3sxga_mm.launch
~~~

4. VGAモード(mm単位)  
~~~
roslaunch rovi ycam3vga_mm.launch
~~~

## Topics
### To publish
<table>
<tr><th>Name<th>Type<th>Description
<tr><td>/rovi/ps_floats<td>Numpy<td>3DデータNumpy形式
<tr><td>/rovi/ps_pc<td>PointCloud<td>3DデータPointCloud形式
<tr><td>/rovi/left/image_raw<td>Image<td>左カメラraw画像
<tr><td>/rovi/left/image_rect<td>Image<td>左カメラrectify画像
<tr><td>/rovi/left/image_rect0<td>Image<td>左カメラrectify画像(ストロボOFF)
<tr><td>/rovi/right/image_raw<td>Image<td>右カメラraw画像
<tr><td>/rovi/right/image_rect<td>Image<td>右カメラrectify画像
<tr><td>/rovi/right/image_rect0<td>Image<td>右カメラrectify画像(ストロボOFF)
<tr><td>/error<td>String<td>Errorログ
<tr><td>/rovi/stat<td>Bool<td>接続状態
<tr><td>/rovi/Y1<td>Bool<td>撮像結果(X1に対するレスポンス)
</table>

### To subscribe
<table>
<tr><th>Name<th>Type<th>Description
<tr><td>/rovi/X1<td>Bool<td>撮像トリガ
</table>

### Parameters

#### Yaml file
<table>
<tr><th>Name<th>Description
<tr><td>ycam3vga.yaml<td>VGAモードパラメータファイル
<tr><td>ycam3sxga.yaml<td>SXGAモードパラメータファイル
<tr><td>param.yaml<td>オーバライドパラメータファイル
</table>

#### List
<table>
<tr><th>Name<th>Description<td>Type<td>Range
<tr><td>/rovi/camera/address<td>YCAMのIPアドレス
<tr><td>/rovi/genpc/Q<td>Qマトリクス<td>float[16]
<tr><td>/rovi/left/remap/D<td>左カメラDマトリクス<td>float[5]
<tr><td>/rovi/left/remap/K<td>左カメラKマトリクス<td>float[9]
<tr><td>/rovi/left/remap/Kn<td>PマトリクスからdecomposeしたKマトリクス。remap_nodeが算出<td>float[9]
<tr><td>/rovi/left/remap/P<td>左カメラPマトリクス<td>float[12]
<tr><td>/rovi/left/remap/R<td>左カメラRマトリクス<td>float[9]
<tr><td>/rovi/left/remap/height<td>イメージの高さ<td>int
<tr><td>/rovi/left/remap/width<td>イメージの幅<td>int
<tr><td>/rovi/live/camera/AcquisitionFrameRate<td>フレームレート<td>int<td>28
<tr><td>/rovi/live/camera/ExposureTime<td>露光時間(&micro;s)<td>int<td>5000
<tr><td>/rovi/live/camera/Gain<td>カメラゲイン<td>int<td>50
<tr><td>/rovi/live/camera/GainAnalog<td>アナログゲイン<td>int<td>3
<tr><td>/rovi/live/camera/SoftwareTriggerRate<td>ストリーミング時フレームレート<br>int<td>4
<tr><td>/rovi/pshift_genpc/calc/bw_diff<td><td>int
<tr><td>/rovi/pshift_genpc/calc/brightness<td><td>int
<tr><td>/rovi/pshift_genpc/calc/darkness<td><td>int
<tr><td>/rovi/pshift_genpc/calc/step_diff<td><td>float
<tr><td>/rovi/pshift_genpc/calc/max_step<td><td>float
<tr><td>/rovi/pshift_genpc/calc/max_ph_diff<td><td>float
<tr><td>/rovi/pshift_genpc/calc/max_tex_diff<td><td>float
<tr><td>/rovi/pshift_genpc/calc/min_parallax<td><td>float
<tr><td>/rovi/pshift_genpc/calc/right_dup_cnt<td><td>int
<tr><td>/rovi/pshift_genpc/calc/ls_points<td><td>int
<tr><td>/rovi/pshift_genpc/calc/depth_unit<td>デプス画像の1ビットと実距離の比<td>int<td>1
<tr><td>/rovi/pshift_genpc/calc/depth_base<td>デプス画像の0に相当する実距離<td>int<td>400
<tr><td>/rovi/pshift_genpc/camera/ExposureTime<td>位相シフト時露光時間(&micro;s)<td>int<td>8400
<tr><td>/rovi/pshift_genpc/camera/Gain<td>位相シフト時カメラゲイン<td>int<td>0
<tr><td>/rovi/pshift_genpc/projector/ExposureTime<td>発光時間(ms)<td>int<td>20
<tr><td>/rovi/pshift_genpc/projector/Intencity<td>発光強度<td>byte<td>150
<tr><td>/rovi/pshift_genpc/projector/Interval<td>S発光間隔(ms)<br>int<td>50
<tr><td>/rovi/pshift_genpc/projector/Mode<td>プロジェクタモード(1:位相シフト、2:ストロボ,
3:マーカ)<td>int<td>1
<tr><td>/rovi/right/remap/D<td>右カメラDマトリクス<td>float[5]
<tr><td>/rovi/right/remap/K<td>右カメラKマトリクス<td>float[9]
<tr><td>/rovi/right/remap/Kn<td>PマトリクスからdecomposeしたKマトリクス。remap_nodeが算出<td>float[5]
<tr><td>/rovi/right/remap/P<td>右カメラPマトリクス<td>float[12]
<tr><td>/rovi/right/remap/R<td>右カメラRマトリクス<td>float[9]
<tr><td>/rovi/right/remap/height<td>イメージの高さ<td>int
<tr><td>/rovi/right/remap/width<td>イメージの幅<td>int
</table>

## ドキュメントリスト  
|ドキュメント名|コンテンツ|
|:----|:----|
|[GettingStarted.md](GettingStarted.md)|インストール手順についての記載|
|[GettingStarted-TX2.md](GettingStarted-TX2.md)|Jetson-TX2用インストール手順|
|[AppDesign.md](AppDesign.md)|roviを利用したアプリケーションの設計について|
|[CameraDriver.md](CameraDriver.md)|カメラドライバーインタフェースについて|
|[TopicsDetail.md](TopicsDetail.md)|トピック詳細|

## 動作確認済み環境  
|OS|Desktop|ROS|STATUS|
|:----|:----|:----|:----|
|Ubuntu 16.04(xenial)|Unity|kinetic|OK|
|LinuxMint 18.04(sonya)|Xfce|kinetic|OK|
