# はじめに
このドキュメントでは、RoVIを構築して実行するための手順を記述します。

# 使用ハードウェア
- Jetson TX2 または YJC-4 (ビジョンコントローラ)
- YCAM3D-III (3Dカメラ)
- GigE LANケーブル (…YCAM3D-IIIに付属する専用品…) (ビジョンコントローラと3Dカメラを接続)

# ビジョンコントローラの前提条件
このドキュメントでは、ビジョンコントローラについて、以下を前提条件とします。  
この前提条件を満たしたビジョンコントローラを用意してください。  
(※一部の詳細手順は次節に記述していますので参考にしてください。)

- OSとしてUbuntu 16.04 LTSをインストール済み。
- gitをインストール済み。
- ROS Kineticをインストール済み。
- ROSワークスペースは~/catkin_wsとする。  
(~/catkin_ws以外のROSワークスペースでも可。  
その場合は以下の例を読み替えること。)

以下はすべて、(次節の[参考1]の手順123を除いて)このビジョンコントローラでの作業となります。

## [参考]ビジョンコントローラに関する詳細手順

### [参考1]OSのインストール手順詳細
1. 親機(Ubuntu 16.04.4をインストールしたPC)に JetPack 3.2 をインストールする。  
※その際、 jetpack/64_TX2/Linux_for_Tegrakernel/dtb/tegra186-quill-p3310-1000-c03-00-base.dtbを汎用キャリアボード対応版に変更しておくこと。これをやらないとUSBデバイスに電源が供給されない。 (TODO YOODSから供給)
2. 親機とビジョンコントーラをmicro USBで接続して、リカバリーモードにて以下のコマンドを実施する。(コマンド実行には20分程度かかる。)
~~~
sudo ./flash.sh  jetson-tx2 mmcblk0p1
~~~
3. 上記のflash.shコマンドの終了後、キーボード, マウス, HDMIケーブル, インターネット接続用LANケーブルを接続して、IPアドレス, 時刻を設定する。
4. デフォルトではuser: nvidia, password: nvidiaでログインできる。必要に応じて、sudo passwd nvidiaでパスワードを変更可能。
5. ビジョンコントローラにログインし、以下のコマンドでOSをアップデートする。
~~~
sudo apt-get update
~~~

### [参考2]ROS Kineticのインストール手順詳細
~~~
mkdir ~/src
cd ~/src
git clone https://github.com/jetsonhacks/installROSTX2.git
cd installROSTX2/
./installROS.sh -p ros-kinetic-desktop-full
./setupCatkinWorkspace.sh
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
. ~/.bashrc
~~~

------

# RoVIの構築手順

## 1. 3Dカメラ関連の設定

### 1-1. 汎用GigEライブラリ (libaravis) のインストール

#### 1-1-1. ビルド用の前準備
~~~
sudo apt-get install automake intltool
~~~

#### 1-1-2. ソースをダウンロード
~~~
cd ~
mkdir aravis
cd aravis
wget http://ftp.gnome.org/pub/GNOME/sources/aravis/0.4/aravis-0.4.1.tar.xz
~~~

#### 1-1-3. ソース解凍先でmakeしてインストール
~~~
tar xvf aravis-0.4.1.tar.xz
cd aravis-0.4.1
./configure
make
sudo make install
~~~

#### 1-1-4. 動作用の設定
~~~
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib" >> ~/.bashrc
. ~/.bashrc
~~~

#### 1-1-5. 動作確認
~~~
arv-tool-0.4
~~~

↓ と表示されることを確認する。  
`No device found`

### 1-2. ROSのGigEカメラ汎用ドライバ (camera_aravis) のインストール
~~~
cd ~/catkin_ws/src
git clone https://github.com/YOODS/camera_aravis
cd ~/catkin_ws
catkin_make
~~~

*上記git cloneのURIに注意。  
(RoVIでは、YOODSで一部改変したcamera_aravisドライバを使用する。)*

### 1-3. 物理接続
ビジョンコントローラと3DカメラをGigE LANケーブルで接続して、3Dカメラの電源を入れる。

### 1-4. 3Dカメラ対向のGigEインターフェースに適切なIPアドレスとMTUを設定
3Dカメラの工場出荷時のIPアドレスは以下なので、  
ビジョンコントローラの3Dカメラ対向のGigEインターフェースに、これと通信できるIPアドレスを設定する。
- 192.168.1.250/24

IPアドレス設定後、ビジョンコントローラからこの3DカメラIPアドレスへpingが通ることを確認する。

また、同インターフェースに、MTUとして 9000 を設定する。

(ビジョンコントローラの再起動後も動作ができるように、これらの設定の永続化も行うこと。)

### 1-5. 接続の最終確認
~~~
arv-tool-0.4
~~~

↓ のようにカメラのIDが表示されることを確認する。  
`YOODS Co,LTD.-`  

## 2. 各種ミドルウェアのインストール

### 2-1. 必須パッケージのインストール
~~~
sudo apt-get -y install curl
sudo apt-get -y install apt-transport-https
curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | sudo apt-key add -
echo "deb https://dl.yarnpkg.com/debian/ stable main" | sudo tee /etc/apt/sources.list.d/yarn.list
sudo apt-get update
sudo apt-get install yarn
~~~
### 2-2. Node.jsのインストール
~~~
cd ~
curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
sudo apt-get -y install nodejs
~~~

### 2-3. rosnodejsのインストール
~~~
cd ~
npm install rosnodejs
~~~

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

### 2-4. js-yamlのインストール
~~~
cd ~
npm install js-yaml
~~~

### 2-5. python-catkin-toolsのインストール
~~~
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get -y install python-catkin-tools
~~~

### 2-6. ros-kinetic-opencv3のインストール
~~~
sudo apt-get install ros-kinetic-opencv3
~~~
※ TODO OpenCVはJetson nativeなものをビルドして入れるべき!

### 2-7. cmakeのインストール
~~~
sudo apt-get install cmake
~~~
※ TODO catkin_cmakeが動いているということはすでにcmakeは入っているはずで、この手順は不要と思われる。

## 3. RoVI本体のROSパッケージのインストール
~~~
npm install shm-typed-array

【TODO: この段落はROSコアパッケージのimage_pipelineへのPull Requestが通るまでの一時的な対処】
cd ~/catkin_ws/src
git clone https://github.com/YOODS/image_pipeline

cd ~/catkin_ws/src
git clone https://github.com/YOODS/rovi

cd rovi
git checkout nedo

cd ~/catkin_ws/src
git clone https://github.com/ros-perception/vision_opencv
TODO 不要?


===========【以下は不要になった】=========
cd ~/catkin_ws/src/rovi/shm-typed-array
npm install nan
npm install node-cleanup
sudo npm install -g node-gyp
node-gyp configure
node-gyp build
======================================

cd ~/catkin_ws/src/rovi
wget --no-check-certificate https://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz
tar -xf 3.3.4.tar.gz
mkdir include
mv eigen-eigen-5a0156e40feb/Eigen/ include
rm -rf eigen-eigen-5a0156e40feb/ 3.3.4.tar.gz

cd ~/catkin_ws
catkin_make
~~~

------

# RoVIチュートリアル

## A. 起動

### A-1. カメラ解像度を VGA (640x480) にする場合
~~~
roslaunch rovi run-ycam3vga.launch
~~~
VGAで使用するカメラパラメータ(ライブ,キャプチャ), 位相シフト等の計測/計算パラメータについては、  
~/catkin_ws/src/rovi/yaml/ycam3vga.yaml  
に保存されている。

TODO ↑この文章要校正。（一部はカメラに入っているし。）

### A-2. カメラ解像度を SXGA (1280x1024) にする場合
~~~
roslaunch rovi run-ycam3sxga.launch
~~~
SXGAで使用するカメラパラメータ(ライブ,キャプチャ), 位相シフト等の計測/計算パラメータについては、  
~/catkin_ws/src/rovi/yaml/ycam3sxga.yaml  
に保存されている。

TODO ↑この文章要校正。（一部はカメラに入っているし。）

### A-3. ライブのON/OFF (トリガーモードのOff/On) の切り替え方法
RoVIのライブ仕様は以下：
- 起動時はライブONとなっている。
- E節の位相シフト処理中は自動的にライブOFF状態となり、処理が終わると自動的にライブONに戻る。
- 上記2つを指して「常時ライブ機能」と呼ぶ。
- 通常は「常時ライブ」のままでよいが、何らかの事情でライブをOFFしたり、そこから再度ONしたりしたい場合は、以下のようにして可能。  
(このようにしてライブOFFしても、上記位相シフト処理後の自動的なライブONは実行されることに注意。)

~~~
# ライブOFFしたい場合 (以下のどちらでもよい)
rosservice call /rovi/live_stop
rosservice call /rovi/ycam_ctrl/parse 'cset {"TriggerMode":"On"}'

# ライブONしたい場合 (以下のどちらでもよい)
rosservice call /rovi/live_start
rosservice call /rovi/ycam_ctrl/parse 'cset {"TriggerMode":"Off"}'
~~~

## B. カメラ画像へのアクセス

### B-1. 画像表示
~~~
rosrun image_view image_view image:=/rovi/camera/image_raw
~~~

### B-2. 画像保存
~~~
~/catkin_ws/src/rovi/script/imsave.js XX
~~~
- XX部分は任意の文字列を指定可能。
- このコマンドによって、以下の2つの画像ファイルがカレントディレクトリに生成される。
  - leftXX.pgm ... 左カメラ画像 (/rovi/left/image_raw Topicに入ってきた画像)
  - rightXX.pgm ... 右カメラ画像 (/rovi/right/image_raw Topicに入ってきた画像)

~~~
~/catkin_ws/src/rovi/script/imsave1.js YY
~~~
- YY部分は任意の文字列を指定可能。
- このコマンドによって、以下の画像ファイルがカレントディレクトリに生成される。
  - captYY.pgm ... 左右カメラ結合画像 (/rovi/camera/image_raw Topicに入ってきた画像。aravisからのraw画像。)

## C. カメラパラメータ (live)

### C-1. ライブカメラパラメータセット
~~~
rosparam get /rovi/live/camera/[Parameter Item]
[Parameter Item]を省略すると一覧を取得できる
~~~

### C-2. カメラパラメータセット
~~~
例) rosparam set /rovi/live/camera/ExposureTime 20000
~~~
| Parameter Item |   min   |   max   | default |
|:---------------|--------:|--------:|--------:|
|AcquisitionFrameRate|1|30|10|
|ExposureTime|1000| 32000| 20000|
|Gain|0|255|100|
|GainAnalog|0|255|0|

## D. プロジェクタ制御

### D-1. 位相シフトパターン発光 (カメラトリガも送出)
~~~
rosservice call /rovi/ycam_ctrl/parse ‘pset {“Go”:2}’
~~~
※プロジェクタ制御コマンドについては別紙資料を参照

## E. 位相シフト

### E-1. 計測実行
~~~
rosservice call /rovi/pshift_genpc
~~~

### E-2. 撮影パラメータ
~~~
rosparam get /rovi/pshift_genpc/

calc: {brightness: 256, bw_diff: 7, darkness: 5, ls_points: 3, max_parallax: 400,
max_ph_diff: 3.0, min_parallax: -300, right_cnt: 5, reject_diff: 1.5, search_div: 2,
step_diff: 1.2}
camera: {ExposureTime: 8400, Gain: 50}
projector: {ExposureTime: 10, Intensity: 100, Interval: 100}
~~~
これらのパラメータを個別に設定することも可能。例えばカメラゲインを100に変更する場合は以下のコマンドを実行する。
~~~
rosparam set /rovi/pshift_genpc/camera/Gain 100
~~~

## F. カメラ情報
~~~
rostopic echo /rovi/left/camera_info
~~~

## G. aravis関連

### G-1. ブートストラップレジスタの内容表示
~~~
arv-tool-0.4 --debug=all:3
~~~

### G-2. GenICam XMLの取得
~~~
arv-tool-0.4 genicam
~~~
