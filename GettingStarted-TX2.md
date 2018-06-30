# はじめに
このドキュメントでは、RoVIを構築して実行するための手順を記述します。

# 使用ハードウェア
- Jetson TX2 または YJC-4 (ビジョンコントローラ)
- YCAM3D-III (3Dカメラ)
- GigE LANケーブル (…YCAM3D-IIIに付属する専用品…) (ビジョンコントローラと3Dカメラを接続)

# ビジョンコントローラの準備
このドキュメントでは、ビジョンコントローラについて、以下を前提条件とします。  
この前提条件を満たしたビジョンコントローラを用意してください。

##1. JETPACK3.2をインストール
- jetpack/64_TX2/Linux_for_Tegrakernel/dtb/tegra186-quill-p3310-1000-c03-00-base.dtbを汎用キャリアボード対応版に変更しておくこと。これをやらないとUSBデバイスに電源が供給されない。 (TODO YOODSから供給)
- 親機(JETPACKをインストールしたPC)とビジョンコントーラをmicro USBで接続して、リカバリーモードにて以下のコマンドを実施 (TODO コマンド実行に20分程度かかる)
~~~
sudo ./flash.sh  jetson-tx2 mmcblk0p1
~~~
- 上記のflash.shコマンドの終了後、キーボード,マウス,HDMIケーブル,インターネット接続用LANケーブルを接続して、IPアドレス,時刻を設定する。
- デフォルトではuser: nvidia, password: nvidiaでログインできる。必要に応じて、sudo passwd nvidiaでパスワードを変更可能。
~~~
sudo apt-get update
~~~
以下はすべて、このビジョンコントローラでの作業となります。 (TODO 上記ログインからはすべてビジョンコントローラ。)

------

# RoVIの構築手順

## 1. 3Dカメラ関連の設定

### 1-1. 汎用GigEライブラリ(libaravis)のインストール

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

### 1-2. ROSのGigEカメラ汎用ドライバ(camera_aravis)のインストール
~~~
cd ~/catkin_ws/src
git clone https://github.com/YOODS/camera_aravis
cd ..
catkin_make
~~~

*上記git cloneのURIに注意。  
(RoVIでは、YOODSで一部改変したcamera_aravisドライバを使用する。)*

### 1-3. 物理接続
ビジョンコントローラと3DカメラをGigE LANケーブルで接続して、3Dカメラの電源を入れる。

### 1-4. 3Dカメラ対向のGigEインターフェースに適切なIPアドレスを設定
3Dカメラ(YCAM3D-I)の工場出荷時のIPアドレスは以下なので、  
ビジョンコントローラ(PC)の3Dカメラ対向のGigEインターフェースに、これらと通信できるIPアドレスを設定する。
- 左側カメラ: 192.168.222.1/24
- 右側カメラ: 192.168.222.2/24
- プロジェクター: 192.168.222.10/24

~~~
Ubuntu Desktop版での設定例：  

Ubuntuの[システム設定]->[ネットワーク]で、
3Dカメラ対向のGigEインターフェースに 192.168.222.99/24 を以下のように設定する。

[IPv4設定]タブで
  方式：手動
  アドレス：192.168.222.99	24	空
~~~

IPアドレス設定後、ビジョンコントローラから上述の3つの3DカメラIPアドレスへpingが通ることを確認する。

### 1-5. 接続の最終確認
~~~
arv-tool-0.4
~~~

↓ のようにカメラ2台(左右)のIDが表示されることを確認する。   
`SENTECH-17AB755`  
`SENTECH-17AB756`  
（各3Dカメラで、IDの具体的な値はこれらと異なる。）

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

### 2-4. ROSパッケージのインストール

~~~
mkdir ~/src
cd ~/src
git clone https://github.com/jetsonhacks/installROSTX2.git
cd installROSTX2/
./installROS.sh -p ros-kinetic-desktop-full
./setupCatkinWorkspace.sh
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
~~~
一旦、ログアウトして再度、ログインする。または、別ターミナルを開く。

### 2-5. python-catkin-toolsインストール
~~~
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get -y install python-catkin-tools
~~~

### 2-6  ros-kinetic-opencv3インストール
~~~
sudo apt-get install ros-kinetic-opencv3
~~~
※ TODO OpenCVはJetson nativeなものをビルドして入れるべき!

### 2-7  cmakeインストール
~~~
sudo apt-get install cmake
~~~

## 3. RoVI本体のROSパッケージのインストール
~~~
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/YOODS/rovi
cd rovi
git checkout nedo

cd ~/catkin_ws/src
git clone https://github.com/ros-perception/vision_opencv
TODO 不要?

cd ~/catkin_ws/src/rovi/shm-typed-array
npm install nan
npm install node-cleanup
sudo npm install -g node-gyp
node-gyp configure
node-gyp build

cd ~/catkin_ws/src/rovi
wget --no-check-certificate https://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz
tar -xf 3.3.4.tar.gz
mkdir include
mv eigen-eigen-5a0156e40feb/Eigen/ include
rm -rf eigen-eigen-5a0156e40feb/ 3.3.4.tar.gz

※catkin_ws/src/rovi/CMakeLists.txtのlink_directoriesをaarch64用に変更する必要がある。
link_directories(
  lib/aarch64
# lib/x86_64
)


cd ~/catkin_ws
catkin_make
~~~

~~~
TODO cd ~; npm install js-yaml
~~~

# RoVIチュートリアル
## 1. 起動
### 1-1. カメラ解像度VGAの場合
~~~
roslaunch rovi run-ycam3vga.launch 
~~~
### 1-2. カメラ解像度SXGAの場合
~~~
roslaunch rovi run-ycam3sxga.launch
~~~

### 1-3. トリガーモードのon/off
起動時はトリガーモードon状態で起動しているので、ライブ状態にするにはoffに変更する必要がある。
~~~
rosservice call /rovi/ycam_ctrl/parse 'cset {"TriggerMode":"Off"}'
~~~

## 2. カメラ画像へのアクセス
### 2-1. 画像表示
~~~
rosrun image_view image_view image:=/rovi/camera/image_raw
~~~

### 2-2. 画像保存
~~~
script/imsave.js XX
leftXX.pgm, rightXX.pgmの2つのファイルをカレントディレクトリに生成
~~~
~~~
script/imsave1.js XX
raw画像(左右結合)をカレントディレクトリに生成
~~~

## 3. カメラパラメータ(live)
### 3-1. ライブｃカメラパラメータセット
~~~
rosparam get /rovi/live/camera/[Parameter Item]
| Parameter Item|   min   |   max   | default |
|:--------------|--------:|--------:|--------:|
|AcquisitionFrameRate|1|30|10|
|ExposureTime|1000| 32000| 20000|
|Gain|0|255|100|
|GainAnalog|0|255|0|
~~~

### 3-2. カメラパラメータセット
~~~
rosparam set /rovi/live/camera/ExposureTime 20000
~~~

## 4. プロジェクタ制御
### 4-1. 位相シフトパターン発光(カメラトリガも送出)
~~~
rosservice call /rovi/ycam_ctrl/parse ‘pset {“Go”:2}’
~~~

## 5. カメラ情報
~~~
rostopic echo /rovi/left/camera_info
~~~

## 6. aravis関連
### 6-1. ブートストラップレジスタの内容表示
~~~
arv-tool-0.4 —debug=all:3
~~~
### 6-2. GENICAM XMLの取得
~~~
arv-tool-0.4 genicam
~~~
