# はじめに
このドキュメントでは、RoVIを構築して実行するための手順を記述します。

# 使用ハードウェア
- Jetson TX2 または YJC-4 (ビジョンコントローラ) (TODO インターネット接続用LANケーブル)
- YCAM3D-III (3Dカメラ), GigE Vision用LANケーブル

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

## 1. 各種ミドルウェアのインストール

### 1-1. 必須パッケージのインストール
~~~
sudo apt-get -y install curl
sudo apt-get -y install apt-transport-https
curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | sudo apt-key add -
echo "deb https://dl.yarnpkg.com/debian/ stable main" | sudo tee /etc/apt/sources.list.d/yarn.list
sudo apt-get update
sudo apt-get install yarn
~~~
### 1-2. Node.jsのインストール
~~~
cd ~
curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
sudo apt-get -y install nodejs
~~~

### 1-3. rosnodejsのインストール
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

### 1-4. ROSパッケージのインストール

~~~
mkdir ~/src
cd ~/src
git clone https://github.com/jetsonhacks/installROSTX2.git
cd installROSTX2/
./installROS.sh
./setupCatkinWorkspace.sh
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
~~~
一旦、ログアウトして再度、ログインする。または、別ターミナルを開く。

### 1-5. python-catkin-toolsインストール
~~~
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get -y install python-catkin-tools
~~~

### 1-6  ros-kinetic-opencv3インストール
~~~
sudo apt-get install ros-kinetic-opencv3
~~~
※ TODO OpenCVはJetson nativeなものをビルドして入れるべき!

### 1-6  cmakeインストール
~~~
sudo apt-get install cmake
~~~

## 2. RoVI本体のROSパッケージのインストール
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
