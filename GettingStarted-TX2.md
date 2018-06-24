# はじめに
このドキュメントでは、RoVIを構築して実行するための手順を記述します。

# 使用ハードウェア
- Jetson TX2(ビジョンコントローラ YJC-4も可)
- YCAM3D-III (3Dカメラ), ACアダプタ(DC24V), GigE Vision用LANケーブル

# ビジョンコントローラの準備
このドキュメントでは、ビジョンコントローラについて、以下を前提条件とします。  
この前提条件を満たしたビジョンコントローラを用意してください。
##1. JETPACK3.2をインストール
-JETPACKをインストールしたPCにUSBで接続して、以下フラッシュ書き換えモードにて以下のコマンドを実施
~~~
sudo ./flash.sh  jetson-tx2 mmcblk0p1
~~~
- jetpack/64_TX2/Linux_for_Tegrakernel/dtb/tegra186-quill-p3310-1000-c03-00-base.dtbを汎用キャリアボード対応版に変更しておくこと。これをやらないとUSBデバイスに電源が供給されない。
- 終了後素直にキーボード,マウス,HDMI,Ethenetを接続して、IPアドレス,時刻を設定する。
- user:nvidia, pw=nvidiaでログインできるが、sudo passwd nvidiaで簡単なパスワードに変更しておくことを勧める。
~~~
sudo apt-get update
~~~
以下はすべて、このビジョンコントローラでの作業となります。

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

### 1-4. RoVI本体のROSパッケージのインストール(2018/6/23現在、ROSパッケージのインストールができなくなっている。)
~~~
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/YOODS/rovi
~~~

~~~
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update

sudo apt-get install ros-kinetic-desktop-full
sudo apt-get install ros-kinetic-slam-gmapping
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
~~~

## 2. RoVI本体のROSパッケージのインストール(ROS本体のインストールができていないため、未実施(2018/6/24))
~~~
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/YOODS/rovi
cd rovi
git checkout develop

cd ~/catkin_ws/src/rovi/shm-typed-array
npm install nan
npm install node-cleanup
sudo npm install -g node-gyp
node-gyp configure
node-gyp build

cd ~/catkin_ws/src/rovi/
wget --no-check-certificate https://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz
tar -xf 3.3.4.tar.gz
mkdir include
mv eigen-eigen-5a0156e40feb/Eigen/ include
rm -rf eigen-eigen-5a0156e40feb/

cd ../..[^2]
catkin_make
~~~


