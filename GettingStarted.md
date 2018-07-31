# はじめに
このドキュメントでは、RoVIを構築して実行するための手順を記述します。

# 使用ハードウェア
- PC (ビジョンコントローラ)
- YCAM3D-I, YCAM3D-II, YCAM3D-III のいずれか (3Dカメラ)
- GigE LANケーブル (ビジョンコントローラと3Dカメラを接続)  

    ※GigE LANケーブルは、3DカメラがYCAM3D-IIIの場合はそれに付属する専用品を使用。  
　(YCAM3D-I, IIの場合は任意のGiE LANケーブルを使用可能。)

# ビジョンコントローラの前提条件
このドキュメントでは、ビジョンコントローラについて、以下を前提条件とします。  
この前提条件を満たしたビジョンコントローラを用意してください。

- OSとしてUbuntu 16.04 LTSをインストール済み。
- gitをインストール済み。
- ROS Kineticを[インストール](http://wiki.ros.org/kinetic/Installation/Ubuntu)済み。
- ROSワークスペースは~/catkin_wsとして作成済み。  
(~/catkin_ws以外のROSワークスペースでも可。  
その場合は以下の例を読み替えること。)

以下はすべて、このビジョンコントローラでの作業となります。

------

# RoVIの構築手順

## 1. 3Dカメラ関連の設定

*※3DカメラとしてYCAM3D-IまたはIIを使う場合は次の1A節を、YCAM3D-IIIを使う場合はその次の1B節を実行してください。*

## 1A. 3Dカメラ関連の設定 (YCAM3D-IまたはIIを使う場合)

### 1A-1. 物理接続
ビジョンコントローラと3DカメラをGigE LANケーブルで接続して、3Dカメラの電源を入れる。

### 1A-2. 3Dカメラ対向のGigEインターフェースに適切なIPアドレスを設定
3Dカメラの工場出荷時のIPアドレスは以下なので、  
ビジョンコントローラの3Dカメラ対向のGigEインターフェースに、これらと通信できるIPアドレスを設定する。(設定の永続化も行う。)
- 左側カメラ: 192.168.222.1/24
- 右側カメラ: 192.168.222.2/24
- プロジェクター: 192.168.222.10/24

~~~
Ubuntu Desktop版での設定(永続化)例：  

Ubuntuの[システム設定]->[ネットワーク]で、
3Dカメラ対向のGigEインターフェースに、
IPアドレスとして 192.168.222.99/24 を以下のように設定する。

[IPv4設定]タブで
  方式：手動
  アドレス：192.168.222.99	24	空
~~~

IPアドレス設定後、ビジョンコントローラから上述の3つの3DカメラIPアドレスへpingが通ることを確認する。

### 1A-3. SentechカメラSDKのインストール

#### 1A-3-1. SDKをダウンロード
https://sentech.co.jp/products/GigE/software.html  
から Sentech SDK Package の Linux SDK x86_x64用をダウンロードする。  
(2018/4/19時点では https://sentech.co.jp/data/software/usb/SentechSDK-1.0.4-x86_64.tgz が最新。)
~~~
cd ~
wget https://sentech.co.jp/data/software/usb/SentechSDK-1.0.4-x86_64.tgz
~~~

#### 1A-3-2. SDKを解凍しインストーラを実行してインストール
~~~
tar xvzf SentechSDK-1.0.4-x86_64.tgz
cd SentechSDK-1.0.4-x86_64
./SentechSDK-1.0.4-x86_64-install.run
~~~

#### 1A-3-3. 動作用の設定
~~~
echo "source /opt/sentech/.stprofile" >> ~/.bashrc
. ~/.bashrc
/opt/sentech/bin/setnetwork.sh <3Dカメラ対向のGigEインターフェース名>
~~~

#### 1A-3-4. 動作確認
~~~
/opt/sentech/bin/StViewer
~~~

以下の画面例のように &lt;3Dカメラ対向のGigEインターフェース名&gt; の下に2台のカメラが表示されることを確認する。  
![StViewer1](https://raw.githubusercontent.com/YOODS/rovi/pics/StViewer1.png)

画面右下の「IP Address」が192.168.222.1になっている方が左側カメラ、192.168.222.2になっている方が右側カメラ。  
<p id="memodn">
のちほど [4. RoVIの動作設定](#4-rovi-) で使うので、左側カメラの表示名と右側カメラの表示名（画面右下の「Display Name」とも同じ）をメモしておくこと。
</p>

*この例では、以下をメモしておく。*
- *左側カメラの表示名: STC_BBE132GE(17AB755)*
- *右側カメラの表示名: STC_BBE132GE(17AB756)*

この画面でOKボタンをクリックすると以下のような画面が表示される。  
![StViewer2](https://raw.githubusercontent.com/YOODS/rovi/pics/StViewer2.png)

この画面上部の Start Acquisition, Stop Acquisition でライブ映像の ON/OFF を実行できるので、ライブ映像が表示されることを確認する。  
Close active cameraで接続を閉じ、 Open a camera からもう1台のカメラでも同様にライブ映像の表示を確認する。

#### 1A-3-5. 動作用の設定の永続化
ビジョンコントローラの再起動後も動作ができるように、設定の永続化を以下のようにして行う。  

- 3Dカメラ対向のGigEインターフェースのMTUを9000に設定
  ~~~
  Ubuntu Desktop版での設定(永続化)例：  

  Ubuntuの[システム設定]->[ネットワーク]で、
  3Dカメラ対向のGigEインターフェースに、
  MTUとして 9000 を以下のように設定する。

  [Ethernet]タブで
    MTU：9000
  ~~~

- カーネルパラメータnet.core.(r/w)mem_(max/default)を33554432に設定
  ~~~
  設定(永続化)例：  
  sudo vi /etc/sysctl.conf
  で
  net.core.rmem_max = 33554432
  net.core.wmem_max = 33554432
  net.core.rmem_default = 33554432
  net.core.wmem_default = 33554432
  を追加する。
  ~~~

## 1B. 3Dカメラ関連の設定 (YCAM3D-IIIを使う場合)

### 1B-1. 汎用GigEライブラリ (libaravis) のインストール

#### 1B-1-1. ビルド用の前準備
~~~
sudo apt-get install automake intltool
~~~

#### 1B-1-2. ソースをダウンロード
~~~
cd ~
mkdir aravis
cd aravis
wget http://ftp.gnome.org/pub/GNOME/sources/aravis/0.4/aravis-0.4.1.tar.xz
~~~

#### 1B-1-3. ソース解凍先でmakeしてインストール
~~~
tar xvf aravis-0.4.1.tar.xz
cd aravis-0.4.1
./configure
make
sudo make install
~~~

#### 1B-1-4. 動作用の設定
~~~
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib" >> ~/.bashrc
. ~/.bashrc
~~~

#### 1B-1-5. 動作確認
~~~
arv-tool-0.4
~~~

↓ と表示されることを確認する。  
`No device found`

### 1B-2. 物理接続
ビジョンコントローラと3DカメラをGigE LANケーブルで接続して、3Dカメラの電源を入れる。

### 1B-3. 3Dカメラ対向のGigEインターフェースに適切なIPアドレスとMTUを設定
3Dカメラの工場出荷時のIPアドレスは以下なので、  
ビジョンコントローラの3Dカメラ対向のGigEインターフェースに、これと通信できるIPアドレスを設定する。
- 192.168.1.250/24

IPアドレス設定後、ビジョンコントローラからこの3DカメラIPアドレスへpingが通ることを確認する。

また、同インターフェースに、MTUとして 9000 を設定する。

(ビジョンコントローラの再起動後も動作ができるように、これらの設定の永続化も行うこと。)

~~~
Ubuntu Desktop版での設定(永続化)例：  

Ubuntuの[システム設定]->[ネットワーク]で、
3Dカメラ対向のGigEインターフェースに、
IPアドレスとして 192.168.1.99/24 を以下のように設定する。

[IPv4設定]タブで
  方式：手動
  アドレス：192.168.1.99	24	空

同様に、MTUとして 9000 を以下のように設定する。

[Ethernet]タブで
  MTU：9000
~~~

### 1B-4. 接続の最終確認
~~~
arv-tool-0.4
~~~

↓ のようにカメラのIDが表示されることを確認する。  
`YOODS Co,LTD.-`  

## 2. 各種ミドルウェアのインストール

### 2-1. Node.jsのインストール
~~~
cd ~
curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
sudo apt-get install nodejs
~~~

### 2-2. rosnodejsのインストール
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

### 2-3. js-yamlのインストール
~~~
cd ~
npm install js-yaml
~~~

## 3. RoVI本体のROSパッケージのインストール
~~~
cd ~/catkin_ws/src
git clone https://github.com/YOODS/rovi

cd rovi
git checkout nedo

【※YCAM3D-IまたはIIを使う場合のみ、この段落の実行が必要 (YCAM3D-IIIの場合は実行不要)】
cd sentech_grabber
make  (←その結果このディレクトリに grabber というファイルができる)

【※YCAM3D-IIIを使う場合のみ、この段落の実行が必要 (YCAM3D-IまたはIIの場合は実行不要)】
cd ~/catkin_ws/src
git clone https://github.com/YOODS/camera_aravis
(このgit cloneのURIに注意。RoVIでは、YOODSで一部改変したcamera_aravisドライバを使用する)

cd ~/catkin_ws/src/rovi/shm-typed-array
npm install nan
npm install node-cleanup
sudo npm install -g node-gyp
node-gyp configure
node-gyp build

cd ~/catkin_ws/src/rovi
wget http://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz
tar xvzf 3.3.4.tar.gz
mkdir include
mv eigen-eigen-5a0156e40feb/Eigen/ include
rm -rf eigen-eigen-5a0156e40feb/ 3.3.4.tar.gz

cd ~/catkin_ws
catkin_make
~~~

## 4. RoVIの動作設定
*※この節の内容は、3DカメラとしてYCAM3D-IまたはIIを使う場合のみ実行が必要。  
　(YCAM3D-IIIの場合は実行不要)*

実際の環境に合わせて、RoVIのROS Parameterファイル  
`~/catkin_ws/src/rovi/yaml/ycam1s.yaml` (... YCAM3D-Iの場合)  
`~/catkin_ws/src/rovi/yaml/ycam2.yaml` (... YCAM3D-IIの場合)  
の以下の部分を、編集する。  
(以下の※2箇所のDisplay Nameは [1A-3-4. 動作確認](#memodn) でメモしておいたもの。)

~~~
left:
  camera:
    ID: 『この部分に左側カメラの実際のDisplay Name(※)』
  remap:
    『この部分(height:からP:まで)に実際の左側キャリブレーションパラメータ』

right:
  camera:
    ID: 『この部分に右側カメラの実際のDisplay Name(※)』
  remap:
    『この部分(height:からP:まで)に実際の右側キャリブレーションパラメータ』
genpc:
    『この部分(Q:)に実際のキャリブレーションパラメータ(透視変換行列)』
~~~

TODO:  
1. remap部分は、Calibration.mdにあるようにCD-ROMでの提供?
2. genpc部分もremapと同じくCD-ROMでの提供?

------

# RoVIの実行手順

## A. RoVIの起動

### A-I. 3DカメラとしてYCAM3D-Iを使う場合
~~~
roslaunch rovi run-ycam1s.launch
~~~

### A-II. 3DカメラとしてYCAM3D-IIを使う場合
~~~
roslaunch rovi run-ycam2.launch
~~~

### A-IIIvga. 3DカメラとしてYCAM3D-IIIを使い、カメラ解像度を VGA (640x480) にする場合
~~~
roslaunch rovi run-ycam3vga.launch
~~~
TODO ~/catkin_ws/src/rovi/yaml/ycam3vga.yaml

### A-IIIsxga. 3DカメラとしてYCAM3D-IIIを使い、カメラ解像度を SXGA (1280x1024) にする場合
~~~
roslaunch rovi run-ycam3sxga.launch
~~~
TODO ~/catkin_ws/src/rovi/yaml/ycam3sxga.yaml

## B. RoVIの機能

### B-1. ROS Parameter
ユーザによる値の変更が可能なRoVIのROS Parameterの一覧については[X-1. RoVIのParameter一覧](#x-1-rovi-parameter-)を参照。

それらの値の取得／設定は rosparam get/set で可能。

TODO rosparam setをそのまま反映させるためにreload機能が必要か。

rosparm setした値を、次回RoVIの起動時にも引き継ぎたい場合は、以下を実行する。
~~~
rosparam dump ~/catkin_ws/src/rovi/yaml/rovi_param.yaml /rovi
~~~

### B-2. 常時ライブ
RoVIは実行中、常にライブ画像を取得するとともにrectify処理を行っている。

これらの画像は以下のTopicにpublishされている。
- 左側カメラライブ画像: /rovi/left/image_raw
- 右側カメラライブ画像: /rovi/right/image_raw
- 左側カメラrectify画像: /rovi/left/image_rect
- 右側カメラrectify画像: /rovi/right/image_rect
~~~
左側カメラライブ画像の表示例:
rosrun image_view image_view image:=/rovi/left/image_raw
~~~

ライブ画像の取得周期は10FPSになっている。

何らかの事情で一時的に常時ライブ機能を停止したい場合は以下で停止と再開を実現できる。
~~~
常時ライブ機能の一時的な停止例:
rosservice call /rovi/pshift_genpc/parse 'cset {"TriggerMode":"On"}'
常時ライブ機能の再開例:
rosservice call /rovi/pshift_genpc/parse 'cset {"TriggerMode":"Off"}'
~~~
TODO ↑の情報公開する?しない?

なお、 [B-3. 位相シフト計算と点群生成](#b-3-) の処理中はこの常時ライブ機能は止まる。  
その処理が終わるとまた常時ライブ機能が再開する。

### B-3. 位相シフト計算と点群生成
~~~
rosservice call /rovi/pshift_genpc
~~~

生成された点群は以下の2つのTopicにpublishされる。
- /rovi/pc  
(形式は /opt/ros/kinetic/share/sensor_msgs/msg/PointCloud.msg)
- /rovi/pc2  
(形式は /opt/ros/kinetic/share/sensor_msgs/msg/PointCloud2.msg)
~~~
点群の表示例:
rviz
(そのRvizの画面で /rovi/pc や /rovi/pc2 を表示すればよい)
~~~

位相シフト計算と点群生成の入力として使われた左側カメラ13枚、右側カメラ13枚のrectify画像は以下のTopicにpublishされる。  
（ただし実際に13枚のうちどれをpublishするかは次に述べる操作によって決まる。)
- 左側カメラrectify画像: /rovi/left/view
- 右側カメラrectify画像: /rovi/right/view
  ~~~
  rectify画像の表示例:
  rosrun image_view image_view image:=/rovi/left/view
  rosrun image_view image_view image:=/rovi/right/view

  以下の操作によって実際にN枚目(N=0, 1, ... 12)の左右カメラrectify画像がpublishされる。
  rosservice call /rovi/pshift_genpc/parse 'view 『N』'

  したがって上記image_viewを実行しておけば、
  以下の操作で順に4, 0, 12枚目のrectify画像の表示が行われる。
  rosservice call /rovi/pshift_genpc/parse 'view 4'
  rosservice call /rovi/pshift_genpc/parse 'view 0'
  rosservice call /rovi/pshift_genpc/parse 'view 12'
  ~~~
TODO ↑生画像も表示する仕組みがあると良い?さらに、生画像とrectify画像の計52枚を52個のTopicにも出すと良い?
------

# 付録

## X-1. RoVIのParameter一覧

ユーザによる値の変更が可能なRoVIのROS Parameterの一覧は以下の通り。

- /rovi/live/camera/ExposureTime  
  (=[常時ライブ機能](#b-2-)でのカメラのExposureTime)
- /rovi/live/camera/Gain  
  (=[常時ライブ機能](#b-2-)でのカメラのGain)
- /rovi/pshift_genpc/camera/ExposureTime  
  (=[位相シフト計算と点群生成機能](#b-3-)でのカメラのExposureTime)
- /rovi/pshift_genpc/camera/Gain  
  (=[位相シフト計算と点群生成機能](#b-3-)でのカメラのGain)

TODO これだけ?

rosparm setした値を、次回RoVIの起動時にも引き継ぎたい場合は、以下を実行する。
~~~
rosparam dump ~/catkin_ws/src/rovi/yaml/rovi_param.yaml /rovi
~~~

## X-2. RoVIのService一覧

ユーザによるcallが可能なRoVIのROS Serviceの一覧は以下の通り。

- /rovi/pshift_genpc  
  (=[位相シフト計算と点群生成機能](#b-3-)のService)
- /rovi/pshift_genpc/parse
  (TODO 公開する?しない?)

## X-3. RoVIのTopic一覧

ユーザが値を参照可能なRoVIのROS Topicの一覧は以下の通り。

- /rovi/left/image_raw  
  (=[常時ライブ機能](#b-2-)での左側カメラライブ画像)
- /rovi/left/image_rect  
  (=[常時ライブ機能](#b-2-)での左側カメラrectify画像)
- /rovi/left/view  
  (=[位相シフト計算と点群生成機能](#b-3-)の入力として使われた左側カメラrectify画像)
- /rovi/right/image_raw  
  (=[常時ライブ機能](#b-2-)での右側カメラライブ画像)
- /rovi/right/image_rect  
  (=[常時ライブ機能](#b-2-)での右側カメラrectify画像)
- /rovi/right/view  
  (=[位相シフト計算と点群生成機能](#b-3-)の入力として使われた右側カメラrectify画像)
- /rovi/pc  
  (=[位相シフト計算と点群生成機能](#b-3-)の出力点群)
------

# JETSON TX2へのインストール手順

## 1. JETPACK3.2のインストール
Ubuntu16.04.4を用意して、その上でJETPACK3.2をインストールする。
注) 2018/04/05のインストールでは、sudo apt-get purge libhighgui-devを実行しないと、途中CUDAのインストール中エラーとなっていた。

## 2. OpenCVのインストール
JETPACK搭載のOpenCVでは位相シフト,カメラキャリブが動かない。OpenCVをソースからビルドする必要がある。
OpenCVビルド方法は以下を参照のこと。
https://github.com/jetsonhacks/buildOpenCVTX2

	$ git clone https://github.com/jetsonhacks/buildOpenCVTX2.git	#buildOpenCVTX2ディレクトリができる
	$ cd buildOpenCVTX2
	$ ./buildOpenCV.sh	#~/opencv以下にopencv3.3.0ビルド(./buildOpenCV.shを編集すれば3.3.1も可能では?)
	$ cd ~/opencv/build
	$ sudo make install

この手順ではJetpackでインストールされるOpenCV3.3をそのまま置き換える。OpenCVのバージョン(3.3)は変わらない。

## 3. Sentechカメラドライバ(YCAM3D-IIの場合のみ)
~~~
tar -xf packageStSDK-aarch64.tar,gz
cd packageStSDK-aarch64/
chmod +x packageStSDK-aarch64-install.run
./packageStSDK-aarch64-install.run
cp /opt/sentech/.stprofile /etc/profile.d/stprofile.sh
/opt/sentech/bin/setnetwork.sh eth0
~~~

## 4. その他インストール
~~~
sudo apt-get install curl
sudo apt-get install yarn

npm install nan
npm install node-cleanup
sudo npm install -g node-gyp
node-gyp configure
node-gyp build
~~~

## 5. ROS, Node.jsののインストール
### 4-1. Node.jsのインストール
~~~
curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
sudo apt-get install nodejs
~~~

### 4-2. rosnodejsのインストール
~~~
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

### 4-3. RoVI本体のROSパッケージのインストール
~~~
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/YOODS/rovi
cd rovi/sentech_grabber
make  (←その結果このディレクトリに grabber というファイルができる)
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

~~~
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
~~~

~~~
vi .bashrc
後ろに以下を追加
export PATH=/usr/local/cuda-9.0/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/snap/bin
export LD_LIBRARY_PATH=/usr/local/cuda-9.0/lib64:
source /opt/ros/kinetic/setup.bash
for rcsetup in *_ws/devel/setup.bash
do
	source $rcsetup
done
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
~~~

~~~
cd ~/catkin_ws/src/rovi/
mkdir include
find /usr/ -name Eigen
ls /usr/include/eigen3/Eigen
ln -s /usr/include/eigen3/Eigen include/
~~~

~~~
cd ~/PhaseShift/
make clean
make
sudo cp -a libyds3d* /usr/local/lib
sudo depmod -a
~~~

~~~
cd ~/catkin_ws/
catkin_make
~~~

~~~
TODO cd ~; npm install js-yaml
~~~


## 番外編 LinuxMintへのインスト−ル
メインマシンがMintであれば、そこで動かすことができます。以下はMintにインストールするときの注意点です。

### ROSのインストール
http://wiki.ros.org/kinetic/Installation/Ubuntu
のとおりですが、apt updateの前に  /etc/apt/sources.list.d/ros-latest.list  を
~~~
deb http://packages.ros.org/ros/ubuntu sonya main
~~~
から
~~~
deb http://packages.ros.org/ros/ubuntu xenial main
~~~
に変更  
SonyaはUbuntu(Xenial)に対応する、LinuxMintのコードネームです。

### Aravisのインストール
makeの前に以下を追加します。
~~~
sudo apt-get install libgstreamer*-dev
~~~

### rosdep
~~~
rosdep --os=ubuntu:xenial
~~~
を付けます。
