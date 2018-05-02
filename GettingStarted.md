# はじめに
このドキュメントでは、RoVIを構築して実行するための手順を記述します。

# 使用ハードウェア
- PC (ビジョンコントローラ)
- YCAM3D-I (3Dカメラ)
- GigE LANケーブル (ビジョンコントローラと3Dカメラを接続)

# ビジョンコントローラの前提条件
このドキュメントでは、ビジョンコントローラについて、以下を前提条件とします。  
この前提条件を満たしたビジョンコントローラを用意してください。

- OSとしてUbuntu 16.04 LTSをインストール済み。
- ROS Kineticをインストール済み。
- gitをインストール済み。
- ROSワークスペースは~/catkin_wsとする。  
(~/catkin_ws以外のROSワークスペースでも可。  
その場合は以下の例を読み替えること。)

以下はすべて、このビジョンコントローラでの作業となります。

------

# RoVIの構築手順

## 1. 3Dカメラ関連の設定

### 1-1. 物理接続
ビジョンコントローラと3DカメラをGigE LANケーブルで接続して、3Dカメラの電源を入れる。

### 1-2. 3Dカメラ対向のGigEインターフェースに適切なIPアドレスを設定
3Dカメラの工場出荷時のIPアドレスは以下なので、  
ビジョンコントローラの3Dカメラ対向のGigEインターフェースに、これらと通信できるIPアドレスを設定する。
- 左側カメラ: 192.168.222.1/24
- 右側カメラ: 192.168.222.2/24
- プロジェクター: 192.168.222.10/24

~~~
Ubuntu Desktop版での設定例：  

Ubuntuの[システム設定]->[ネットワーク]で、
3Dカメラ対向のGigEインターフェースに、
IPアドレスとして 192.168.222.99/24 を以下のように設定する。

[IPv4設定]タブで
  方式：手動
  アドレス：192.168.222.99	24	空
~~~

IPアドレス設定後、ビジョンコントローラから上述の3つの3DカメラIPアドレスへpingが通ることを確認する。

### 1-3. SentechカメラSDKのインストール

#### 1-3-1. SDKをダウンロード
https://sentech.co.jp/products/GigE/software.html  
から Sentech SDK Package の Linux SDK x86_x64用をダウンロードする。  
(2018/4/19時点では https://sentech.co.jp/data/software/usb/SentechSDK-1.0.4-x86_64.tgz が最新。)
~~~
cd ~
wget https://sentech.co.jp/data/software/usb/SentechSDK-1.0.4-x86_64.tgz
~~~

#### 1-3-2. SDKを解凍しインストーラを実行してインストール
~~~
tar xvzf SentechSDK-1.0.4-x86_64.tgz
cd SentechSDK-1.0.4-x86_64
./SentechSDK-1.0.4-x86_64-install.run
~~~

#### 1-3-3. 動作用の設定
~~~
source /opt/sentech/.stprofile
/opt/sentech/bin/setnetwork.sh <3Dカメラ対向のGigEインターフェース名>
~~~

#### 1-3-4. 動作確認
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

#### 1-3-5. 動作用の設定の永続化
ビジョンコントローラの再起動後も動作ができるように、設定の永続化を以下のようにして行う。  

- 3Dカメラ対向のGigEインターフェースのMTUを9000に設定
  ~~~
  Ubuntu Desktop版での設定例：  

  Ubuntuの[システム設定]->[ネットワーク]で、
  3Dカメラ対向のGigEインターフェースに、
  MTUとして 9000 を以下のように設定する。

  [Ethernet]タブで
    MTU：9000
  ~~~

- カーネルパラメータnet.core.(r/w)mem_(max/default)を33554432に設定
  ~~~
  設定例：  
  sudo vi /etc/sysctl.conf
  で
  net.core.rmem_max = 33554432
  net.core.wmem_max = 33554432
  net.core.rmem_default = 33554432
  net.core.wmem_default = 33554432
  を追加する。
  ~~~

## 2. 各種ミドルウェアのインストール

### 2-1. OpenCVのインストール
~~~
sudo apt-get install libopencv-dev
~~~

### 2-2. Node.jsのインストール
~~~
curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
sudo apt-get install nodejs
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

## 3. RoVI本体のROSパッケージのインストール
~~~
cd ~/catkin_ws/src
git clone https://github.com/YOODS/rovi

cd rovi/sentech_grabber
make  (←その結果このディレクトリに grabber というファイルができる)

cd ../shm-typed-array
npm install nan
npm install node-cleanup
sudo npm install -g node-gyp
node-gyp configure
node-gyp build

cd ..
wget http://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz
tar xvzf 3.3.4.tar.gz
mkdir include
mv eigen-eigen-5a0156e40feb/Eigen/ include
rm -rf eigen-eigen-5a0156e40feb/ 3.3.4.tar.gz

cd ../..
catkin_make
~~~

## 4. RoVIの動作設定
実際の環境に合わせて、RoVIのROS Parameterファイル
`~/catkin_ws/src/rovi/yaml/rovi_param.yaml`  
の以下の部分を、編集する。  
(以下の※2箇所のDisplay Nameは [1-3-4. 動作確認](#memodn) でメモしておいたもの。)

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
    『この部分(Q:)に実際のキャリブレーションパラメータ』 TODO 透視変換行列

TODO
http://opencv.jp/opencv-2.1/cpp/camera_calibration_and_3d_reconstruction.html

~~~

TODO:  
1. remap部分は、Calibration.mdにあるようにCD-ROMでの提供?
2. genpc部分もremapと同じくCD-ROMでの提供?

------

# RoVIの実行手順

## A. RoVIの起動
~~~
roslaunch rovi run.launch
~~~

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

生成された点群は以下のTopicにpublishされる。
- /rovi/pc  
(形式は /opt/ros/kinetic/share/sensor_msgs/msg/PointCloud.msg)
~~~
点群の表示例:
rviz
(そのRvizの画面で /rovi/pc を表示すればよい)
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

## 3. その他インストール
`sudo apt-get install curl`

