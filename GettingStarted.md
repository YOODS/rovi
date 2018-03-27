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

### 2-1. OpenCVのインストール
~~~
sudo apt-get install libopencv-dev
~~~

### 2-2. Node.jsのインストール
~~~
curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
sudo apt-get install nodejs
~~~

### 2-3. rosnodejsとそのパッケージ類のインストール
~~~
cd ~
npm install rosnodejs
npm install ws
npm install opencv
npm install canvas
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

TODO:  
Eigenが必要ならばここに # 3. セクションを入れて ↓こういう感じ。
~~~
cd ~
wget http://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz
mv eigen-eigen-5a0156e40feb/Eigen/ ~/catkin_ws/src/rovi/include/
rm -rf eigen-eigen-5a0156e40feb/
~~~

## 3. RoVI本体のROSパッケージのインストール
~~~
cd ~/catkin_ws/src
git clone https://github.com/YOODS/rovi
cd ..
catkin_make
~~~

## 4. RoVIの動作設定
実際の環境に合わせて、RoVIのROS Parameterファイル
`~/catkin_ws/src/rovi/yaml/rovi_rosparam_dump.yaml`  
の以下の部分を、編集する。

~~~
cam_l:
  camera:
    ID: 『この部分に左側カメラの実際のID』
    (略)
  remap:
    『この部分(height:からP:まで)に実際の左側キャリブレーションパラメータ』

cam_r:
  camera:
    ID: 『この部分に右側カメラの実際のID』
    (略)
  remap:
    『この部分(height:からP:まで)に実際の右側キャリブレーションパラメータ』
genpc:
    『この部分(Q:)に実際のキャリブレーションパラメータ』
~~~

TODO:  
1. カメラID（特に左右の区別）はユーザはどうやって知る?
2. remap部分は、Calibration.mdにあるようにCD-ROMでの提供?
3. genpc部分もremapと同じくCD-ROMでの提供?

------

# RoVIの実行手順

## A. RoVIの起動
~~~
roslaunch rovi rovi_run.launch
~~~

## B. TODO RoVIの名前空間 ?
node, topic, param, serviceなどすべて書く?

TODO 名前空間は
	/rovi/
		pshift_genpc（＝位相シフト撮影、計算、点群生成）◆サービス
			parse（13枚撮影は置いといて、ここかも。）

		cam_l/
			camera/
			remap/
		cam_r/
			camera/
			remap/

		phase_shift/
			calc（＝視差マップを作る）
			parse（ここで細々したものを実装）

		genpc/
			do
			setup
			try
			pcl
			pcl2

		recog/

	4/1時点でユーザに呼んでもらうのは◆のみ。
	（常時ライブとpshift_genpcのみでいい。）

## C. RoVIの機能

### C-1. ROS Parameter
TODO 詳細説明。camnodeはdynparam get/setしてもらう。

TODO rosparm setやdynparam setしたParameter値を、次回RoVIの起動時にも引き継ぎたい場合は、以下を実行する。
~~~
rosparam dump ~/catkin_ws/src/rovi/yaml/rovi_rosparam_dump.yaml /rovi
~~~

### C-2. 常時ライブ
TODO topicやimage_view例?

### C-3. 位相シフト計算と点群生成
~~~
rosservice call /rovi/pshift_genpc
~~~

TODO topicやimage_view例?
