# RoVI

 RoVIはハンドアイによるロボットピッキングをサポートするソフトウェアパッケージです。ロボットピッキングを実現するためには下の4つの技術要素が必要です。

1. 3Dセンシング
2. サーチメソッド
3. ロボットキャリブレーション

RoVIは各要素に対応したソフトウェアを提供します。

## 1.3Dセンシング

### YCAM3D
 YCAM3Dは、YOODS社が提供する、ステレオカメラ方式の高精度3Dスキャナです。このカメラを使って、ROS上で距離画像および点群(PCL,PCL2)を出力するためのソフトウェアを提供します。[詳細はこちら](YCAM3D.md)。

#### YCAM3D-III Specification

### 他社製の対応H/Wがあれば...


## 2.サーチメソッド
 サーチメソッドは2D、3D画像からそこに存在する対象物を認識する手段、そしてその対象物の座標を得る(測位)手段です。対象物の認識には、その目的から大きく２つに分類されます。
1. 抽象認識(モノの種類を識別する)
2. 形状一致認識(参照形状と同一のモノを識別する。結果的に測位が同時に行われる)

 工業的に利用頻度が高いのが「2」の認識手段です。RoVIでは以下のソフトウェアを提供します。

### Halcon起源<sup>(1)</sup>のもの
 Halconが提供する物体認識ソフトウェアを、ROS上で利用するためのソフトウェアを提供します。[詳細はこちら](Search.md)。
(1)利用にはHalconのライセンスが必要です。「詳細はこちら」を参照

## 3.ロボットキャリブレーション
 ロボットキャリブレーションとは、カメラ等のセンサーが取得した、3Dおよび2Dデータの座標系からロボットの座標系への、変換行列を求める作業です。ハンドアイはロボットの先端(メカニカルインタフェース)に取り付けられるため、カメラ座標系とメカニカルインタフェース座標系との間の変換行列は定数の行列になります。これがハンドアイのメリットの一つです。この変換行列を精度よく求めることが、ロボットピッキングを実現するには必須です。

### 自動キャリブレーション
 最適化計算を用いた自動キャリブレーションのソフトウェアと、その使用方法を提供します。[詳細はこちら](Calib.md)。


## 番外【ロボットインタフェース】

 ROSで制御されるロボットであれば、1〜3によりロボットに対する動作指示が出来ます。しかし一般的な市販のロボットでは、RoVIにアクセスするための追加のソフトウェアが必要です。これはメーカ毎に異なるため順次対応を行っています。現在対応済のロボットは下記のとおり。

<table border>
<tr><th>メーカ<th>シリーズ
<tr><td>三菱電機<th>RF
<tr><th>Nachi<th>
</table># カメラキャリブレーション

カメラのRaw画像には歪があるため正確な測位のためにはRectify画像に変換しなければなりません。この変換を行うためには事前にカメラキャリブレションを実施することが必要です。YOODSでは独自の技術による高精度なキャリブレーションデータを、カメラと共にご提供しています。

## キャリブレーションデータのご提供方法
- YCAM1,2

	YCAM1-2では電子データ(CD-ROM)にてご提供します。データ形式はROSのメッセージのテキスト形式のYAMLファイルです。
- YCAM3

	YCAM3では内蔵ROM、および筐体へのラベル表示にてご提供します。
- その他カメラ

	筐体へのラベル表示および試験成績書(紙媒体)にてご提供します。

## キャリブレーションデータの確認
- YCAM1,2
- YCAM3
- その他カメラ

## キャリブレーションデータの設定

キャリブレーションデータの設定手順はカメラに毎に異なります。ご使用のカメラに応じた手順にて実施ください。
- YCAM1,2
- YCAM3
- その他カメラ

## ROSレベルの操作

RoVIでは/remap_nodeにてRectifyを行います。このノードはキャリブレーションデータをパラメータサーバから取得します。パラメータは下記書式となります。
~~~
remap:
  width: 640
  height: 480
  D: [0.09586000000000001, 0.0, 0.0, 0.0, 0.0]
  K: [1511.29203, 0.0, 339.88041,
      0.0, 1506.44056, 259.60377,
      0.0, 0.0, 1.0]
  R: [1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0]
  P: [1511.29203, 0.0, 339.88041, 0.0,
      0.0, 1506.44056, 259.60377, 0.0,
      0.0, 0.0, 1.0, 0.0]
~~~
- ROSのクセですが、：の後にはスペースが必要なので注意しましょう
- １行の文字数が長すぎるとエラーになるので、行列は複数行で入力しましょう







前提条件
・PCでOSはUbuntu 16.04 LTS。
・ROS Kineticをインストール済み。
・gitをインストール済み。
・~/catkin_wsがROSワークスペースとする。
  (~/catkin_ws以外のROSワークスペースでも可。その場合は以下の例を読み替えること。)


①汎用GigEライブラリ(libaravis)をビルド、インストール

1. ビルド用の前準備
sudo apt-get install automake intltool

2. ソースをDL
cd ~
mkdir aravis
cd aravis
wget http://ftp.gnome.org/pub/GNOME/sources/aravis/0.4/aravis-0.4.1.tar.xz

3. 解凍先でmakeしてインストール
tar xvf aravis-0.4.1.tar.xz
cd aravis-0.4.1
./configure
make
sudo make install

4. 動作用の設定
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib" >> ~/.bashrc
. ~/.bashrc

5. 動作確認
arv-tool-0.4

↓ と表示されればひとまずOK
No device found

②ROSのGigEカメラ汎用ドライバをDL (YOODS版なので注意）
cd ~/catkin_ws/src
git clone https://github.com/YOODS/camera_aravis
cd ..
catkin_make

<3>
YCAM3Dに接続するGigEインターフェースに適切なIPアドレスを設定する。

YCAM3DのIPアドレスは、
	カメラ2つ(192.168.222.1,2)とプロジェクター(192.168.222.10)。

Ubuntuの[システム設定]->[ネットワーク]で、
カメラに接続するGigEインターフェースにIPアドレス(192.168.222.99/24)を設定する。

	[IPv4設定]タブで
		方式：手動
		アドレス：192.168.222.99	24	空

	カメラ2つ(192.168.222.1,2)とプロジェクター(192.168.222.10)へpingが通る。

<4>
物理的に接続して、
take@ubuntu:~$ arv-tool-0.4
SENTECH-17AB755
SENTECH-17AB756
のようにカメラ2台のIDが表示されればOK
（もちろんカメラによってIDはこれらと異なる。）


①OpenCVインストール
sudo apt-get install libopencv-dev


<Node.jsのインストール>
curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
sudo apt-get install nodejs

<rosnodejsその他のインストール>
cd ~
npm install rosnodejs
npm install ws
npm install opencv
npm install canvas

上記でインストールされるrosnodejsは、
RoVIで必要とする機能(ROS Serviceの同期呼び出し機能)が含まれていない。
rosnodejsの最新Gitソースにはその機能が含まれているので、
以下のようにして、最新Gitソースで上書きする。

cd ~
git clone https://github.com/RethinkRobotics-opensource/rosnodejs
cd ~/node_modules/rosnodejs
rm -rf dist
cp -a ~/rosnodejs/src/ dist


<RoVI>
cd ~/catkin_ws/src
git clone https://github.com/YOODS/rovi
cd ..
catkin_make

TODO:
Eigenが必要ならば
cd ~
wget http://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz
mv eigen-eigen-5a0156e40feb/Eigen/ ~/catkin_ws/src/rovi/include/
rm -rf eigen-eigen-5a0156e40feb/


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

yaml/rovi_rosparam_dump.yaml の、camera idやremapのユーザ側での編集など。

現状はdumpは↓の手作業でやってください、と。
rosparam dump ~/catkin_ws/src/rovi/yaml/rovi_rosparam_dump.yaml /rovi

それら手順の下に、/rovi/pshift_genpc を呼んで、なども書く。

