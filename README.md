# RoVIについて

RoVIは3Dビジョンセンサーを組み込んだロボットのアプリケーションを開発するための、ソフトウェア群を提供します。
## 構成
下図にRoVIのソフトウェア構成を示します。コア部(I/O,Base)とUtilityをプラットフォーム層として、これを利用し用途に合わせてApplicationを開発します。基本的な機能はプラットフォーム層を利用できるので、アプリケーションの開発L/Tが短縮出来ます。  
またApplication実装例として、MTM(Master Teaching Method)パッケージも公開予定です(https://github.com/YOODS/MTM)。
<img src="img/fig1.png" width="500px" >
   
## このRepositoryについて  
このRepositoryはYOODS社が提供するYCAM3Dを制御するソフトウェアです。

## インストール方法  
ROSのソースディレクトリ(~/catkin_ws/srcなど)にて
~~~
git clone -b israfel --depth 1 https://github.com/YOODS/rovi.git
~~~
GetingStarted.mdに記載のとおりです。この記載内容をInstall.shにて一括処理することも可能です。
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
<tr><td>/rovi/right/image_raw<td>Image<td>右カメラraw画像
<tr><td>/rovi/right/image_rect<td>Image<td>右カメラrectify画像
<tr><td>/rovi/ycam_ctrl/errlog<td>Strinfg<td>Errorログ
<tr><td>/rovi/ycam_ctrl/stat<td>Bool<td>システム状態
<tr><td>/rovi/Y1<td>Bool<td>撮像結果(X1に対するレスポンス)
</table>

### To subscribe
<table>
<tr><th>Name<th>Type<th>Description
<tr><td>/rovi/X1<td>Bool<td>撮像トリガ
</table>

## ドキュメントリスト  
|ドキュメント名|コンテンツ|
|:----|:----|
|GettingStarted.md|インストール手順についての記載|
|GettingStarted-TX2.md|Jetson-TX2用インストール手順|
|AppDesign.md|roviを利用したアプリケーションの設計について|

## 動作確認済み環境  
|OS|Desktop|ROS|STATUS|
|:----|:----|:----|:----|
|Ubuntu 16.04(xenial)|Unity|kinetic|OK|
|LinuxMint 18.04(sonya)|Xfce|kinetic|OK|
