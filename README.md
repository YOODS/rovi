# RoVI

RoVIは3Dビジョンセンサーを組み込んだロボットのアプリケーションを開発するための、ソフトウェア群を提供します。
## Structure
下図にRoVIのソフトウェア構成を示します。I/O,Base,Utilityをプラットフォーム層として、これを利用し用途に合わせてApplicationを開発します。基本的な機能はプラットフォーム層を利用できるので、アプリケーションの開発L/Tが短縮出来ます。  
またApplication実装例として、MTM(Master Teaching Method)パッケージも公開しています(https://github.com/YOODS/MTM)。
<img src="img/fig1.png" width="500px" >

   
## このRepositoryについて  
このRepositoryはYOODS社が提供する、YCAM3Dを制御するソフトウェアです。

## インストール方法  
GetingStarted.mdに記載のとおりです。この記載内容をInstall.shにて一括処理することも可能です。
~~~
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