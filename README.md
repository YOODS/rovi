# RoVI/Ramiel edition

 Ramiel editionの目標はRoVIの
- 信頼性
- 処理速度

の向上にあります。この目標のため以下の変更を行っています。

1. Imageメッセージの共有メモリ化

容量の大きいメッセージによるノード間通信遅延がROSの欠点のひとつと言われています。Ramiel EditionではImageメッセージを共有メモリ化し通信負荷を大幅に低減しました。対象となる箇所は
  - GigEカメラドライバ(camera_aravis) - カメラコントローラ(ycam_ctrl)間
  - イメージレクティファイア(remap_node) - カメラコントローラ(ycam_ctrl)間

2. SGBMをコアパッケージから削除

SGBMはコアパッケージから削除され、RoVIアプリのひとつに位置づけました(RoVIアプリはRoVIコア起動後に追加起動します)。

3. ライブラリ化を徹底

インクルードされているClassの外部(ライブラリ)化を徹底し、見通しのよいコードに整理しました。


## 追加パッケージのインストール

1. shm-typed-array

NodejsのSys-V共有メモリパッケージです。
~~~
npm install shm-typed-array
~~~

2. camera_aravis

最新にupdateします。
~~~
roscd camera_aravis
git pull
~~~
リビルドも忘れず


## 起動

1. SXGAモード
~~~
roslaunch rovi ycam3sxga.launch
~~~

2. VGAモード
~~~
roslaunch rovi ycam3vga.launch
~~~

