# Robot Calibration

## 記号の定義
1. 座標変換行列
<img src="https://latex.codecogs.com/gif.latex?{}^{A}T_{B}" />
は、座標系Aからみた座標系Bの変換行列

2. 座標系記号

|記号|座標系|
|:----|:----|
|b|ロボットベース|
|m|ロボットエンド|
|c|カメラ|
|s|対象物(剛体)|

## キャリブレーション原理(ハンドアイ)
ハンドアイでは以下の変換が成立する

  <img src="https://latex.codecogs.com/gif.latex?{}^{b}T_{s}={}^{b}T_{m}\cdot{}^{m}T_{c}\cdot{}^{c}T_{s}~~~~-(1)" />

visp_hand2eyeおよび佐藤さんのソルバーは、

<img src="https://latex.codecogs.com/gif.latex?{}^{b}T_{s}">が不変のもと

<img src="https://latex.codecogs.com/gif.latex?{}^{b}T_{m}">および<img src="https://latex.codecogs.com/gif.latex?{}^{c}T_{s}">を既知入力とし
未知の変換<img src="https://latex.codecogs.com/gif.latex?{}^{m}T_{c}">
を求めるものである。

## キャリブレーション原理(固定カメラ)
固定カメラでは求解したい変換は<img src="https://latex.codecogs.com/gif.latex?{}^{b}T_{c}" />である。このため上式の<img src="https://latex.codecogs.com/gif.latex?{}^{m}T_{c}" />に置き換え下記の等式を得る。

  <img src="https://latex.codecogs.com/gif.latex?{}^{m}T_{s}={}^{m}T_{b}\cdot{}^{b}T_{c}\cdot{}^{c}T_{s}~~~~-(2)" />

この式から、<img src="https://latex.codecogs.com/gif.latex?{}^{b}T_{m}">に代えて<img src="https://latex.codecogs.com/gif.latex?{}^{m}T_{b}(={}^{b}T_{m}^{-1})">を入力とすることで、同じソルバーにて求解できる。

## キャリブレーション結果の評価
CSV形式のファイルにてキャリブレーション結果を書き出します。

1. 出力場所  
$HOME/.ros/以下

2. input.txt
入力情報(<img src="https://latex.codecogs.com/gif.latex?{}^{b}T_{m}">,<img src="https://latex.codecogs.com/gif.latex?{}^{c}T_{s}">)を保存します。

3. result.txt
入力の各行に対応した検算結果(<img src="https://latex.codecogs.com/gif.latex?{}^{b}T_{s}">,<img src="https://latex.codecogs.com/gif.latex?{}^{m}T_{s}">)を保存します。
これらは本来不変なTransformですが様々な誤差によりバラつきが生じます。

## Install

1. VISPスタックをインストール
https://visp.inria.fr/

~~~
sudo apt-get install ros-kinetic-visp
sudo apt-get install ros-kinetic-visp-hand2eye-calibration
~~~

## Run

RoVIの起動後、以下を追加起動します。

1. 起動
~~~
roslaunch calib.launch
~~~

2. Tuning

ソフトウェアはリアルタイムにキャリブ板を認識し、その剛体座標を/gridboard/tfトピックに出力します。  
キャリブ板を正しく認識できているかどうかは、/gridboard/image_outトピックで確認します。正しい認識は以下のステップを経たものです。
- 輪郭を緑色でマーキング
- そのうち楕円と分類したものを黄色でマーキング
- さらに、大きさ、間隔などが指定した条件内の●にインデクス番号を赤字で表示  
下図は認識が正しい時の例です
<img src="fig2.png">  
一方、認識ができないときは下のような表示となります。
<img src="fig1.png">  
これは二値化のしきい値が適切でないときです。二値化のしきい値はbin_param0パラメータで以下のように設定します。

~~~
rosparam set /gridboard/bin_param0 50
~~~

## Topics

1. To subscribe

|トピック名|型|説明|
|:----|:----|:----|
|/rovi/left/image_rect|Image|基準カメラ(左)のレクティファイ画像|
|/robot/tf|Transform|ベース座標基準のロボット機械端座標|
|/robot/euler|Transform|ベース座標基準のロボット機械端座標(オイラー角)。このトピックに発行したデータはQuaternion変換され、/robot/carteに再発行されます。手入力時のインタフェースとして用意されています|
|/solver/X0|Bool|取得したデータをクリアする|
|/solver/X1|Bool|データ(物体とロボットのTransformペア)をバッファにストアする。/solver/X1にて処理完了を通知します|
|/solver/X2|Bool|ストアされたデータから、機械端からカメラへの座標変換を算出し、パラメータ/robot/calib/mTc(固定カメラではbTc)に出力する。/solver/Y2にて計算完了を通知します|

2. To publish

|トピック名|型|説明|
|:----|:----|:----|
|/gridboard/image_out|Image|キャリブ板の認識結果|
|/gridboard/tf|Transform|カメラ画像から推定した、キャリブ板のカメラ座標に対する座標変換|
|/solver/Y1|Bool|X1処理完了でアサートされます|
|/solver/Y2|Bool|X2処理完了でアサートされます|

## 操作

1. データ入力  
通常はロボットから通信経由で座標値が得られ、/robot/tfにパブリッシュされます。通信ができないときは、<b>rqt</b>などのツールにて/robot/eulerに発行します。/robot/eulerはオイラー角表記の座標変換入力をQuaternionに変換し即/robot/tfに再発行します。
2. コマンド  
/solver/X0~X2をトリガします。各接点の機能はTopic説明を参考。基本的な操作は以下のようになります。
  - 座標バッファクリア  
  /solver/X0をアサート(メッセージを発行)します
  - 座標(ロボット＋キャリブ板)をストア  
  キャリブ板をカメラ視界に捉えつつロボットを動かし、/solver/X1をアサートします。これによりその時の座標がバッファにストアされます。ロボット座標がて入力の場合は、/solver/X1をアサートしたあと、/robot/tfまたは/robot/eulerをアサートします。
  - キャリブレーション計算  
/solver/X2をアサートすることで、カメラへの座標変換を算出します。演算が終わると/solver/Y2がアサートされます。結果はパラメータ/robot/calib/mTc(ハンドアイ)、/robot/calib/bTc(固定カメラ)、に書き出されます。
3. キャリブレーション結果の評価  
$HOME/.rosのinput.txtおよびresult.txtから結果を評価しましょう
4. 画像の保存  
お馴染みのimsave.jsをこのディレクトリ下にコピーしています。保存するのは左右が結合されたRaw画像です。適切なファイル名に改変して使用します。
~~~
./imsave 1
~~~

## パラメータ一覧
川村さんのドキュメントから抜粋です。

- "n_circles_x"	X軸方向のマーカーの数(デフォルト値=13)
- "n_circles_y"	Y軸方向のマーカーの数(デフォルト値=19)
- "unitleng"		マーカー重心間の距離(デフォルト値=60.0) _実際のキャリブ板の寸法に変更すること_
- "distance_between_circles"	マーカー端から次のマーカー端までの最大距離のマーカー直径に対する比率(デフォルト値=1.2)

- "n_circles_minimum"	カメラ画像内に写るべき最小マーカー数(デフォルト値=9)
- "min_rate"	キャリブ板全体の大きさの最小許容サイズ(画像サイズに対する比率。デフォルト値=0.25) 
- "max_rate"	キャリブ板全体の大きさの最大許容サイズ(画像サイズに対する比率。デフォルト値=2.00)
- "fitscore"	取り出された輪郭線を無理やり楕円近似したときに、どの程度の観測点が楕円上に乗っていれば輪郭線==楕円と判断するか(デフォルト値=0.95)
- "fitrange"	上記の乗っていると判断するための観測点と楕円との距離(許容誤差。デフォルト値=1.50)

- "do_qualize_hist"	ヒストグラム均一化を行う(1)か否(0)か。(デフォルト値=0)
- "do_smoothing"	スムージングを行う(1)か否(0)か。(デフォルト値=1)
- "bin_type"	二値化タイプ(0: 通常二値化。1: 判別分析二値化。2: 適応二値化)(デフォルト値=0)	(2の場合はあまり上手くいかないので、ほとんど使ってない。)
- "bin_param0"	bin_type==0の場合、閾値。2の場合ブロックサイズ。
- "bin_param1"	bin_type==2の場合、平均値からのオフセット値。
