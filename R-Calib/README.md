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

  <img src="https://latex.codecogs.com/gif.latex?{}^{b}T_{s}={}^{b}T_{m} {}^{m}T_{c} {}^{c}T_{s}" />

visp_hand2eyeおよび佐藤さんのソルバーは、

<img src="https://latex.codecogs.com/gif.latex?{}^{b}T_{s}">が不変のもと

<img src="https://latex.codecogs.com/gif.latex?{}^{b}T_{m}">および<img src="https://latex.codecogs.com/gif.latex?{}^{c}T_{s}">を既知入力とし
未知の変換<img src="https://latex.codecogs.com/gif.latex?{}^{m}T_{c}">
を求めるものである。

## キャリブレーション原理(固定カメラ)
固定カメラでは求解したい変換は<img src="https://latex.codecogs.com/gif.latex?{}^{b}T_{c}" />である。このため上式の<img src="https://latex.codecogs.com/gif.latex?{}^{m}T_{c}" />に置き換え下記の等式を得る。

  <img src="https://latex.codecogs.com/gif.latex?{}^{m}T_{s}={}^{m}T_{b} {}^{b}T_{c} {}^{c}T_{s}" />

この式から、<img src="https://latex.codecogs.com/gif.latex?{}^{b}T_{m}">に代えて<img src="https://latex.codecogs.com/gif.latex?{}^{b}T_{m}^{-1}">を入力とすることで、同じソルバーにて求解できる。



