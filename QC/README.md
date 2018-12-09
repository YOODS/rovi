# QC-Tools

このパッケージではroviを使った以下のテストができます。

1. 明度
2. 合焦度
3. 再投影誤差

## 合焦度評価

1. 起動
~~~
roslaunch hist.launch
~~~

### Topics

1. To subscribe

|トピック名|型|説明|
|:----|:----|:----|
|/rovi/left/image_raw|Image|基準カメラ(左)の生画像|
|/rovi/right/image_raw|Image|右カメラの生画像|

2. To publish

|トピック名|型|説明|
|:----|:----|:----|
|/rovi/left/hist/image|Image|左カメラ入力画像に統計値をオーバレイした画像|
|/rovi/left/hist/plot|Image|左カメラヒストグラム|
|/rovi/right/hist/image|Image|右カメラ入力画像に統計値をオーバレイした画像|
|/rovi/right/hist/plot|Image|右カメラヒストグラム|

### Parameters

|パラメータ名|説明|初期値|
|:----|:----|----:|
|/hist/Range|ヒストグラムグラフの縦軸最大値|1000|
|/hist/Font|ヒストグラムグラフの文字サイズ|1|


## 再投影誤差

-- 準備中
