# MasterTeach specification

## RoVI-Cropper

- デフォルト名前空間：/cropper  
- camera座標系にて処理
- 合成時は一枚目のカメラ座標に揃える

### Topics(to subscribe)  
|Topic|タイプ|説明|
|:----|:----|:----|
|/robot/tf|Transform|ロボットからロボット座標を受け取る|
|パラメタ参照"source"/floats|Floats|シーン点群データ。デフォルト/rovi/floats|
|sphare/set|Bool|球Crop範囲初期化(点群重心から66%点を含む範囲)|
|box/set|Bool|箱Crop範囲初期化(点群重心から66%点を含む範囲)|

## Topics(to publish)  
|Topic|タイプ|説明|
|:----|:----|:----|
|clear|Bool|シーンクリア|
|tf|Transform|シーン生成時ロボット座標(合成時は1枚目の位置)|
|source/floats|Floats|撮影時の点群。合成点群|
|floats|Floats|Cropping後点群|

### Parameters(this)
|Parameter|説明|
|:----|:----|
|source|シーン点群ソース|
|box|箱クロップ座標(ox,oy,oz,sx,sy,sz)<br>box/setにて初期化|
|sphare|球Crop座標(ox,oy,r)sphare/setにて初期化|

### Parameters(exernal)
|Parameter|説明|
|:----|:----|


## RoVI-Finder

- デフォルト名前空間：/finder

### Topics(to subscribe)  
|Topic|タイプ|説明|
|:----|:----|:----|
|/robot/tf|Transform|ロボットからロボット座標を受け取る|
|パラメタ参照"source"/floats|Floats|シーン点群データ|
|solve|Bool|シーン解析|
|regMaster|Bool|マスター登録。ファイル名は(パラメータ"model/name")参照。tfも同時同所に保存|

### Topics(to publish)  
|Topic|タイプ|説明|
|:----|:----|:----|
|scene/floats|Floats|撮影時の点群|
|model/floats|Floats|マスター点群|
|scene/cannotpick/floats|Floats|ICPの結果(Fitnessが閾値以下の場合の点群)|
|scene/icpresult/floats|Floats|ICPの結果OKの点群(クロップしたロボット座標の点群)|

### Parameters(this)
|Parameter|説明|
|:----|:----|
|model/name|モデルPLYファイル名|
|model/tf|モデル作成時ロボット座標|
|source|シーン点群ソース|
|frame_id|座標系選択(camera|world|hand)|
|algor|マッチングアルゴリズム|

### Parameters(exernal)
|Parameter|説明|
|:----|:----|
|/rovi/camera/mTc|キャリブレーション(handeye)|

<hr>

## MasterTeach

デフォルト名前空間：/mt

## Topics(to subscribe)

|Topic|タイプ|説明|
|:----|:----|:----|
|X0|std_msgs/Bool|キャプチャデータ消去|
|X1|std_msgs/Bool|キャプチャ|
|X2|std_msgs/Bool|シーン解析|
|X3|std_msgs/Bool|レシピ読込|
|reset|std_msgs/Bool|エラーリセット|

## Topics(to publish)

|Topic|タイプ|説明|
|:----|:----|:----|
|Y[0-9]|std_msgs/Bool|Response for /solver/X[0-9]|
|message|std_msgs/String|メッセージ|
|error|std_msgs/String|エラーメッセージ|

## Parameters

|Parameter|説明|
|:----|:----|
