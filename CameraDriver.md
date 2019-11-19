# RoVIカメラドライバー仕様
<hr>
## パラメータ  
カメラのハードウェアに関わるパラメータの階層構造は以下のようである。
~~~
rovi
  - live
    - camera
      - ... 
  - pshift_genpc
    - camera
      - ... 
    - projector
      - ...
~~~

### 必須パラメータ  
以下のパラメータは必須であり、ドライバーはこれらのパラメータに対する動作を実装しなければならない。  
<table>
<tr><th>パラメータ名<th>概要<th>値:処理
<tr><td>/rovi/pshift_genpc/projector/Go<td>プロジェクター撮影モードを指定する<td>1:シングル撮影を行う<br>2:位相シフト撮影を行う
<tr><td>/rovi/pshift_genpc/projector/Mode<td>プロジェクターパタンをロードする<td>1:位相シフトパタンをロード
</table>

<hr>

## ドライバー  
ドライバーファイルとはrequire(...)にてycamctl.jsに組み込まれる、各カメラ用のオブジェクトである。

### Property
<table>
<tr><th>プロパティ名<th>Type<th>機能
<tr><td>cstat<td>Bool<td>カメラの状態を保持する
<tr><td>pstat<td>Bool<td>プロジェクトタの状態を保持する
<tr><td>cset<td>Function<td>カメラパラメータをセットする
<tr><td>pset<td>Function<td>プロジェクタパラメータをセットする
<tr><td>open<td>Function<td>カメラを接続する。戻り値としてドライバーイベント(後述)を返す。
<tr><td>normal<td>Function<td>カメラの状態を返す
<tr><td>kill<td>Function<td>カメラを切断する
<tr><td>reset<td>Function<td>カメラをリセットする
</table>

<hr>

## ドライバーイベント

### Property
<table>
<tr><th>プロパティ名<th>Type<th>機能
<tr><td>streaming<td>null,非null<td>ストリーミング中は非null
<tr><td>scanStart<td>Function<td>ストリーミング開始。引数として開始ディレイ(ms)を取る。
<tr><td>scanStop<td>Function<td>ストリーミング停止。引数として停止待ち打ち切り時間(ms)を取る。
</table>

### Event
<table>
<tr><th>イベント名<th>説明
<tr><td>stat<td>定周期(1s)発行。引数でカメラ状態(Bool)を渡す
<tr><td>wake<td>カメラ起動完了にて発行
<tr><td>shutdown<td>カメラ切断にて発行
<tr><td>left<td>左カメラ画像取得にて発行。引数でImageとタイムスタンプを渡す
<tr><td>right<td>右カメラ画像取得にて発行。引数でImageとタイムスタンプを渡す
</table>