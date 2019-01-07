# RoVI Requirements

## 1. コア機能

### 1-1. 主要諸元

<table>
<tr><td rowspan="2">H/W<td>CPU
<tr><td>NIC<td>Jumboパケット適合
<tr><td rowspan="2">Input<td>カメラ<td>GeV準拠
<tr><<td>プロジェクタ<td>独自
<tr><td rowspan="6">Output<td>2D画像<td>較正画像
<tr><td>解像度<td>1280x1024
<tr><td>フレームレート<td>10fps
<tr><td>点群データ<td>1.ROS PointCloud型<br>2.配列(Numpy)
<tr><td>点群撮像時間<td>0.6秒以内
<tr><td>点群演算時間<td>1.0秒以内(100kポイント)
</table>

### 1-2. 詳細仕様
<table>
<tr><td rowspan="2">モニタリング<td>機器状態<td>ROS Bool型
<tr><td>異常履歴<td>ROS String型
<tr><td rowspan="3">エラー処理<td>ネットワーク瞬断<td>エラーあり<br>自己復帰
<tr><td>GVSP受信エラー<td>エラーなし<br>自動リトライ(aravis)
<tr><td>GVCP送信エラー<td>???
<tr><td rowspan="2">設定<td>初期値<td>ファイル(yaml)
<tr><td>ランニングチェンジ<td>パラメータサーバ(rosparam)
<tr><td rowspan="2">その他<td>ストロボ撮影<td>可
<tr><td>温度補正<td>？
</table>

## 2. 補助機能

<table>
<tr><td rowspan="3">ロボットI/F<td>三菱<td>Host通信機能
<tr><td>Fanuc<td>KARELおよびユーザソケットメッセージ
<tr><td>安川
<tr><td>設置支援<td>Robotキャリブレーション<td>固定およびオンハンド
<tr><td rowspan="2">SGBM<td>出力<td>
<tr><td>フレームレート<td>5fps
<tr><td rowspan="2">エッジ3D化<td>出力<td>点群・Numpy
<tr><td>フレームレート<td>5fps
</table>
