# ソフトウェア　インデックス

# Launch
<table>
<tr><th>パッケージ<th>名称<th>分類<th>機能<td>Lang<th>状況
<tr><td rowspan="3">rovi<td>ycam3sxga<br>ycam3vga<td>バックエンド<td>YCAM-III撮像制御<td>nodejs<td>完了
<tr><td>p2p<td>ツール<td>カメラ検査(平行面距離)<td>python<td>完了
<tr><td>cropper<td>フロントエンド<td>撮像条件設定・クロップ領域設定<td><td>製作中
<tr><td rowspan="2">rovi-alib<td>r-calib<td>ツール<td>ロボットキャリブレーション<td>python<td>整理中
<tr><td>c-calib<td>ツール<td>カメラキャリブレーション<td>python<td>製作(移植)予定
<tr><td rowspan="4">rovi-I<td>f-socket<td rowspan="4">インタフェース(産ロボ)<td>Fanuc用<td rowspan="4">nodejs<td>完了
<tr><td>m-socket<td>Mitsubishi用<td>完了
<tr><td>n-socket<td>Nachi用<td>製作予定
<tr><td>y-socket<td>Yasukawa用<td>製作予定
<tr><td>rovi-finder<td>finder<td>バックエンド<td>ワーク認識・測位<br>オフライン共用(plyファイル入力)<td>python<td>整理中
<tr><td rowspan="2">master-teach<td>UI<td>フロントエンド<td>rovi-finder用フロントエンド<td>python<td>製作中
<tr><td>??<td>オフラインツール<td>PLY編集・レシピ編集<td>??<td>製作予定
</table>

# Node
<table>
<tr><th>パッケージ<th>名称<th>機能<th>Lang<th>状況
<tr><td rowspan="5">rovi<td>remap<td>画像レクティファイ<td>C++<td>完了
<tr><td>genpc<td>点群生成<td>C++<td>完了
<tr><td>grid<td>キャリブ板認識・測位<td>C++<td>完了
<tr><td>ycamctl<td>カメラ制御<td>nodejs<td>完了
<tr><td>paraman<td>パラメータ変更通知<td>nodejs<td>完了
<tr><td>rqt-ezui<td>rqt-ezui<td>簡易UI作成ツール<td>python<td>製作中
</table>

# Library/Tool
<table>
<tr><th>パッケージ<th>名称<th>機能<th>Lang<th>入力<th>出力<th>状況
<tr><td rowspan="2">yds3d.so<td>-<td>位相シフト<td rowspan="2">C++<td>-<td>-<td>完了
<tr><td>-<td>○列(キャリブ板)認識・測位<td>-<td>-<td>完了
<tr><td rowspan="2">rovi<td>gvload.js<td>カメラパラメータ読出<td>nodejs<td>GVCP<td>Param<td>製作済
<tr><td>notifier.js<td>パラメータ変更通知<td>nodejs<td>Param<td>Event<td>製作済
<tr><td rowspan="3">rovi-finder<td>??<td>点群プリプロセス<td>python<td>ndarray<td>ndarray<td>???
<tr><td>ppf<td>特徴点抽出<td>python<td>ndarray<td>ndarray<td>???
<tr><td>icp<td>測位<td>python<td>ndarray<td>tf<td>???
<tr><td>WPC(TMCカスタム)<td>??<td>精密ICP<td>python<td>ndarray<td>tf<td>???
</table>

<hr>  
# Windows Tool
<table>
<tr><th>名称<th>機能<th>Lang<th>状況
<tr><td>test_ycam<td>カメラキャリブレーション<td>C++<td>製作済
</table>
