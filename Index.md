# ソフトウェア　インデックス

# Launch
<table>
<tr><th>パッケージ<th>名称<th>分類<th>機能<th>状況
<tr><td rowspan="5">rovi<td>ycam3sxga<br>ycam3vga<td>バックエンド<td>YCAM-III撮像制御<td>製作済
<tr><td>p2p<td>ツール<td>カメラ検査(平行面距離)<td>製作済
<tr><td>??<td>フロントエンド<td>撮像条件設定<td>製作予定
<tr><td>??<td>フロントエンド<td>クロップ領域設定<td>製作予定
<tr><td>r-calib<td>ツール<td>ロボットキャリブレーション<td>製作中
<tr><td>rovi-finder<td>finder<td>バックエンド<td>ワーク認識・測位<br>オフライン共用(plyファイル入力)<td>製作済
<tr><td rowspan="2">master-teach<td>UI<td>フロントエンド<td>rovi-recogフロントエンド<td>???
<tr><td>??<td>オフラインツール<td>PLY編集・レシピ編集<td>??
</table>

# Node
<table>
<tr><th>パッケージ<th>名称<th>機能<th>Lang<th>状況
<tr><td rowspan="5">rovi<td>remap<td>画像レクティファイ<td>C++<td>製作済
<tr><td>genpc<td>点群生成<td>C++<td>製作済
<tr><td>grid<td>キャリブ板認識・測位<td>C++<td>製作済
<tr><td>ycamctl<td>カメラ制御<td>nodejs<td>製作済
<tr><td>paraman.js<td>パラメータ変更通知<td>nodejs<td>製作済
<tr><td rowspan="4">rovi-I<td>f-socket<td>Fanucインタフェース<td rowspan="4">nodejs<td>製作済
<tr><td>m-socket<td>Mitsubishiインタフェース<td>製作済
<tr><td>n-socket<td>Nachiインタフェース<td>製作予定
<tr><td>y-socket<td>Yasukawaインタフェース<td>製作予定
<tr><td>rqt-ezui<td>rqt-ezui<td>簡易UI作成ツール<td>Python<td>製作中
</table>

# Library/Tool
<table>
<tr><th>パッケージ<th>名称<th>機能<th>Lang<th>入力<th>出力<th>状況
<tr><td rowspan="2">rovi<td>gvload.js<td>カメラパラメータ読出<td>nodejs<td>GVCP<td>Param<td>製作済
<tr><td>notifier.js<td>パラメータ変更通知<td>nodejs<td>Param<td>Event<td>製作済
<tr><td rowspan="3">rovi-finder<td>??<td>点群プリプロセス<td>Python<td>ndarray<td>ndarray<td>???
<tr><td>ppf<td>特徴点抽出<td>Python<td>ndarray<td>ndarray<td>???
<tr><td>icp<td>測位<td>Python<td>ndarray<td>tf<td>???
<tr><td>WPC(TMCカスタム)<td>??<td>精密ICP<td>Python<td>ndarray<td>tf<td>???
</table>

<hr>
# Windows Tool
<table>
<tr><th>名称<th>機能<th>Lang<th>入力<th>出力<th>状況
<tr><td>???<td>カメラキャリブレーション<td>---<td>剛体変換<td>Param<td>製作済
</table>
