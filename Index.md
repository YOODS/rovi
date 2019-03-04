# ソフトウェア　インデックス

# Launch
<table>
<tr><th>パッケージ<th>名称<th>分類<th>機能<th>状況
<tr><td rowspan="5">rovi<td>ycam3sxga<br>ycam3vga<td>バックエンド<td>YCAM-III撮像制御<td>製作済
<tr><td>p2p<td>ツール<td>カメラ検査(平行面距離)<td>製作済
<tr><td>??<td>フロントエンド<td>撮像条件設定<td>製作予定
<tr><td>??<td>フロントエンド<td>クロップ領域設定<td>製作予定
<tr><td>r-calib<td>ツール<td>ロボットキャリブレーション<td>製作中
<tr><td rowspan="4">rovi-I<td>f-socket<td>インタフェース<td>Fanucインタフェース<td>製作済
<tr><td>m-socket<td>インタフェース<td>Mitsubishiインタフェース<td>製作済
<tr><td>n-socket<td>インタフェース<td>Nachiインタフェース<td>???
<tr><td>y-socket<td>インタフェース<td>Yasukawaインタフェース<td>???
<tr><td>rovi-finder<td>finder<td>バックエンド<td>ワーク認識・測位<br>オフライン動作可能のこと(plyファイル入力)<td>製作済
<tr><td rowspan="2">master-teach<td>??<td>フロントエンド<td>rovi-recogフロントエンド<td>???
<tr><td>??<td>オフラインツール<td>PLY編集・レシピ編集<td>??
</table>

# Executable
<table>
<tr><th>パッケージ<th>名称<th>機能<th>状況
<tr><td>rqt-ezui<td>rqt-ezui<td>UIツール<td>製作中
</table>

# Library
<table>
<tr><th>パッケージ<th>名称<th>機能<th>入力<th>出力<th>状況
<tr><td rowspan="3">rovi-finder<td>??<td>点群プリプロセス<td>ndarray<td>ndarray<td>???
<tr><td>ppf<td>特徴点抽出<td>ndarray<td>ndarray<td>???
<tr><td>icp<td>測位<td>ndarray<td>tf<td>???
</table>
