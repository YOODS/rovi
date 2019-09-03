# Design Requirement

## 設計基準
<table>
<tr><th>分類<th>小分類<th>設計基準
<tr><td rowspan="2">モデル<td>Class<td>ROSノード単位
<tr><td>Request/Response<td>RosRules v1.0準拠[RosRules](https://github.com/KazukiHiraizumi/RosRules)
<tr><td rowspan="2">データ構造<td>パラメータ<td>Yaml/JSON互換メモリー構造
<tr><td>Naming Rule<td>RosRules v1.0準拠[RosRules](https://github.com/KazukiHiraizumi/RosRules)
</table>

## 機能要件
<table>
<tr><th>大分類<th>中分類<th>小分類<th>設計基準
<tr><td rowspan="5">リクエスト<td rowspan="3">3D撮像<td>遅延<td>0.1s以内
<tr><td>ターンアラウンドタイム<td>5s以内/1Mポイント
<tr><td>インターロック<td>本リクエスト非処理中<sup>(1)</sup>
<tr><td rowspan="2">ストロボ有効<td>遅延<td>0.1s以内
<tr><td>インターロック<td>ライブ状態
<tr><td rowspan="7">パラメータ<td rowspan="2">2D撮像<td>露光時間<td>即時反映
<tr><td>ゲイン設定<td>即時反映
<tr><td rowspan="3">3D撮像<td>露光時間<td>即時反映
<tr><td>ゲイン設定<td>即時反映
<tr><td>演算パラメータ<td>次サイクル反映
<tr><td rowspan="1">プロジェクタ<td>発光強度<td>即時反映
<tr><td>キャリブレーション<td>カメラ<td>再起動可
<tr><td rowspan="7">アウトプット<td rowspan="2">ストリーミング(ステレオ)<td>形式<td>ビットマップ
<tr><td>解像度切替<td>再起動可
<tr><td rowspan="3">3Dデータ<td>形式<td>PointClond像,浮動小数点配列,デプス画
<tr><td>解像度切替<td>再起動可
<tr><td>単位切替<td>m⇔mm再起動してもよい
<tr><td rowspan="2">エラー<td>形式<td>真偽値
<tr><td>識別<td>カメラ接続、3D撮像
</table>

## 非機能要件
<table>
<tr><th>大分類<th>中分類<th>小分類<th>設計基準
<tr><td rowspan="8">エラー<td rowspan="4">検出<td>カメラ切断<td>遅延1s以内
<tr><td>3D撮像<td>遅延1s以内
<tr><td>ストリーミング<td>遅延1s以内
<tr><td>3D撮像リクエスト無効<td>即時
<tr><td rowspan="4">処理<td>カメラ切断<td>自動再接続
<tr><td>3D撮像<td>3D撮像リクエスト中断
<tr><td>ストリーミング<td>自動切断再接続
<tr><td>3D撮像リクエスト無効<td>無視
<tr><td rowspan="3">信頼性<td rowspan="2">リソース消費<td>メモリー<td>2GB以下
<tr><td>CPU<td>定常20%以下
<tr><td>耐久性<td>電源断耐性<td>1000サイクル以上<sup>(2)</sup>
<tr><td rowspan="3">利便性<td rowspan="3">ログ<td>エラーメッセージ<td>履歴をオンメモリ−
<tr><td>サイクルカウント<td>履歴をオンメモリ−
<tr><td>設定変更<td>履歴をオンメモリ−
</table>
  
---
(1)DLP仕様の制約につき、ストロボ有効にて3D撮像処理は不可  
(2)システムの信頼性は担保されている前提で、ファイル書込などアプリケーション起因のNCのみ  
