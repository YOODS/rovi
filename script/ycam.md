# YCAMドライバ

 YCAMは、カメラとプロジェクタから構成されている。これらの構成は、YCAMのバージョンによって異なるため、上位階層からのアクセス手段がバージョン毎に変わるという不都合が生じる。
 そこで抽象化YCAMをNodejsライブラリ(require...)で実現する。

<table border>
<tr><th><th>カメラ構成<th>プロジェクタ構成
<tr><td>YCAM-I,II<td>Sentechカメラ&times;2<td>TCP/Serial&times;1
<tr><th>YCAM-III<th>GigeVision&times;1<td>Gige Vision Command Protocolを共用
</table>

## Property

## Method
1. open(nh,id1,id2,addr)
  - nh:ノードハンドル
  - id1:#1カメラID
  - id2:#2カメラID
  - addr:TCPシリアルのIPアドレス:ポート
  - 戻り値:EventEmitter。詳細は後述
2. set(obj)
  - obj:YCAMに設定するパラメータリスト。以下Key:Value一覧
  - TriggerMode:'On'または'Off'
  - AcquisitionFrameRate:Number
  - Gain:Number
  - ExposureTimeAbs:Number(&micro;秒)
3. get(Array)
  - Array:YCAMから取得するパラメータ名の配列
  - 戻り値:YCAMから取得したパラメータリスト。
4. stat()
  - 戻り値:boolean。YCAMが管理するノードがすべて正常のときTrue
5. pset(cmd)
  - cmd:String。Projectorに送信するコマンド。詳細は後述

## EventEmitter
openメソッドの戻り値から、以下のYCAMイベントがキャッチできます。
1. cam_L(image)
左カメラの画像が送られます
2. cam_R(image)
右カメラの画像が送られます
3. pinfo(string)
プロジェクタからのInfo文字列が送られます

## Projectorコマンド
1. コマンド
  - l 
  -
  -

## 使用例
~~~
const ros=require('rosnodejs');

const sens=require('./ycam1h.js');    //パッケージに非ず。ファイルです。

const rosNode=await ros.initNode('/test9');

const sensEv=sens.open(rosNode,'Basler-1','Basler-2','192.168.2.66:5000');

sensEv.on('cam_L',function(img){

});

~~~