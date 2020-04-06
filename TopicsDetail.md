# Topic詳細

## トピック(サブスクライブ)
1. /rovi/X1 : Bool  
位相シフト撮影をリクエストします。このリクエストはセンサー状態(/rovi/ycam_ctrl/stat)がTrueでなければ、受け付けられません。またレスポンスは/rovi/Y1に出力されます。  
なおメッセージのBool値は無視されます。

## トピック(パブリッシュ)
1. /rovi/ycam_ctrl/stat : Bool  
センサー(YCAM3D)の状態を出力します。接続状態で定期的にTrue、切断状態でFlaseを出力します。
2. /rovi/Y1 : Bool  
位相シフト撮影のリクエスト/rovi/X1のレスポンスです。撮像成功でTrue、失敗でFlaseを出力します。
3. /rovi/ps_pc : sensor_msgs/PointCloud  
撮像リクエスト(/rovi/X1)にて取得した3次元データを、PointCloud形式で出力します。
4. /rovi/ps_floats : rovi_msgs/Floats  
/rovi/ps_pcのジオメトリ(Points[])のみをfloatの配列として出力します。これはPythonでPointCloudを処理する際の利便性への配慮です。Pythonコードでサブスクするのは以下のように行い、ndarray形式で点群データの処理が可能です。
~~~
import numpy as np
from rospy.numpy_msg import numpy_msg
from rovi.msg import Floats
  :
  :
def callback_floats(msg)
  ary=np.reshape(msg.data,(-1,3))  # N行x3列の配列として取得
  :
  :
rospy.Subscriber("~floats",numpy_msg(Floats),callback_floats)
~~~
5. /rovi/ps_base64 : String  
ps_floatsをbase64エンコードした文字列に出力します。こちらはfloats[]のメモリーリーク対策のための出力です。Pythonコードでサブスクするのは以下のように行います。
~~~
  :
  :
~~~
6. /rovi/image_depth : sensor_msgs/Image  
撮像リクエスト(/rovi/X1)にて取得したデプス画像(mono16)を出力します。画素値は1/256mm単位の16ビットで深度を表しています。深度０はパラメータの */rovi/pshift_genpc/calc/depth_base* で与えられた、カメラ座標のZ軸値(mm単位)を基準とします。
7. /rovi/left/image_raw : sensor_msgs/Image  
2D画像は撮像リクエストとは無関係に、常時ストリーミングされます。このトピックは左カメラraw画像(mono8)をストリーミング出力します。
8. /rovi/left/image_rect : sensor_msgs/Image  
左カメラrectify画像(mono8)をストリーミング出力します。
9. /rovi/right/image_raw : sensor_msgs/Image  
右カメラraw画像(mono8)をストリーミング出力します。
10. /rovi/right/image_rect : sensor_msgs/Image  
右カメラrectify画像(mono8)をストリーミング出力します。

