## tf_network.cpp
カメラから画像を入力（Subscribe）⇒　画像処理　⇒　処理結果を出力（publish）

### グローバル変数
トピックへのpublishに使用するクラスのポインタ変数（pubL, pubR）
```
static ros::Publisher *pubL, *pubR;
```

### main関数
1. kizania_nodeという名前のノードを作成（ ⇐ ros::init）
2. ノードの初期化（ ⇐ ros::init）
3. トピックにsensor_msgs::Image型の画像を発行する準備（ ⇐ ros::Publisher）
    - 左カメラと右カメラの結果画像をpublishするために、（kidzania/image_left_outkidzania/image_right_out）という名前のトピックにsensor_msgs::Image型（rosの画像形式）の画像を発行に使うインスタンス（pL, pR）を作成＆初期化
4. 上で作成したインスタンスのアドレスを格納するグローバル変数（pubL, pubR）を初期化
5. トピック（/rovi/left/image_rect, kidzania/image_right_out）にsensor_msgs::Image型の画像を受信するためのインスタンス（subL, subR）を作成
　　- この際にコールバック関数（find_marker_L, find_marker_R）が呼び出される

```
int main(int argc, char** argv){
	
	// 1. 新しいノード（kidzania_node）の作成
	ros::init(argc, argv,"kidzania_node");
	
	// 2. ノードへのハンドラの作成（ノードの初期化）
	ros::NodeHandle n;
	
	// 3. トピックにsensor_msgs::Image型の画像を発行する準備
	ros::Publisher pL = n.advertise<sensor_msgs::Image>("kidzania/image_left_out", 1000);
	ros::Publisher pR = n.advertise<sensor_msgs::Image>("kidzania/image_right_out", 1000);
	
	// 4. アドレスを格納
	pubL = &pL;
	pubR = &pR;
	
	// 5. トピック（chatter）にsensor_msgs::Image型の画像を受信（コールバック関数処理）
	ros::Subscriber subL = n.subscribe("/rovi/left/image_rect", 1000, find_marker_L);
	ros::Subscriber subR = n.subscribe("/rovi/right/image_rect", 1000, find_marker_R);
	
	ros::spin();
	
	return 0;
}
```

### find_marker_L, find_marker_R 関数 
カメラからの入力画像（buf）と左右を区別するラベル（左：0, 右：1）を **find_marker関数**に渡す仲介役

```
void find_marker_L(sensor_msgs::Image buf){
	find_marker(buf, 0);	//左カメラ用
}
	
void find_marker_R(sensor_msgs::Image buf){
	find_marker(buf, 1);	//右カメラ用
}	
```
### find_marker 関数 

