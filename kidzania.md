# kidzaniaのコードのマニュアル

## tf_network.cpp
カメラから画像を入力（Subscribe）⇒　画像処理　⇒　処理結果を出力（publish）

### グローバル変数
トピックへのpublishに使用するクラスのポインタ変数（pubL, pubR）
"""
static ros::Publisher *pubL, *pubR;

"""

### main関数
1. kizania_nodeという名前のノードを作成（ ⇐ ros::init）
2. ノードの初期化（ ⇐ ros::init）
3. トピックにsensor_msgs::Image型の画像を発行する準備（ ⇐ ros::Publisher）
    - 左カメラの結果画像をpublishするために、kidzania/image_left_outという名前のトピックにsensor_msgs::Image型（rosの画像形式）の画像を発行に使うインスタンス（pL）を初期化
    - 右カメラの結果画像をpublishするために、kidzania/image_right_outという名前のトピックにsensor_msgs::Image型（rosの画像形式）の画像を発行に使うインスタンス（pR）を初期化
4. 上で初期化したインスタンスのアドレスを格納するグローバル変数（pubL, pubR）を初期化
5. 

"""
	//新しいノード（kidzania_node）の作成
	ros::init(argc, argv,"kidzania_node");
	
	//ノードへのハンドラの作成（ノードの初期化）
	ros::NodeHandle n;
	
	//トピックにsensor_msgs::Image型の画像を発行する準備
	ros::Publisher pL = n.advertise<sensor_msgs::Image>("kidzania/image_left_out", 1000);
	ros::Publisher pR = n.advertise<sensor_msgs::Image>("kidzania/image_right_out", 1000);
	
	pubL = &pL;
	pubR = &pR;
	
	
	//トピック（chatter）にsensor_msgs::Image型の画像を受信（コールバック関数処理）
	ros::Subscriber subL = n.subscribe("/rovi/left/image_rect", 1000, find_marker_L);
	ros::Subscriber subR = n.subscribe("/rovi/right/image_rect", 1000, find_marker_R);
	
	ros::spin();
	
	return 0;
"""
