# tf_network.cpp のマニュアル
**【役割】**　
1. **カメラから画像を入力（Subscribe）**：main関数
2. **画像処理**：find_marker関数
3. **処理結果を出力（publish）**：main関数、find_marker関数

※ find_marker_L, find_marker_Rは、main関数とfind_marker関数の仲介役

### ★ includeライブラリ
```
#include "ros/ros.h"	//rosの機能を使用
#include "sensor_msgs/Image.h"	//画像をトピックに渡すために必要
#include <sstream>	//文字列処理に使用するライブラリ（今回は不要かも）
#include <cv_bridge/cv_bridge.h>  //画像の形式ををROSとOpenCV用に変換するために必要
#include "boost/bind.hpp"	//これを使って、右カメラと左カメラのコールバック関数をまとめようとしたけど難しかったので不要
#include <iostream>	//標準入出力に必要
```

### ★ グローバル変数
トピックへのpublishに使用するクラスのポインタ変数（pubL, pubR）
```
static ros::Publisher *pubL, *pubR;
```

### ★ main関数
1. kizania_nodeという名前のノードを作成（ros::init）
```
ros::init(argc, argv,"kidzania_node");
```
2. ノードの初期化（ros::NodeHandle）
```
ros::NodeHandle n;
```

3. トピックにsensor_msgs::Image型の画像を発行する準備（ros::Publisher）
    - 左カメラと右カメラの結果画像をpublishするために、（kidzania/image_left_out, kidzania/image_right_out）という名前のトピックにsensor_msgs::Image型（rosの画像形式）の画像を発行に使うインスタンス（pL, pR）を作成＆初期化
```
ros::Publisher pL = n.advertise<sensor_msgs::Image>("kidzania/image_left_out", 1000);	//左カメラ用
ros::Publisher pR = n.advertise<sensor_msgs::Image>("kidzania/image_right_out", 1000);	//右カメラ用
```

4. グローバル変数（pubL, pubR）に、上で作成したインスタンスのアドレスを格納
```
pubL = &pL;	
pubR = &pR;
```

5. トピック（/rovi/left/image_rect, kidzania/image_right_out）にsensor_msgs::Image型の画像を受信するためのインスタンス（subL, subR）を作成
	- この際に**コールバック関数（find_marker_L, find_marker_R）** が呼び出される
```
ros::Subscriber subL = n.subscribe("/rovi/left/image_rect", 1000, find_marker_L);
ros::Subscriber subR = n.subscribe("/rovi/right/image_rect", 1000, find_marker_R);
```

6. コールバック関数を呼ぶために必要なおまじない
```
ros::spin();
```

### ★ find_marker_L, find_marker_R 関数 
カメラからの入力画像（buf）と左右を区別するラベル（左：0, 右：1）を **find_marker関数**に渡す仲介役

```
void find_marker_L(sensor_msgs::Image buf){
	find_marker(buf, 0);	//左カメラ用
}
	
void find_marker_R(sensor_msgs::Image buf){
	find_marker(buf, 1);	//右カメラ用
}	
```
### ★ find_marker 関数 
1. ROS形式の画像をOpenCV用に変換

```
cv_bridge::CvImagePtr cv_ptr;	//OpenCV用のポインタを用意
cv_ptr = cv_bridge::toCvCopy(buf, sensor_msgs::image_encodings::BGR8);	//ROSからOpenCV形式に変換。cv_ptr->imageがcv::Matフォーマット
```

2. グレースケール処理、平滑化
```
cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);	//グレースケール変換
cv::GaussianBlur(gray_image, gaussian_image, cv::Size(5, 5), 3, 3);	//平滑化
```

3. 画素値の分布を正規化
	- 画素値の分布の偏りを改善するために分布を正規化
	- 正規化後の分布のパラメータ（平均mと標準偏差s）は、マーカーなどに応じて手動で調節
	- 正規化の結果、画素値が負の場合は0, 255以上のものは255に変更して格納

```
cv::meanStdDev(gaussian_image, mean, stddev);	//平滑化後の画素値の分布を計算

//以下は正規化後の分布の形状を決定するパラメータ
int s = 90;	//分布の標準偏差を設定するパラメータ
int m = 100;	//分布の平均値を設定するパラメータ
int rows = gaussian_image.rows;
int cols = gaussian_image.cols;
cv::Mat mean_Mat = cv::Mat::ones(rows, cols, CV_32FC1) * mean[0];
cv::Mat m_Mat = cv::Mat::ones(rows, cols, CV_32FC1) * m;
gaussian_image.convertTo(g_img, CV_32FC1);
norm_img = (g_img - mean_Mat) / stddev[0] * s + m_Mat;   //画素値の正規化
norm_img.convertTo(normalized_int, CV_32SC1);	
normalized_image = cv::Mat::ones(rows, cols, CV_8UC1);
for (int i = 0; i < norm_img.rows; i++) {   //正規化の結果、画素値が負の場合は0, 255以上のものは255に変更して配列に格納
	int* nintP = normalized_int.ptr<int>(i);
	uchar* nimgP = normalized_image.ptr<uchar>(i);
	for (int j = 0; j < norm_img.cols; j++) {
		int value = nintP[j];
		if (value < 0) {
			value = 0;
		}else if (value > 255) {
			value = 255;
		}
		nimgP[j] = value;
	}
}
```

4. 二値化処理
	- **ボール**を検出する場合は、大津の二値化処理を適用（１行目のコメントを解除して、２行目をコメントアウト）
	- **電球**を検出する場合は、固定の閾値（const int threshold=245）を適用
```
//cv::threshold(normalized_image, thr_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);	//ボールを検出する場合は、こちらの大津の二値化処理を使用
cv::threshold(normalized_image, thr_image, threshold, 255, cv::THRESH_BINARY);	//電球を検出する場合は、固定の閾値（const int threshold = 245）で二値化
```
	
5. 輪郭抽出
	- cv::findContoursで輪郭を検出（contoursには、すべての輪郭が格納されている（はず））
```
cv::findContours(thr_image, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);	
```

6. カラー画像に変換
	- 輪郭や検出した円をカラーで描くために、カラー画像に変換
```
cv::cvtColor(normalized_image, color_img, cv::COLOR_GRAY2BGR);	//グレースケール画像をRBGに変換
```

7. 輪郭の描画＆円検出
	- for文で各輪郭ごとの処理を行う
	- idxは何番目の輪郭かを表す数字
	- 各輪郭の長さ（L）と輪郭で囲まれた面積（S）から、各輪郭の円形度(4πS / L^2)を計算
	- 電球の場合は円形度が0.7と面積が1000以上（ボールの場合は円形度0.8以上かつ100以上（にしたほうが良い）
	- 
```
int idx = 0, flag = 0;
//各輪郭ごとの処理
if (contours.size()) {
	for (; idx >= 0; idx = hierarchy[idx][0]) {
		drawContours(color_img, contours, idx, cv::Scalar(80, 244, 255), 2);	// 輪郭を描く。輪郭の色はレモンイエロー
		const std::vector<cv::Point>& c = contours[idx];
		double area = fabs(cv::contourArea(cv::Mat(c)));	//輪郭で囲まれた面積Sを計算
		double perimeter = cv::arcLength(c, true); 	//輪郭の長さを計算	

		//円形度(4πS / L^2)の高い輪郭を検出
		double circle_deg;
		float radius;
		cv::Point2f center;
		circle_deg = 4 * M_PI*area / pow(perimeter, 2.0);	//円形度の計算

		//円を推定＆円と中心座標を描画
		if (circle_deg > 0.8 && area > 100) {
			cv::minEnclosingCircle(c, center, radius);	//最小外接円を計算
			cv::circle(color_img, center, radius, cv::Scalar(255, 0, 255), 2);	//外接円を描画
			cv::drawMarker(color_img, center, cv::Scalar(255, 0, 255));	//中心座標を描画
		}
	}
}
```

8. 結果画像をROS形式に変換してpublish
```
sensor_msgs::Image img;
cv_ptr->image = color_img;
cv_ptr->encoding="bgr8";	
cv_ptr->toImageMsg(img);	//toImageMsg()でOpenCVからrosの型に変換
if (label==0){
	pubL->publish(img);
}else{
	pubR->publish(img);
}
```

9. マーカーの座標をカメラ座標に変換してpublish（まだ実装していないが追加の必要あり）
```
/** ３D座標への変換 **/
	std::cout << "sequence" << label << buf.header.seq << std::endl;	
}	
```
