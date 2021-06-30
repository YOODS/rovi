#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <sstream>
#include <cv_bridge/cv_bridge.h>
// #include "boost/bind.hpp"
#include <iostream>
#include <std_msgs/Bool.h>

#define GET_CAMERA_LABEL(camno) (camno==0?'L':(camno==1?'R':'?'))
constexpr int CAMERA_NUM = 2;

struct marker{
    int seq;
    double x;
    double y;
    void set_data(int seq, int x = -1, int y = -1){
        this->seq = seq;
        this->x = x;
        this->y = y;
        this->print_data();
    }
    void print_data()
    {
        ROS_INFO("test_node::print_data seq=%d x=%lf y=%lf", this->seq, this->x, this->y);
    }
    bool check_data()
    {
      	bool ret=false;
        if ((this->x < 0) || (this->y < 0)) {
	    	ROS_ERROR("test_node::check NG. (%lf, %lf)", this->x, this->y);
    	}else{
	    	ROS_INFO("test_node::check OK. (%lf, %lf)", this->x, this->y);
            ret=true;
	    }
        return ret;    
    }
};

static ros::Publisher *pubL, *pubR;
ros::NodeHandle *nh;
static marker makL,makR;
static std::vector<std::string> paramP = {"test/left/P", "test/right/P"};
static std::vector<double> pvec[CAMERA_NUM];

//画像処理して３D座標を返す
void find_marker(sensor_msgs::Image buf, int label){
	const int threshold = 245;
	
	std::cout << "enter callback" << std::endl;	
	/** 画像処理 **/
	cv_bridge::CvImagePtr cv_ptr;	//OpenCV用のポインタを用意
	cv_ptr = cv_bridge::toCvCopy(buf, sensor_msgs::image_encodings::BGR8);	//ROSからOpenCV形式に変換。cv_ptr->imageがcv::Matフォーマット
	
	//画像の前処理
	cv::Mat image, gray_image, gaussian_image, thr_image, norm_img, g_img, normalized_int, normalized_image;
	cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);	//グレースケール変換
	cv::GaussianBlur(gray_image, gaussian_image, cv::Size(5, 5), 3, 3);	//平滑化

	//画素値の正規化（平均と標準偏差）
	cv::Scalar mean, stddev;
	cv::meanStdDev(gaussian_image, mean, stddev);
	int s = 90;	//標準偏差
	int m = 100;	//平均
	int rows = gaussian_image.rows;
	int cols = gaussian_image.cols;
	cv::Mat mean_Mat = cv::Mat::ones(rows, cols, CV_32FC1) * mean[0];
	cv::Mat m_Mat = cv::Mat::ones(rows, cols, CV_32FC1) * m;
	gaussian_image.convertTo(g_img, CV_32FC1);
	norm_img = (g_img - mean_Mat) / stddev[0] * s + m_Mat;	//正規化処理
	norm_img.convertTo(normalized_int, CV_32SC1);	// CV_8Sは8bit(1byte) = char型
	normalized_image = cv::Mat::ones(rows, cols, CV_8UC1);
	for (int i = 0; i < norm_img.rows; i++) {
		int* nintP = normalized_int.ptr<int>(i);
		uchar* nimgP = normalized_image.ptr<uchar>(i);
		for (int j = 0; j < norm_img.cols; j++) {	//画素値が負の場合は0, 255以上の場合は255に設定して格納
			int value = nintP[j];
			if (value < 0) {
				value = 0;
			}else if (value > 255) {
				value = 255;
			}
			nimgP[j] = value;
		}
	}

	//二値化処理
	//cv::threshold(normalized_image, thr_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);	//大津の二値化（ボール用）
	cv::threshold(normalized_image, thr_image, threshold, 255, cv::THRESH_BINARY);	//固定の閾値（電球用）
	
	
	//輪郭抽出
	cv::Mat color_img;
	std::vector<std::vector<cv::Point>> contours;
	std::vector< cv::Vec4i > hierarchy;
	cv::findContours(thr_image, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);	
	cv::cvtColor(normalized_image, color_img, cv::COLOR_GRAY2BGR);	//グレースケール画像をRBGに変換

	//輪郭のうち円形度の高いものを検出＆円を推定
	int idx = 0, flag = 0;
	cv::Point2f center(-1.,-1.);
	if (contours.size())
	{
		for (; idx >= 0; idx = hierarchy[idx][0])
		{
			drawContours(color_img, contours, idx, cv::Scalar(80, 244, 255), 2);	// i 番目の輪郭を描く。輪郭の色はレモンイエロー
			const std::vector<cv::Point>& c = contours[idx];
			double area = fabs(cv::contourArea(cv::Mat(c)));	//輪郭で囲まれた面積S
			double perimeter = cv::arcLength(c, true); //輪郭の長さ

			//円形度(4πS / L^2)の高い輪郭を検出
			double circle_deg;
			float radius;
			// cv::Point2f center(-1.,-1.);
			circle_deg = 4 * M_PI*area / pow(perimeter, 2.0);

			//円を推定＆円と中心座標を描画
			if (circle_deg > 0.7 && area > 1000) {	//電球用
			//if (circle_deg > 0.8 && area > 100) {	//ボール用				
				cv::minEnclosingCircle(c, center, radius);	//最小外接円を計算
				cv::circle(color_img, center, radius, cv::Scalar(255, 0, 255), 5);	//外接円を描画
				cv::drawMarker(color_img, center, cv::Scalar(255, 0, 255));	//中心座標を描画
				break;
			}

		}
	}
	if (label==0)
	{
		makL.set_data(buf.header.seq,center.x,center.y);
		std::cout << "set_data_L_label="<< label << std::endl;
	}
	else
	{
		makR.set_data(buf.header.seq,center.x,center.y);
		std::cout << "set_data_R_label="<< label << std::endl;
	}
	
	//結果画像をROS用形式に変換
	sensor_msgs::Image img;
	cv_ptr->image = color_img;
	cv_ptr->encoding="bgr8";	
	cv_ptr->toImageMsg(img);	//toImageMsg()でOpenCVからrosの型に変換
	if (label==0){
		pubL->publish(img);
	}else{
		pubR->publish(img);
	}
	
	/** ３D座標への変換 **/
	if (makL.seq == makR.seq)
	{
		if (makL.check_data() && makR.check_data())
		{
			// std::cout << "seq =" << buf.header.seq << std::endl;

		}
		
	}
	

	// std::cout << "sequence" << label << buf.header.seq << std::endl;
	// // printf("label=%d\n",label);
	// std::cout << "label="<< label << makL.check_data() << std::endl;
	// std::cout << "label="<< label << makR.check_data() << std::endl;
	
}
void reload(std_msgs::Bool e)
{
	ROS_INFO("test_node LD_PATH=%s",getenv("LD_LIBRARY_PATH"));
	
	for( int camno=0; camno < CAMERA_NUM; ++camno ){
    	if (! nh->getParam(paramP[camno].c_str(), pvec[camno])) {
    		ROS_ERROR("test_node::parameter \"P(%c)\" not found",GET_CAMERA_LABEL(camno));
    		return;
    	}
        std::cout << "test_node::parameter pvec(" << GET_CAMERA_LABEL(camno) << ")=";
        for (size_t i = 0; i < pvec[camno].size(); ++i) {
            std::cout << pvec[camno][i] << "; ";
        }
        std::cout << std::endl;
	}
}

void find_marker_L(sensor_msgs::Image buf){
	find_marker(buf, 0);
}
	
void find_marker_R(sensor_msgs::Image buf){
	find_marker(buf, 1);
}	
	

int main(int argc, char** argv){
	for (int i = 0; i < argc; i++)
	{
		std::cout << "argc :" << argc << "argv :" << argv[i] << std::endl;
	}
	for( int camno=0; camno < CAMERA_NUM; ++camno )
	{
		if (argc >= camno + 2)
		{
			paramP[camno] = argv[camno+1];
		}
		std::cout << "P(" << GET_CAMERA_LABEL(camno) << ")=" << paramP[camno] << "\n";
	}

	// 新しいノード（kidzania_node）の作成
	ros::init(argc, argv,"kidzania_node");
	
	// ノードへのハンドラの作成（ノードの初期化）
	ros::NodeHandle n;
	nh = &n;
	
	//p行列取得
	std_msgs::Bool msg;
	reload(msg);
	// トピックにsensor_msgs::Image型の画像を発行する準備
	ros::Publisher pL = n.advertise<sensor_msgs::Image>("kidzania/image_left_out", 1000);
	ros::Publisher pR = n.advertise<sensor_msgs::Image>("kidzania/image_right_out", 1000);
	
	// アドレスを格納
	pubL = &pL;
	pubR = &pR;
	
	// トピック（chatter）にsensor_msgs::Image型の画像を受信（コールバック関数処理）
	ros::Subscriber subL = n.subscribe("/rovi/left/image_rect", 1000, find_marker_L);
	ros::Subscriber subR = n.subscribe("/rovi/right/image_rect", 1000, find_marker_R);
	
	ros::spin();
	
	return 0;
}
