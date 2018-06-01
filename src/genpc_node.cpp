#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/ChannelFloat32.h>
#include "rovi/DigitalFilter.h"
#include "rovi/GenPC.h"
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ps_main.h"

ros::NodeHandle *nh;
//ros::Publisher *pub1,*pub2;

bool reload(rovi::DigitalFilter::Request &req,rovi::DigitalFilter::Response &res){
	ROS_INFO("genpc::setup called: %d",req.data.size());
	return true;
}
bool genpc(rovi::GenPC::Request &req,rovi::GenPC::Response &res){
	ROS_INFO("genpc called: %d %d",req.imgL.size(),req.imgR.size());

	int width = req.imgL[0].width;
	int height = req.imgL[0].height;

	ROS_INFO("genpc img w, h: %d %d",width,height);

/*
//Making dummy points
	sensor_msgs::PointCloud pts;
	pts.header.stamp = ros::Time::now();
	pts.header.frame_id="/map";  //RViz default Frame
	pts.points.resize(10000);
	pts.channels.resize(1);
	pts.channels[0].name="intensities";
	pts.channels[0].values.resize(10000);
	for(int i=0;i<100;i++){
		for(int j=0;j<100;j++){
			int k=100*i+j;
			pts.points[k].x=0.02*i-1.0;
			pts.points[k].y=0.02*j-1.0;
 			pts.points[k].z=0.002*(i-50);
			pts.channels[0].values[k]=100;
		}
	}
*/

	std::vector<double> vecQ;
	nh->getParam("genpc/Q",vecQ);
	if(vecQ.size()!=16){
		ROS_ERROR("Param Q NG");
		return false;
	}

	//位相シフト計算パラメータ
	PS_PARAMS param={
		.search_div = PH_SEARCH_DIV,
		.bw_diff = BW_DIFF,
		.brightness = BRIGHTNESS,
		.max_ph_diff = MAX_PH_DIFF,
		.max_parallax = MAX_PARALLAX,
		.min_parallax = MIN_PARALLAX,
		.max_tex_diff = MAX_TEX_DIFF,
		.speckle_range = SPECKLE_RANGE,
	        .speckle_phase = SPECKLE_PHASE,
	        .speckle_pixel = SPECKLE_PIXEL,
		.ls_points = LS_POINTS,
	};

	// カメラ, 位相シフトを初期化
	ps_init(width, height);

	// パラメータを設定
	ps_setparams(param);

	// 位相シフトデータ画像(左13枚, 右13枚の読込み)
	try{
		for (int j=0; j<13; j++) {
			cv::Mat img = cv_bridge::toCvCopy(req.imgL[j], sensor_msgs::image_encodings::MONO8)->image;
			ps_setpict(0, j, img);
			cv::imwrite(cv::format("/tmp/imgL%d.jpg", j), img); // or png
		}
		for (int j=0; j<13; j++) {
			cv::Mat img = cv_bridge::toCvCopy(req.imgR[j], sensor_msgs::image_encodings::MONO8)->image;
			ps_setpict(1, j, img);
			cv::imwrite(cv::format("/tmp/imgR%d.jpg", j), img); // or png
		}
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("genpc:cv_bridge:exception: %s", e.what());
		return false;
	}

	// 計算実行
	Eigen::MatrixXd &diff=ps_exec();
	Eigen::Matrix4d Q;
	memcpy(Q.data(),vecQ.data(),sizeof(double)*4*4);
	int N=genPC(diff,ps.texture,ps.mask[0],ps.pt,Q);
	// TODO
	ROS_ERROR("genPC returned N=%d", N);

	// 点群出力
	sensor_msgs::PointCloud pts;
	pts.header.stamp = ros::Time::now();
	pts.header.frame_id="/map";  //RViz default Frame
	pts.points.resize(N);
	pts.channels.resize(3);
	pts.channels[0].name="r";
	pts.channels[0].values.resize(N);
	pts.channels[1].name="g";
	pts.channels[1].values.resize(N);
	pts.channels[2].name="b";
	pts.channels[2].values.resize(N);
	for(int n=0;n<N;n++){
		pts.points[n].x=_pcd[n].coord[0];
		pts.points[n].y=_pcd[n].coord[1];
		pts.points[n].z=_pcd[n].coord[2];
		pts.channels[0].values[n] = _pcd[n].col[0] / 255.0;
		pts.channels[1].values[n] = _pcd[n].col[1] / 255.0;
		pts.channels[2].values[n] = _pcd[n].col[2] / 255.0;
		// TODO
		if (n < 20 || (N - 20) < n) {
			ROS_ERROR("n=%d x,y,z=%f,%f,%f r,g,b=%f,%f,%f",
					n,
					pts.points[n].x,
					pts.points[n].y,
					pts.points[n].z,
					pts.channels[0].values[n],
					pts.channels[1].values[n],
					pts.channels[2].values[n]
			);
		}
	}

	outPLY("/tmp/test.ply");

//	pub1->publish(pts);
	res.pc=pts;
//	sensor_msgs::PointCloud2 pts2;
//	sensor_msgs::convertPointCloudToPointCloud2(pts,pts2);
//	ROS_INFO("genpc::do %d %d %d\n",pts2.width,pts2.height,pts2.point_step);
//	pts2.row_step = pts2.width * pts2.point_step;
//	pub2->publish(pts2);
	return true;
}
bool trypc(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res){
	rovi::GenPC srv;
	return genpc(srv.request,srv.response);
}
int main(int argc, char **argv){
	ros::init(argc,argv,"genpc_node");
	ros::NodeHandle n;
	nh=&n;
	ros::ServiceServer svc0=n.advertiseService("genpc/reload",reload);
	ros::ServiceServer svc1=n.advertiseService("genpc",genpc);
	ros::ServiceServer svc2=n.advertiseService("genpc/try",trypc);
//	ros::Publisher p1=n.advertise<sensor_msgs::PointCloud>("genpc/pcl",1);
//	pub1=&p1;
//	ros::Publisher p2=n.advertise<sensor_msgs::PointCloud2>("genpc/pcl2",1);
//	pub2=&p2;
	ros::spin();
	return 0;
}
