#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/image_encodings.h>
#include "rovi/dialog.h"
#include "rovi/ImageFilter.h"
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

ros::NodeHandle *nh;
ros::Publisher pub;

cv::Mat rmapx,rmapy;

bool reload(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res){
	res.success=false;
	std::vector<double> D;
   nh->getParam("remap/D",D);
	if(D.size()<5){
		res.message="D NG";
		return true;
	}
	std::vector<double> K;
	nh->getParam("remap/K",K);
	if(K.size()<9){
		res.message="K NG";
		return true;
	}
	std::vector<double> R;
	nh->getParam("remap/R",R);
	if(R.size()<9){
		res.message="R NG";
		return true;
	}
	std::vector<double> P;
	nh->getParam("remap/P",P);
	if(P.size()<12){
		res.message="P NG";
		return true;
	}
	cv::Mat Pro(P),nCam,nRot,nTrans;
	cv::OutputArray oCam(nCam),oRot(nRot),oTrans(nTrans);
	cv::decomposeProjectionMatrix(Pro.reshape(1,3),oCam,oRot,oTrans);
	int width=0,height=0;
	nh->getParam("remap/width",width);
	nh->getParam("remap/height",height);
	cv::Size imgsz(width,height);
	if(imgsz.area()==0){
		res.message="Size NG";
		return true;
	}
	cv::Mat Cam(K),Rot(R);
	cv::initUndistortRectifyMap(Cam.reshape(1,3),D,Rot.reshape(1,3),nCam,imgsz,CV_32FC1,rmapx,rmapy);
	res.success=true;
	res.message="Remap table ready";
	return true;
}
bool remap(rovi::ImageFilter::Request &req,rovi::ImageFilter::Response &res){
	ROS_DEBUG("remap:start");
	cv_bridge::CvImagePtr cv_ptr;
	try{
		cv_ptr=cv_bridge::toCvCopy(req.img,sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("remap:cv_bridge:exception: %s", e.what());
		return false;
	}
	cv::Mat result;
	cv::remap(cv_ptr->image,result,rmapx,rmapy,cv::INTER_LINEAR,cv::BORDER_TRANSPARENT,0);
	cv_ptr->image=result;
	cv_ptr->toImageMsg(res.img);
	pub.publish(res.img);
	ROS_DEBUG("remap:end");
	return true;
}

int main(int argc, char **argv){
	ros::init(argc,argv,"remap_node");
	ros::NodeHandle n;
	nh=&n;
	ros::ServiceServer svc0=n.advertiseService("remap/reload",reload);
	ros::ServiceServer svc1=n.advertiseService("remap/do",remap);
	pub=n.advertise<sensor_msgs::Image>("remap/image",1);
	ros::spin();
	return 0;
}
