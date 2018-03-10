#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include "rovi_srvs/ImageFilter.h"
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

ros::NodeHandle *nh;
ros::Publisher pub;

cv::Mat rmapx,rmapy;
sensor_msgs::CameraInfo cam_info;

bool set_caminfo(sensor_msgs::SetCameraInfo::Request &req,sensor_msgs::SetCameraInfo::Response &res){
	cv::Size image_size=cv::Size(req.camera_info.width,req.camera_info.height);
	cv::Mat P0;
	cv::Mat newCam,newVec,newRot;
//	cv::decomposeProjectionMatrix(P0, newCam, newRot, newVec);
//	cv::initUndistortRectifyMap(camMat,dstCef,R0,newCam,image_size,CV_32FC1,rmapx,rmapy);
	pub.publish(req.camera_info);
	return true;
}
bool remap(rovi_srvs::ImageFilter::Request &req,rovi_srvs::ImageFilter::Response &res){
	cv_bridge::CvImagePtr cv_ptr;
	try{
		cv_ptr=cv_bridge::toCvCopy(req.img,sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("remap:cv_bridge:exception: %s", e.what());
		return false;
	}
	cv::Mat result;
//	cv::remap(cv_ptr->image,result,rmapx,rmapy,cv::INTER_LINEAR,cv::BORDER_TRANSPARENT,0);
//	cv_ptr->image=result;
	cv_ptr->toImageMsg(res.img);
//	pub.publish(res.img);
	return true;
}


int main(int argc, char **argv){
	ros::init(argc,argv,"remap_node");
	ros::NodeHandle n;
	nh=&n;
	ros::ServiceServer svc0=n.advertiseService("remap/setup",set_caminfo);
	ros::ServiceServer svc1=n.advertiseService("remap/do",remap);
//	pub=n.advertise<sensor_msgs::Image>("remap/image",1);
	pub=n.advertise<sensor_msgs::CameraInfo>("remap/camera_info",1);
	ros::spin();
	return 0;
}
