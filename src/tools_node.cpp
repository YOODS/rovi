#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "rovi/dialog.h"
#include <string>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

ros::NodeHandle *nh;
ros::Publisher pub;

bool change_dir(rovi::dialog::Request &req,rovi::dialog::Response &res){
	if(chdir(req.hello.c_str())==0){
		res.answer="OK";
		return true;
	}
	else{
		res.answer="Failed";
		return false;
	}
}

bool load_img(rovi::dialog::Request &req,rovi::dialog::Response &res){
	cv_bridge::CvImage cv_img;
	cv_img.encoding="mono8";
	cv_img.image=cv::imread(req.hello,CV_LOAD_IMAGE_GRAYSCALE);
	ROS_INFO("tools/imread : %s",req.hello.c_str());
	if(cv_img.image.data!=NULL){
		pub.publish(cv_img.toImageMsg());
		res.answer="OK";
		return true;
	}
	else{
		ROS_ERROR("tools/imread : %s",req.hello.c_str());
		res.answer="File error";
		return false;
	}
}

int main(int argc, char **argv){
	ros::init(argc,argv,"tools_node");
	ros::NodeHandle n;
	nh=&n;
	pub=n.advertise<sensor_msgs::Image>("tools/image",1);
	ros::ServiceServer svc0=n.advertiseService("tools/cd",change_dir);
	ros::ServiceServer svc1=n.advertiseService("tools/imread",load_img);
	ros::spin();
	return 0;
}
