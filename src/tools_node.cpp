#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "rovi_srvs/dialog.h"
#include <string>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

ros::NodeHandle *nh;
ros::Publisher pub;

bool load_img(rovi_srvs::dialog::Request &req,rovi_srvs::dialog::Response &res){
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
	ros::ServiceServer svc1=n.advertiseService("tools/imread",load_img);
	fprintf(stdout,"\nRoVI-Test-Tools comprises services as:\n\t*tools/imread <image file>\n tries reading file and publish to the topic *tools/image. Image format must be mono 8bit\n");
	ros::spin();
	return 0;
}
