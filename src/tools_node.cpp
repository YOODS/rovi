#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "rovi/Dialog.h"
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/surface_matching.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>
#include <opencv2/core/utility.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>

ros::NodeHandle *nh;
ros::Publisher pub1;
ros::Publisher pub2;

bool change_dir(rovi::Dialog::Request &req,rovi::Dialog::Response &res){
	if(chdir(req.hello.c_str())==0){
		res.answer="OK";
		return true;
	}
	else{
		res.answer="Failed";
		return false;
	}
}

bool load_img(rovi::Dialog::Request &req,rovi::Dialog::Response &res){
	cv_bridge::CvImage cv_img;
	cv_img.encoding="mono8";
	cv_img.image=cv::imread(req.hello,CV_LOAD_IMAGE_GRAYSCALE);
	ROS_INFO("tools/imread : %s",req.hello.c_str());
	if(cv_img.image.data!=NULL){
		pub1.publish(cv_img.toImageMsg());
		res.answer="OK";
		return true;
	}
	else{
		ROS_ERROR("tools/imread : %s",req.hello.c_str());
		res.answer="File error";
		return false;
	}
}

bool load_ply(rovi::Dialog::Request &req,rovi::Dialog::Response &res){
	cv::Mat pc=cv::ppf_match_3d::loadPLYSimple(req.hello.c_str(),0);
	sensor_msgs::PointCloud pts;
	pts.header.stamp=ros::Time::now();
	pts.header.frame_id="/map";  //RViz default Frame
	pts.points.resize(pc.rows);
	pts.channels.resize(1);
	pts.channels[0].name="intensities";
	pts.channels[0].values.resize(pc.rows);
	for(int i=0;i<pc.rows;i++){
		pts.points[i].x=pc.at<float>(i,0);
		pts.points[i].y=pc.at<float>(i,1);
		pts.points[i].z=pc.at<float>(i,2);
		pts.channels[0].values[i]=255;
	}
	pub2.publish(pts);
	return true;
}

int main(int argc, char **argv){
	ros::init(argc,argv,"tools_node");
	ros::NodeHandle n;
	nh=&n;
	pub1=n.advertise<sensor_msgs::Image>("tools/image",1);
	pub2=n.advertise<sensor_msgs::PointCloud>("tools/pcl",1);
	ros::ServiceServer svc0=n.advertiseService("tools/cd",change_dir);
	ros::ServiceServer svc1=n.advertiseService("tools/imread",load_img);
	ros::ServiceServer svc2=n.advertiseService("tools/plread",load_ply);
	ros::spin();
	return 0;
}
