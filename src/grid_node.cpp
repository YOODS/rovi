#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Trigger.h>
#include "rovi_srvs/GetGrid.h"
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "CircleCalibBoard.h"

//ros::Publisher pub;

CircleCalibBoard cboard;

bool get_grid(rovi_srvs::GetGrid::Request &req,rovi_srvs::GetGrid::Response &res){
//	ROS_INFO("get_grids notif %d",req.notif.size());
	cv_bridge::CvImagePtr cv_ptr2,cv_ptr1;
	try{
		cv_ptr1=cv_bridge::toCvCopy(req.img,sensor_msgs::image_encodings::MONO8);
//		cv_ptr2=cv_bridge::toCvCopy(req.img,sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("get_grids:cv_bridge:exception: %s", e.what());
		return false;
	}
	cboard.init();
	std::vector<cv::Point2f> imagePoints;
//	imagePoints.resize(cboard.para["n_circles_x"] * cboard.para["n_circles_y"]);
	cv::Mat mat;
	if(cboard.scan(cv_ptr1->image,imagePoints,&mat)){
//	if(cboard.scan(cv_ptr1->image,imagePoints,&cv_ptr2->image)){
		ROS_ERROR("CircleCalibBoard::scan:failed:");
	}
	else{
		geometry_msgs::Point p;
		p.z=0;
		for(int i=0;i<imagePoints.size();i++){
			p.x=imagePoints[0].x;
			p.y=imagePoints[0].y;
			res.grid.push_back(p);
		}
	}
	cv_ptr1->image=mat;
	cv_ptr1->encoding="bgr8";
	cv_ptr1->toImageMsg(res.img);
//	cv_ptr2->toImageMsg(res.img);
//	pub.publish(cv_ptr->toImageMsg());
	return true;
}

ros::NodeHandle *nh;
bool reload(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res){
	for(std::map<std::string, double>::iterator itr=cboard.para.begin();itr!=cboard.para.end();++itr){
		std::string pname("gridboard/");
		pname+=itr->first;
		if(nh->hasParam(pname)) nh->getParam(pname,itr->second);
	}
	return true;
}

int main(int argc, char **argv){
	ros::init(argc,argv,"grid_node");
	ros::NodeHandle n;
	nh=&n;
	cboard.para["bin_type"]=1;

	ros::ServiceServer svc1=n.advertiseService("gridboard",get_grid);
	ros::ServiceServer svc2=n.advertiseService("gridboard/reload",reload);
	std_srvs::Trigger::Request req;
	std_srvs::Trigger::Response res;
	reload(req,res);
	ros::spin();
	return 0;
}
