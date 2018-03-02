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

ros::NodeHandle *nh;
ros::Publisher *pub1,*pub2;

bool setup(rovi::DigitalFilter::Request &req,rovi::DigitalFilter::Response &res){
	ROS_INFO("genpc::setup called: %d",req.data.size());
	return true;
}
bool genpc(rovi::GenPC::Request &req,rovi::GenPC::Response &res){
	ROS_INFO("genpc called: %d %d",req.imgL.size(),req.imgR.size());
	sensor_msgs::PointCloud pts;
	pts.header.stamp = ros::Time::now();
	pts.header.frame_id="/map";  //RViz default Frame
	pts.points.resize(10000);
	pts.channels.resize(1);
	pts.channels[0].name="intensities";
	pts.channels[0].values.resize(10000);
//Making dummy points
	for(int i=0;i<100;i++){
		for(int j=0;j<100;j++){
			int k=100*i+j;
			pts.points[k].x=0.02*i-1.0;
			pts.points[k].y=0.02*j-1.0;
 			pts.points[k].z=0.002*(i-50);
			pts.channels[0].values[k]=100;
		}
	}
	pub1->publish(pts);
	sensor_msgs::PointCloud2 pts2;
	sensor_msgs::convertPointCloudToPointCloud2(pts,pts2);
	ROS_INFO("genpc::do %d %d %d\n",pts2.width,pts2.height,pts2.point_step);
	pts2.row_step = pts2.width * pts2.point_step;
	pub2->publish(pts2);
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
	ros::ServiceServer svc0=n.advertiseService("genpc/setup",setup);
	ros::ServiceServer svc1=n.advertiseService("genpc/do",genpc);
	ros::ServiceServer svc2=n.advertiseService("genpc/try",trypc);
	ros::Publisher p1=n.advertise<sensor_msgs::PointCloud>("genpc/pcl",1);
	pub1=&p1;
	ros::Publisher p2=n.advertise<sensor_msgs::PointCloud2>("genpc/pcl2",1);
	pub2=&p2;
	ros::spin();
	return 0;
}
