#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <stdlib.h>

ros::NodeHandle *nh;
static ros::Publisher *pub1;
static tf2_ros::TransformBroadcaster *broadcaster;

void send(std_msgs::Bool e)
{
	cv::Point3f markerpos(1.23, 2.34, 3.45);

	geometry_msgs::Transform tf;
	tf.translation.x=markerpos.x;
	tf.translation.y=markerpos.y;
	tf.translation.z=markerpos.z;
	tf.rotation.x=0;
	tf.rotation.y=0;
	tf.rotation.z=0;
	tf.rotation.w=1;

	pub1->publish(tf);
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "camera";
	transformStamped.child_frame_id = "test";
	transformStamped.transform=tf;
	broadcaster->sendTransform(transformStamped);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv,"test_tf_node");
	ros::NodeHandle n;
	nh = &n;

	broadcaster=new tf2_ros::TransformBroadcaster;

	ros::Subscriber s3=n.subscribe("test/request", 1, send);
	ros::Publisher p1 = n.advertise<geometry_msgs::Transform>("test/response", 1);
	pub1 = &p1;

	std_msgs::Bool msg;
	send(msg);

	ros::spin();

	return 0;
}
