#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include "rovi/Floats.h"

ros::NodeHandle *nh;
ros::Publisher *pub;

void subn(const rovi::Floats& buf){
  int N=buf.data.size()/3;
  ROS_INFO("points=%d",N);
  sensor_msgs::PointCloud pts;
  pts.header.stamp = ros::Time::now();
  pts.header.frame_id = "/hand";
  pts.points.resize(N);
  for (int n=0,i=0; n<N; n++){
    pts.points[n].x=buf.data[i++];
    pts.points[n].y=buf.data[i++];
    pts.points[n].z=buf.data[i++];
  }
  pub->publish(pts);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "floats2pc");
  ros::NodeHandle n;
  nh = &n;
  ros::Subscriber s0=n.subscribe("floats",1,subn);
  ros::Publisher p0=n.advertise<sensor_msgs::PointCloud>("pc",1);
  pub = &p0;
  ros::spin();
  return 0;
}
