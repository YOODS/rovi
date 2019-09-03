#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include "rovi/Floats.h"

ros::NodeHandle *nh;
ros::Publisher *pub;

std::string fl2pc_frameid("/hand");
union PACK{ float d[3]; char a[12];};
void base64decode(const std::string& input, std::vector<geometry_msgs::Point32>& out) {
  static const unsigned char kDecodingTable[] = {
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 62, 64, 64, 64, 63,
    52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 64, 64, 64, 64, 64, 64,
    64,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14,
    15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 64, 64, 64, 64, 64,
    64, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
    41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64
  };
  size_t in_len=input.size();
  size_t out_len=in_len/16;
  out.resize(out_len);
  PACK u;
  for(int i=0,j=0;i<out_len;i++){
    for(int k=0;k<12;k+=3){
      uint32_t a = kDecodingTable[input[j++]]&0x3F;
      uint32_t b = kDecodingTable[input[j++]]&0x3F;
      uint32_t c = kDecodingTable[input[j++]]&0x3F;
      uint32_t d = kDecodingTable[input[j++]]&0x3F;
      uint32_t e = (a<<18)|(b<<12)|(c<<6)|d;
      u.a[k] = (e>>16)&0xFF;
      u.a[k+1] = (e>>8)&0xFF;
      u.a[k+2] = e&0xFF;
    }
    out[i].x=u.d[0];
    out[i].y=u.d[1];
    out[i].z=u.d[2];
  }
}

void subn(const rovi::Floats& buf){
  int N=buf.data.size()/3;
  ROS_INFO("points=%d",N);
  sensor_msgs::PointCloud pts;
  pts.header.stamp = ros::Time::now();
  pts.header.frame_id = fl2pc_frameid;
  pts.points.resize(N);
  for (int n=0,i=0; n<N; n++){
    pts.points[n].x=buf.data[i++];
    pts.points[n].y=buf.data[i++];
    pts.points[n].z=buf.data[i++];
  }
  pub->publish(pts);
}
void subs(const std_msgs::String& msg){
  sensor_msgs::PointCloud pts;
  pts.header.stamp = ros::Time::now();
  pts.header.frame_id = fl2pc_frameid;
  base64decode(msg.data,pts.points);
  pub->publish(pts);
}

int main(int argc, char **argv){
  if (argc >= 4)
  {
    fl2pc_frameid = argv[1];
  }
  ros::init(argc, argv, "floats2pc");
  ros::NodeHandle n;
  nh = &n;
  ros::Subscriber s0=n.subscribe("floats",1,subn);
  ros::Subscriber s1=n.subscribe("base64",1,subs);
  ros::Publisher p0=n.advertise<sensor_msgs::PointCloud>("pc",1);
  pub = &p0;
  ros::spin();
  return 0;
}
