#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>
#include "rovi/ImageFilter.h"
#include "rovi/Floats.h"
#include "CircleCalibBoard.h"

CircleCalibBoard cboard;
ros::NodeHandle *nh;
std::string paramK("gridboard/K");
static std::vector<double> kvec;
static ros::Publisher *pub1, *pub2, *pub3, *pub4, *pub5;
static double torelance=1.0;

void solve(sensor_msgs::Image src){
  std_msgs::Bool done;
  geometry_msgs::Transform tf;
  cv_bridge::CvImagePtr cv_ptr1;
  try{
    cv_ptr1 = cv_bridge::toCvCopy(src, sensor_msgs::image_encodings::MONO8);
  }
  catch(cv_bridge::Exception& e){
    ROS_ERROR("get_grids:cv_bridge:exception: %s", e.what());
    pub4->publish(done);
    return;
  }
  std::vector<cv::Point2f> imagePoints;
  cv::Mat mat;
  
  try {
    int cbres=cboard.scan(cv_ptr1->image, imagePoints, &mat);
    sensor_msgs::Image img;
    cv_ptr1->image=mat;
    cv_ptr1->encoding="bgr8";
    cv_ptr1->toImageMsg(img);
    pub1->publish(img);

    if(cbres){
      ROS_WARN("CircleCalibBoard::scan:failed:");
      pub4->publish(done);
      return;
    }
  }
  catch(char *str) {
    ROS_WARN("CircleCalibBoard::scan:failed:");
    pub4->publish(done);
    return;
  }
  std::vector<cv::Point3f> model;
  std::vector<cv::Point2f> scene;
  rovi::Floats buf;
  int N=imagePoints.size();
  buf.data.resize(5*N);
  for (int i = 0; i < N; i++){
    cv::Point3f pm(cboard.get_3d_position(i));
    cv::Point2f ps(imagePoints[i].x, imagePoints[i].y);
    int i5=5*i;
    buf.data[i5]= pm.x;
    buf.data[i5+1]= pm.y;
    buf.data[i5+2]= pm.z;
    if (ps.x == FLT_MAX || ps.y == FLT_MAX){
      buf.data[i5+3]=buf.data[i5+4]=FLT_MAX;
      continue;
    }
    buf.data[i5+3]= ps.x;
    buf.data[i5+4]= ps.y;
    model.push_back(pm);
    scene.push_back(ps);
  }
  pub3->publish(buf);
  N=model.size();

  cv::Mat kmat(kvec);
  cv::Mat Kmat=kmat.reshape(1,3);
  cv::Mat dvec = cv::Mat::zeros(5, 1, cv::DataType<float>::type); // No distortion
  cv::Mat rvec(3, 1, cv::DataType<double>::type);
  cv::Mat tvec(3, 1, cv::DataType<double>::type);
  cv::OutputArray oRvec(rvec), oTvec(tvec);
  cv::solvePnP(model, scene, Kmat, dvec, oRvec, oTvec);
  float rx = rvec.at<double>(0, 0);
  float ry = rvec.at<double>(1, 0);
  float rz = rvec.at<double>(2, 0);
  float rw = sqrt(rx * rx + ry * ry + rz * rz);
  tf.translation.x = tvec.at<double>(0, 0);
  tf.translation.y = tvec.at<double>(1, 0);
  tf.translation.z = tvec.at<double>(2, 0);
  tf.rotation.x = rw > 0 ? sin(rw / 2) * rx / rw : rx;
  tf.rotation.y = rw > 0 ? sin(rw / 2) * ry / rw : ry;
  tf.rotation.z = rw > 0 ? sin(rw / 2) * rz / rw : rz;
  tf.rotation.w = cos(rw / 2);

  cv::Mat Rmat(3,3,cv::DataType<double>::type);
  cv::OutputArray oRmat(Rmat);
  cv::Rodrigues(rvec,oRmat);
  double errMax=0,errAve=0;
  int errX,errY;
  for(int i=0;i<N;i++){
    cv::Point3f pm=model[i];
    cv::Mat XYZ(3,1,cv::DataType<double>::type);
    XYZ.at<double>(0,0)=pm.x;
    XYZ.at<double>(1,0)=pm.y;
    XYZ.at<double>(2,0)=pm.z;
    cv::Mat xyz=Rmat*XYZ+tvec;
    cv::Mat uv1=Kmat*xyz;
    double s=uv1.at<double>(2,0);
    double u=uv1.at<double>(0,0)/s;
    double v=uv1.at<double>(1,0)/s;
    cv::Point2f ps=scene[i];
    double dx=ps.x-u;
    double dy=ps.y-v;
    double err=sqrt(dx*dx+dy*dy);
    errAve+=err;
    if(err>errMax){
      errMax=err;
      errX=floor(u);
      errY=floor(v);
    }
  }
  errAve/=N;
  if(errAve<torelance){
    pub2->publish(tf);
    done.data=true;
  }
  pub4->publish(done);
  ROS_WARN("Ave %f  Max.err %f(%d,%d)",errAve,errMax,errX,errY);
  rovi::Floats stats;
  stats.data.resize(2);
  stats.data[0]=errAve;
  stats.data[1]=errMax;
  pub5->publish(stats);
}

void reload(std_msgs::Bool e)
{
  for (std::map<std::string, double>::iterator itr = cboard.para.begin(); itr != cboard.para.end(); ++itr)
  {
    std::string pname("gridboard/");
    pname += itr->first;
    if (nh->hasParam(pname))
    {
      nh->getParam(pname, itr->second);
    }
  }
  cboard.init();
  if (! nh->getParam(paramK.c_str(), kvec))
  {
    ROS_ERROR("GetGrid::paramer \"K\" not found");
    return;
  }
  if (! nh->getParam("gridboard/torelance", torelance))
  {
    ROS_WARN("GetGrid::paramer \"torelance\" not found");
  }
  ROS_WARN("grid::param::reload %f",torelance);
}

int main(int argc, char **argv)
{
  if (argc >= 2)
  {
    paramK = argv[1];
    std::cout << "K=" << paramK << "\n";
  }
  ros::init(argc, argv, "grid_node");
  ros::NodeHandle n;
  nh = &n;
  cboard.para["bin_type"] = 1;

  ros::Subscriber s1=n.subscribe("gridboard/image_in", 1, solve);
  ros::Subscriber s2=n.subscribe("gridboard/X0", 1, reload);
  std_msgs::Bool msg;
  reload(msg);
  ros::Publisher p1 = n.advertise<sensor_msgs::Image>("gridboard/image_out", 1);
  pub1 = &p1;
  ros::Publisher p2 = n.advertise<geometry_msgs::Transform>("gridboard/tf", 1);
  pub2 = &p2;
  ros::Publisher p3 = n.advertise<rovi::Floats>("gridboard/floats", 1);
  pub3 = &p3;
  ros::Publisher p4 = n.advertise<std_msgs::Bool>("gridboard/done", 1);
  pub4 = &p4;
  ros::Publisher p5 = n.advertise<rovi::Floats>("gridboard/stats", 1);
  pub5 = &p5;
  ros::spin();
  return 0;
}
