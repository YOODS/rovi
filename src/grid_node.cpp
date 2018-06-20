#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "rovi/GetGrid.h"
#include "CircleCalibBoard.h"

CircleCalibBoard cboard;
ros::NodeHandle *nh;
std::string paramK("gridboard/K");
static double px2mm = 0.1;
static std::vector<double> kvec;

bool get_grid(rovi::GetGrid::Request &req, rovi::GetGrid::Response &res)
{
  cv_bridge::CvImagePtr cv_ptr1;
  try
  {
    cv_ptr1 = cv_bridge::toCvCopy(req.img, sensor_msgs::image_encodings::MONO8);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("get_grids:cv_bridge:exception: %s", e.what());
    return false;
  }
  std::vector<cv::Point2f> imagePoints;
  cv::Mat mat;
  if (cboard.scan(cv_ptr1->image, imagePoints, &mat))
  {
    ROS_ERROR("CircleCalibBoard::scan:failed:");
    return false;
  }
  else
  {
    cv_ptr1->image = mat;
    cv_ptr1->encoding = "bgr8";
    cv_ptr1->toImageMsg(res.img);
    std::vector<cv::Point3f> model;
    std::vector<cv::Point2f> scene;
    geometry_msgs::Point p;
    p.z = 0;
    for (int i = 0; i < imagePoints.size(); i++)
    {
      if (imagePoints[i].x == FLT_MAX || imagePoints[i].y == FLT_MAX)
      {
        p.z = -1; // recognition failed
      }
      else
      {
        p.z = 0;
      }
      p.x = imagePoints[i].x;
      p.y = imagePoints[i].y;
      res.grid.push_back(p);
      if (p.z < 0)
      {
        continue;
      }
      cv::Point3f pm(cboard.get_3d_position(i));
      pm *= px2mm;
      cv::Point2f ps(p.x, p.y);
      model.push_back(pm);
      scene.push_back(ps);
    }

/*
    // Test of simple case
    int ncx = (int)cboard.para["n_circles_x"];
    int ncy = (int)cboard.para["n_circles_y"];
    int ind00 = ncx / 2 + ncx * (ncy / 2);
    int ind01 = ind00 + 5;
    int ind10 = ind00 + 5 * ncx;
    int ind11 = ind00 + 5 + 5 * ncx;
    std::cout << "Model\n";
    std::cout << model[ind00] << "\n" << model[ind10] << "\n" << model[ind01] << "\n" << model[ind11] << "\n";
    std::cout << "Scene\n";
    std::cout << scene[ind00] << "\n" << scene[ind10] << "\n" <<scene[ind01] << "\n" << scene[ind11] << "\n";
    std::vector<cv::Point3f> mod;
    std::vector<cv::Point2f> sce;
    mod.push_back(model[ind00]);
    mod.push_back(model[ind10]);
    mod.push_back(model[ind01]);
    mod.push_back(model[ind11]);
    sce.push_back(scene[ind00]);
    sce.push_back(scene[ind10]);
    sce.push_back(scene[ind01]);
    sce.push_back(scene[ind11]);
*/

    cv::Mat kmat(kvec);
    cv::Mat dmat = cv::Mat::zeros(5, 1, cv::DataType<float>::type); // No distortion
    cv::Mat rmat(3, 1, cv::DataType<double>::type);
    cv::Mat tmat(3, 1, cv::DataType<double>::type);
    cv::OutputArray oRmat(rmat), oTmat(tmat);
    cv::solvePnP(model, scene, kmat.reshape(1, 3), dmat, oRmat, oTmat);
    float rx = rmat.at<double>(0, 0);
    float ry = rmat.at<double>(1, 0);
    float rz = rmat.at<double>(2, 0);
    float rw = sqrt(rx * rx + ry * ry + rz * rz);
    res.pose.position.x = tmat.at<double>(0, 0);
    res.pose.position.y = tmat.at<double>(1, 0);
    res.pose.position.z = tmat.at<double>(2, 0);
    res.pose.orientation.x = rw > 0 ? sin(rw / 2) * rx / rw : rx;
    res.pose.orientation.y = rw > 0 ? sin(rw / 2) * ry / rw : ry;
    res.pose.orientation.z = rw > 0 ? sin(rw / 2) * rz / rw : rz;
    res.pose.orientation.w = cos(rw / 2);
    return true;
  }
}

bool reload(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
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
  double umm;
  if (! nh->getParam("gridboard/unitmm", umm))
  {
    ROS_ERROR("GetGrid::paramer \"unitmm\" not found");
    return false;
  }
  px2mm = umm / cboard.para["unitleng"];
  if (! nh->getParam(paramK.c_str(), kvec))
  {
    ROS_ERROR("GetGrid::paramer \"K\" not found");
    return false;
  }
  return true;
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

  ros::ServiceServer svc1 = n.advertiseService("gridboard", get_grid);
  ros::ServiceServer svc2 = n.advertiseService("gridboard/reload", reload);
  std_srvs::Trigger::Request req;
  std_srvs::Trigger::Response res;
  reload(req, res);
  ros::spin();
  return 0;
}
