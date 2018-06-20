#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "rovi/GenPC.h"
#include "ps_main.h"

// for disparityCallback() and depthCallback()
//#include <stereo_msgs/DisparityImage.h>
//#include <sensor_msgs/Image.h>
//#include <image_geometry/stereo_camera_model.h>

ros::NodeHandle *nh;
//ros::Publisher *pub1,*pub2;

// Phase Shift method calc parameters
PS_PARAMS param =
{
  .search_div = PH_SEARCH_DIV,
  .bw_diff = BW_DIFF,
  .brightness = BRIGHTNESS,
  .darkness = DARKNESS,
  .step_diff = STEP_DIFF,
  .reject_diff = REJECT_DIFF,
  .max_ph_diff = MAX_PH_DIFF,
  .max_parallax = MAX_PARALLAX,
  .min_parallax = MIN_PARALLAX,
  .rdup_cnt = RIGHT_DUP_N,
  .ls_points = LS_POINTS,
  .evec_error = EVEC_ERROR,
};

/*
void disparityCallback(const stereo_msgs::DisparityImageConstPtr& msg)
{
  ROS_ERROR("disparityCallback");
  ROS_ERROR("f=%f", msg->f);
  ROS_ERROR("T=%f", msg->T);
  ROS_ERROR("min_disparity=%f", msg->min_disparity);
  ROS_ERROR("max_disparity=%f", msg->max_disparity);
  ROS_ERROR("width=%u", msg->image.width);
  ROS_ERROR("height=%u", msg->image.height);
  ROS_ERROR("step=%u", msg->image.step);
  ROS_ERROR("image.data.size()=%d", msg->image.data.size());
  const float *dispfloatp = (float*)&msg->image.data[0];
  ROS_ERROR("dispfloat=%f", *dispfloatp);

  const cv::Mat_<float> dmat(msg->image.height, msg->image.width, (float*)&msg->image.data[0], msg->image.step);

  float min = 0;
  float max = 0;

  for (int ri = 0; ri < dmat.rows; ri++)
  {
    for (int ci = 0; ci < dmat.cols; ci++)
    {
      if (ri < 2 && ci < 2)
      {
        ROS_ERROR("disp dmat(%d, %d)=%f", ri, ci, dmat(ri, ci));
      }
      if (ri == 0 && ci == 0)
      {
        min = dmat(ri, ci);
        max = dmat(ri, ci);
      }
      else
      {
        if (dmat(ri, ci) < min)
        {
          min = dmat(ri, ci);
        }
        if (dmat(ri, ci) > max)
        {
          max = dmat(ri, ci);
        }
      }
    }
  }

  ROS_ERROR("disparity min=%f, max=%f", min, max);
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_ERROR("depthCallback");
  ROS_ERROR("data.size()=%d", msg->data.size());
  const float *depthfloatp = (float*)&msg->data[0];
  ROS_ERROR("depthfloat=%f", *depthfloatp);

  const cv::Mat_<float> dmat(msg->height, msg->width, (float*)&msg->data[0], msg->step);

  float min = 0;
  float max = 0;

  for (int ri = 0; ri < dmat.rows; ri++)
  {
    for (int ci = 0; ci < dmat.cols; ci++)
    {
      if (ri < 2 && ci < 2)
      {
//        ROS_ERROR("depth dmat(%d, %d)=%f", ri, ci, dmat(ri, ci));
      }
      if (dmat(ri, ci) < image_geometry::StereoCameraModel::MISSING_Z)
      {
        if (ri == 0 && ci == 0)
        {
          min = dmat(ri, ci);
          max = dmat(ri, ci);
        }
        else
        {
          if (dmat(ri, ci) < min)
          {
            min = dmat(ri, ci);
          }
          if (dmat(ri, ci) > max)
          {
            max = dmat(ri, ci);
          }
        }
      }
    }
  }

  ROS_ERROR("depth min=%f, max=%f", min, max);
}
*/

bool reload(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = false;
  res.message = "";
  nh->getParam("pshift_genpc/calc/search_div", param.search_div);
  nh->getParam("pshift_genpc/calc/bw_diff", param.bw_diff);
  nh->getParam("pshift_genpc/calc/brightness", param.brightness);
  nh->getParam("pshift_genpc/calc/darkness", param.darkness);
  nh->getParam("pshift_genpc/calc/step_diff", param.step_diff);
  nh->getParam("pshift_genpc/calc/reject_diff", param.reject_diff);
  nh->getParam("pshift_genpc/calc/max_ph_diff", param.max_ph_diff);
  nh->getParam("pshift_genpc/calc/max_parallax", param.max_parallax);
  nh->getParam("pshift_genpc/calc/min_parallax", param.min_parallax);
  nh->getParam("pshift_genpc/calc/rdup_cnt", param.rdup_cnt);
  nh->getParam("pshift_genpc/calc/ls_points", param.ls_points);
/*
  ROS_ERROR("reload param.search_div=%d", param.search_div);
  ROS_ERROR("reload param.bw_diff=%d", param.bw_diff);
  ROS_ERROR("reload param.brightness=%d", param.brightness);
  ROS_ERROR("reload param.darkness=%d", param.darkness);
  ROS_ERROR("reload param.step_diff=%f", param.step_diff);
  ROS_ERROR("reload param.reject_diff=%f", param.reject_diff);
  ROS_ERROR("reload param.max_ph_diff=%f", param.max_ph_diff);
  ROS_ERROR("reload param.max_parallax=%f", param.max_parallax);
  ROS_ERROR("reload param.min_parallax=%f", param.min_parallax);
  ROS_ERROR("reload param.rdup_cnt=%d", param.rdup_cnt);
  ROS_ERROR("reload param.ls_points=%d", param.ls_points);
*/
  res.success = true;
  res.message = "genpc calc param ready";
  ROS_INFO("genpc:reload ok");
  return true;
}

bool genpc(rovi::GenPC::Request &req, rovi::GenPC::Response &res)
{
  ROS_INFO("genpc called: %d %d", req.imgL.size(), req.imgR.size());

  int width = req.imgL[0].width;
  int height = req.imgL[0].height;

  ROS_INFO("genpc img w, h: %d %d", width, height);

/*
  // Making dummy points
  sensor_msgs::PointCloud pts;
  pts.header.stamp = ros::Time::now();
  pts.header.frame_id = "/map"; // RViz default Frame
  pts.points.resize(10000);
  pts.channels.resize(1);
  pts.channels[0].name = "intensities";
  pts.channels[0].values.resize(10000);
  for (int i = 0; i < 100; i++)
  {
    for (int j = 0; j < 100; j++)
    {
      int k = 100 * i + j;
      pts.points[k].x = 0.02 * i - 1.0;
      pts.points[k].y = 0.02 * j - 1.0;
      pts.points[k].z = 0.002 * (i - 50);
      pts.channels[0].values[k] = 100;
    }
  }
*/

  std::vector<double> vecQ;
  nh->getParam("genpc/Q", vecQ);
  if (vecQ.size() != 16)
  {
    ROS_ERROR("Param Q NG");
    return false;
  }

  ps_init(width, height);
  ROS_INFO("ps_init done");

/*
  ROS_ERROR("param.max_parallax=%f", param.max_parallax);
  ROS_ERROR("param.rdup_cnt=%d", param.rdup_cnt);
  ROS_ERROR("param.ls_points=%d", param.ls_points);
*/

  ps_setparams(param);
  ROS_INFO("ps_setparams done");

  // read Phase Shift data images. (13 left images and 13 right images)
  try
  {
    for (int j = 0; j < 13; j++)
    {
      cv::Mat img = cv_bridge::toCvCopy(req.imgL[j], sensor_msgs::image_encodings::MONO8)->image;
      ps_setpict(0, j, img);
      cv::imwrite(cv::format("/tmp/capt%02d_0.pgm", j), img);
    }
    for (int j = 0; j < 13; j++)
    {
      cv::Mat img = cv_bridge::toCvCopy(req.imgR[j], sensor_msgs::image_encodings::MONO8)->image;
      ps_setpict(1, j, img);
      cv::imwrite(cv::format("/tmp/capt%02d_1.pgm", j), img);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("genpc:cv_bridge:exception: %s", e.what());
    return false;
  }

  // do calc
  ROS_INFO("before ps_exec");
  Eigen::MatrixXd &diff = ps_exec();
  Eigen::Matrix4d Q;
  ROS_INFO("before memcpy");
  memcpy(Q.data(), vecQ.data(), sizeof(double) * 4 * 4);
  ROS_INFO("before genPC");
  int N = genPC(diff, ps.texture, ps.mask[0], ps.pt, Q);
  ROS_INFO("genPC returned N=%d", N);

  // output point clouds
  sensor_msgs::PointCloud pts;
  pts.header.stamp = ros::Time::now();
  pts.header.frame_id = "/map"; // RViz default Frame
  pts.points.resize(N);
  pts.channels.resize(3);
  pts.channels[0].name = "r";
  pts.channels[0].values.resize(N);
  pts.channels[1].name = "g";
  pts.channels[1].values.resize(N);
  pts.channels[2].name = "b";
  pts.channels[2].values.resize(N);
  for (int n = 0; n < N; n++)
  {
    pts.points[n].x = _pcd[n].coord[0];
    pts.points[n].y = _pcd[n].coord[1];
    pts.points[n].z = _pcd[n].coord[2];
    pts.channels[0].values[n] = _pcd[n].col[0] / 255.0;
    pts.channels[1].values[n] = _pcd[n].col[1] / 255.0;
    pts.channels[2].values[n] = _pcd[n].col[2] / 255.0;
    if (n < 20 || (N - 20) < n)
    {
      ROS_INFO("n=%d x,y,z=%f,%f,%f r,g,b=%f,%f,%f",
        n,
        pts.points[n].x,
        pts.points[n].y,
        pts.points[n].z,
        pts.channels[0].values[n],
        pts.channels[1].values[n],
        pts.channels[2].values[n]
      );
    }
  }

  ROS_INFO("before outPLY");
  outPLY("/tmp/test.ply");
  ROS_INFO("after  outPLY");

//  pub1->publish(pts);
  res.pc = pts;

  sensor_msgs::PointCloud2 pts2;
  sensor_msgs::convertPointCloudToPointCloud2(pts, pts2);
//  ROS_INFO("genpc::do %d %d %d\n", pts2.width, pts2.height, pts2.point_step);
  pts2.row_step = pts2.width * pts2.point_step;
//  pub2->publish(pts2);
  res.pc2 = pts2;

  ROS_INFO("now return");
  return true;
}

bool trypc(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  rovi::GenPC srv;
  return genpc(srv.request, srv.response);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "genpc_node");
  ros::NodeHandle n;
  nh = &n;
//  ros::Subscriber sub_disp = n.subscribe("disparity", 1, disparityCallback);
//  ros::Subscriber sub_depth = n.subscribe("depth", 1, depthCallback);
  ros::ServiceServer svc0 = n.advertiseService("genpc/reload", reload);
  ros::ServiceServer svc1 = n.advertiseService("genpc", genpc);
  ros::ServiceServer svc2 = n.advertiseService("genpc/try", trypc);
//  ros::Publisher p1 = n.advertise<sensor_msgs::PointCloud>("genpc/pcl", 1);
//  pub1 = &p1;
//  ros::Publisher p2 = n.advertise<sensor_msgs::PointCloud2>("genpc/pcl2", 1);
//  pub2 = &p2;
  std_srvs::Trigger::Request req;
  std_srvs::Trigger::Response res;
  reload(req, res);
  if (res.success)
  {
    ros::spin();
  }
  else
  {
    ROS_ERROR("genpc:unmatched parameters");
  }
  return 0;
}
