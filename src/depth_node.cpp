#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/Image.h>
#include <image_geometry/stereo_camera_model.h>
#include <opencv2/opencv.hpp>

ros::NodeHandle *nh;
ros::Publisher pub_depth;

double cx_l = 0;
double cx_r = 0;

bool reload(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = false;
  res.message = "";

  std::vector<double> lP;
  std::vector<double> rP;
  nh->getParam("left/remap/P", lP);
  nh->getParam("right/remap/P", rP);
  if (lP.size() != 12)
  {
    ROS_ERROR("Param left P NG");
    res.message += "left P NG/";
  }
  if (rP.size() != 12)
  {
    ROS_ERROR("Param right P NG");
    res.message += "right P NG/";
  }

  if (res.message.size() > 0) // Error happened
  {
    return true;
  }

  cx_l = lP[2];
  cx_r = rP[2];
//  ROS_ERROR("cx_l=%f, cx_r=%f", cx_l, cx_r);

  res.success = true;
  res.message = "depth param ready";
  ROS_INFO("depth:reload ok");

  return true;
}

void outputDepth(const stereo_msgs::DisparityImageConstPtr& disp_msg)
{
/*
  ROS_ERROR("got disp_msg...");
  ROS_ERROR("f=%f", disp_msg->f);
  ROS_ERROR("T=%f", disp_msg->T);
  ROS_ERROR("min_disparity=%f", disp_msg->min_disparity);
  ROS_ERROR("max_disparity=%f", disp_msg->max_disparity);
  ROS_ERROR("width=%u", disp_msg->image.width);
  ROS_ERROR("height=%u", disp_msg->image.height);
  ROS_ERROR("step=%u", disp_msg->image.step);
  ROS_ERROR("image.data.size()=%d", disp_msg->image.data.size());
  const float *dispfloatp = (float*)&disp_msg->image.data[0];
  ROS_ERROR("dispfloat=%f", *dispfloatp);
  const cv::Mat_<float> dmat(disp_msg->image.height, disp_msg->image.width, (float*)&disp_msg->image.data[0], disp_msg->image.step);
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
*/

  const sensor_msgs::Image& disp_image = disp_msg->image;
  const cv::Mat_<float> disp_mat(disp_image.height, disp_image.width, (float*)&disp_image.data[0], disp_image.step);

  //// output Depth
  sensor_msgs::ImagePtr depth_image = boost::make_shared<sensor_msgs::Image>();

  depth_image->header = disp_msg->header;
  depth_image->width = disp_image.width;
  depth_image->height = disp_image.height;
  depth_image->encoding = disp_image.encoding;
  depth_image->is_bigendian = disp_image.is_bigendian;
  depth_image->step = disp_image.step;
  depth_image->data.resize(depth_image->step * depth_image->height);

  cv::Mat_<float> depth_mat(depth_image->height, depth_image->width, (float*)&depth_image->data[0], depth_image->step);

  for (int v = 0; v < depth_mat.rows; ++v)
  {
    for (int u = 0; u < depth_mat.cols; ++u)
   {
      const float& disp = disp_mat(v, u);
      if (disp > 0 && !std::isinf(disp))
      {
        depth_mat(v, u) = disp_msg->f * disp_msg->T / (disp - (cx_l - cx_r));
//        ROS_ERROR("disp=%f, depth=%f", disp, depth_mat(v, u));
      }
      else
      {
        depth_mat(v, u) = image_geometry::StereoCameraModel::MISSING_Z;
      }
    }
  }

  pub_depth.publish(depth_image);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_node");
  ros::NodeHandle n;
  nh = &n;
  pub_depth = n.advertise<sensor_msgs::Image>("depth", 1);
  ros::Subscriber sub_disp = n.subscribe("disparity", 1, outputDepth);
  ros::ServiceServer svc0 = n.advertiseService("depth/reload", reload);
  ros::spin();
  return 0;
}
