#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/Image.h>
#include <image_geometry/stereo_camera_model.h>
#include <opencv2/opencv.hpp>

ros::NodeHandle *nh;
ros::Publisher pub_depth;

void outputDepth(const stereo_msgs::DisparityImageConstPtr& disp_msg)
{
//  paramUpdate();

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
        //depth_mat(v, u) = txf / (disp - (cx_l - cx_r));
        depth_mat(v, u) = disp_msg->f * disp_msg->T / (disp + 499.796617); // TODO
        ROS_ERROR("disp=%f, depth=%f", disp, depth_mat(v, u));
      }
      else
      {
        depth_mat(v, u) = image_geometry::StereoCameraModel::MISSING_Z;
      }
    }
  }

  pub_depth.publish(depth_image);
}

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

  outputDepth(msg);
}

/*
int windowSize = 1;
int minDisparity = 0;
int numDisparities = 64;
int blockSize = 3; // SADWindowSize
int P1 = 8 * 3 * windowSize * windowSize;
int P2 = 32 * 3 * windowSize * windowSize;
int disp12MaxDiff = 0;
int preFilterCap = 31;
int uniquenessRatio = 15;
int speckleWindowSize = 100;
int speckleRange = 2;

void paramUpdate()
{
  int prev_windowSize = windowSize;
  int prev_minDisparity = minDisparity;
  int prev_numDisparities = numDisparities;
  int prev_blockSize = blockSize;
  int prev_disp12MaxDiff = disp12MaxDiff;
  int prev_preFilterCap = preFilterCap;
  int prev_uniquenessRatio = uniquenessRatio;
  int prev_speckleWindowSize = speckleWindowSize;
  int prev_speckleRange = speckleRange;

  nh->getParam("sgbm/windowSize", windowSize);
  nh->getParam("sgbm/minDisparity", minDisparity);
  nh->getParam("sgbm/numDisparities", numDisparities);
  nh->getParam("sgbm/blockSize", blockSize);
  nh->getParam("sgbm/disp12MaxDiff", disp12MaxDiff);
  nh->getParam("sgbm/preFilterCap", preFilterCap);
  nh->getParam("sgbm/uniquenessRatio", uniquenessRatio);
  nh->getParam("sgbm/speckleWindowSize", speckleWindowSize);
  nh->getParam("sgbm/speckleRange", speckleRange);

  if (numDisparities < 16)
  {
    ROS_WARN("numDisparities(%d) is set to 16", numDisparities);
    numDisparities = 16;
    nh->setParam("sgbm/numDisparities", numDisparities);
  }
  int ndred = numDisparities % 16;
  if (ndred != 0)
  {
    numDisparities -= ndred;
    ROS_WARN("ndred=%d, now numDisparities=%d", ndred, numDisparities);
    nh->setParam("sgbm/numDisparities", numDisparities);
  }

  if (windowSize < 1)
  {
    ROS_WARN("windowSize(%d) is set to 1", windowSize);
    windowSize = 1;
    nh->setParam("sgbm/windowSize", windowSize);
  }
  if ((windowSize % 2) == 0)
  {
    windowSize--;
    ROS_ERROR("windowSize--, now windowSize=%d", windowSize);
    nh->setParam("sgbm/windowSize", windowSize);
  }

  if (blockSize < 1)
  {
    ROS_WARN("blockSize(%d) is set to 1", blockSize);
    blockSize = 1;
    nh->setParam("sgbm/blockSize", blockSize);
  }
  if ((blockSize % 2) == 0)
  {
    blockSize--;
    ROS_ERROR("blockSize--, now blockSize=%d", blockSize);
    nh->setParam("sgbm/blockSize", blockSize);
  }

  if (uniquenessRatio < 0)
  {
    ROS_WARN("uniquenessRatio(%d) is set to 0", uniquenessRatio);
    uniquenessRatio = 0;
    nh->setParam("sgbm/uniquenessRatio", uniquenessRatio);
  }
  else if (uniquenessRatio > 100)
  {
    ROS_WARN("uniquenessRatio(%d) is set to 100", uniquenessRatio);
    uniquenessRatio = 100;
    nh->setParam("sgbm/uniquenessRatio", uniquenessRatio);
  }

  if (speckleWindowSize < 0)
  {
    ROS_WARN("speckleWindowSize(%d) is set to 0", speckleWindowSize);
    speckleWindowSize = 0;
    nh->setParam("sgbm/speckleWindowSize", speckleWindowSize);
  }

  if (speckleRange < 0)
  {
    ROS_WARN("speckleRange(%d) is set to 0", speckleRange);
    speckleRange = 0;
    nh->setParam("sgbm/speckleRange", speckleRange);
  }


  if (prev_windowSize != windowSize)
  {
    ROS_INFO("paramUpdate now windowSize=%d", windowSize);
  }
  if (prev_minDisparity != minDisparity)
  {
    ROS_INFO("paramUpdate now minDisparity=%d", minDisparity);
  }
  if (prev_numDisparities != numDisparities)
  {
    ROS_INFO("paramUpdate now numDisparities=%d", numDisparities);
  }
  if (prev_blockSize != blockSize)
  {
    ROS_INFO("paramUpdate now blockSize=%d", blockSize);
  }
  if (prev_disp12MaxDiff != disp12MaxDiff)
  {
    ROS_INFO("paramUpdate now disp12MaxDiff=%d", disp12MaxDiff);
  }
  if (prev_preFilterCap != preFilterCap)
  {
    ROS_INFO("paramUpdate now preFilterCap=%d", preFilterCap);
  }
  if (prev_uniquenessRatio != uniquenessRatio)
  {
    ROS_INFO("paramUpdate now uniquenessRatio=%d", uniquenessRatio);
  }
  if (prev_speckleWindowSize != speckleWindowSize)
  {
    ROS_INFO("paramUpdate now speckleWindowSize=%d", speckleWindowSize);
  }
  if (prev_speckleRange != speckleRange)
  {
    ROS_INFO("paramUpdate now speckleRange=%d", speckleRange);
  }

  ssgbm->setMinDisparity(minDisparity);
  ssgbm->setNumDisparities(numDisparities);
  ssgbm->setBlockSize(blockSize);
  ssgbm->setDisp12MaxDiff(disp12MaxDiff);
  ssgbm->setPreFilterCap(preFilterCap);
  ssgbm->setUniquenessRatio(uniquenessRatio);
  ssgbm->setSpeckleWindowSize(speckleWindowSize);
  ssgbm->setSpeckleRange(speckleRange);
  P1 = 8 * 3 * windowSize * windowSize;
  P2 = 32 * 3 * windowSize * windowSize;
  ssgbm->setP1(P1);
  ssgbm->setP2(P2);
}
*/

/*
bool reload(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = false;
  res.message = "";
  paramUpdate();

  ROS_ERROR("reload minDisparity=%d", minDisparity);
  ROS_ERROR("reload numDisparities=%d", numDisparities);
  ROS_ERROR("reload blockSize=%d", blockSize);
  ROS_ERROR("reload disp12MaxDiff=%d", disp12MaxDiff);
  ROS_ERROR("reload preFilterCap=%d", preFilterCap);
  ROS_ERROR("reload uniquenessRatio=%d", uniquenessRatio);
  ROS_ERROR("reload speckleWindowSize=%d", speckleWindowSize);
  ROS_ERROR("reload speckleRange=%d", speckleRange);

  res.success = true;
  res.message = "depth param ready";
  ROS_INFO("depth:reload ok");
  return true;
}
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_node");
  ros::NodeHandle n;
  nh = &n;
  pub_depth = n.advertise<sensor_msgs::Image>("depth", 1);
  ros::Subscriber sub_disp = n.subscribe("disparity", 1, disparityCallback);
/*
  ros::ServiceServer svc0 = n.advertiseService("depth/reload", reload);
  std_srvs::Trigger::Request req;
  std_srvs::Trigger::Response res;
  reload(req, res);
  if (res.success)
  {
*/
    ros::spin();
/*
  }
  else
  {
    ROS_ERROR("depth:unmatched parameters");
  }
*/
  return 0;
}
