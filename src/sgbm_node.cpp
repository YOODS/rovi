#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <opencv2/opencv.hpp>
#include <image_geometry/stereo_camera_model.h>
#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif
#include <cv_bridge/cv_bridge.h>
#include <chrono>


#define TM_DEBUG false 

ros::NodeHandle *nh;
ros::Publisher pub_disp;
ros::Publisher pub_depth;
ros::Publisher pub_pcl2;

int seq_lrect = -1;
int seq_rrect = -1;
int seq_linfo = -1;
int seq_rinfo = -1;

cv::Mat_<uint8_t> l_image;
cv::Mat_<uint8_t> r_image;
sensor_msgs::CameraInfo linfo;
sensor_msgs::CameraInfo rinfo;

cv::Ptr<cv::StereoSGBM> ssgbm;

int minDisparity = 0;
int numDisparities = 64;
int blockSize = 3; // SADWindowSize
int P1 = 8 * 3 * blockSize * blockSize;
int P2 = 32 * 3 * blockSize * blockSize;
int disp12MaxDiff = 0;
int preFilterCap = 31;
int uniquenessRatio = 15;
int speckleWindowSize = 100;
int speckleRange = 2;

inline bool isValidPoint(const cv::Vec3f& pt)
{
  return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
}

void makeSgbm()
{
	ssgbm = cv::StereoSGBM::create(
		minDisparity,
		numDisparities,
		blockSize,
		P1,
		P2,
		disp12MaxDiff,
		preFilterCap,
		uniquenessRatio,
		speckleWindowSize,
		speckleRange,
		cv::StereoSGBM::MODE_HH4);
}

void paramUpdate()
{
	int prev_minDisparity = minDisparity;
	int prev_numDisparities = numDisparities;
	int prev_blockSize = blockSize;
	int prev_disp12MaxDiff = disp12MaxDiff;
	int prev_preFilterCap = preFilterCap;
	int prev_uniquenessRatio = uniquenessRatio;
	int prev_speckleWindowSize = speckleWindowSize;
	int prev_speckleRange = speckleRange;

#if TM_DEBUG
	auto tm1 = std::chrono::system_clock::now();
#endif

	nh->getParam("sgbm/minDisparity", minDisparity);
	nh->getParam("sgbm/numDisparities", numDisparities);
	nh->getParam("sgbm/blockSize", blockSize);
	nh->getParam("sgbm/disp12MaxDiff", disp12MaxDiff);
	nh->getParam("sgbm/preFilterCap", preFilterCap);
	nh->getParam("sgbm/uniquenessRatio", uniquenessRatio);
	nh->getParam("sgbm/speckleWindowSize", speckleWindowSize);
	nh->getParam("sgbm/speckleRange", speckleRange);

	if (numDisparities < 16) {
		ROS_WARN("numDisparities(%d) is set to 16", numDisparities);
		numDisparities = 16;
		nh->setParam("sgbm/numDisparities", numDisparities);
	}
	int ndred = numDisparities % 16;
	if (ndred != 0) {
		numDisparities -= ndred;
		ROS_WARN("ndred=%d, now numDisparities=%d", ndred, numDisparities);
		nh->setParam("sgbm/numDisparities", numDisparities);
	}

	if (blockSize < 1) {
		ROS_WARN("blockSize(%d) is set to 1", blockSize);
		blockSize = 1;
		nh->setParam("sgbm/blockSize", blockSize);
	}
	if ((blockSize % 2) == 0) {
		blockSize--;
		ROS_ERROR("blockSize--, now blockSize=%d", blockSize);
		nh->setParam("sgbm/blockSize", blockSize);
	}

	if (uniquenessRatio < 0) {
		ROS_WARN("uniquenessRatio(%d) is set to 0", uniquenessRatio);
		uniquenessRatio = 0;
		nh->setParam("sgbm/uniquenessRatio", uniquenessRatio);
	}
	else if (uniquenessRatio > 100) {
		ROS_WARN("uniquenessRatio(%d) is set to 100", uniquenessRatio);
		uniquenessRatio = 100;
		nh->setParam("sgbm/uniquenessRatio", uniquenessRatio);
	}

	if (speckleWindowSize < 0) {
		ROS_WARN("speckleWindowSize(%d) is set to 0", speckleWindowSize);
		speckleWindowSize = 0;
		nh->setParam("sgbm/speckleWindowSize", speckleWindowSize);
	}

	if (speckleRange < 0) {
		ROS_WARN("speckleRange(%d) is set to 0", speckleRange);
		speckleRange = 0;
		nh->setParam("sgbm/speckleRange", speckleRange);
	}


	if (prev_minDisparity != minDisparity) {
		ROS_INFO("paramUpdate now minDisparity=%d", minDisparity);
	}
	if (prev_numDisparities != numDisparities) {
		ROS_INFO("paramUpdate now numDisparities=%d", numDisparities);
	}
	if (prev_blockSize != blockSize) {
		ROS_INFO("paramUpdate now blockSize=%d", blockSize);
	}
	if (prev_disp12MaxDiff != disp12MaxDiff) {
		ROS_INFO("paramUpdate now disp12MaxDiff=%d", disp12MaxDiff);
	}
	if (prev_preFilterCap != preFilterCap) {
		ROS_INFO("paramUpdate now preFilterCap=%d", preFilterCap);
	}
	if (prev_uniquenessRatio != uniquenessRatio) {
		ROS_INFO("paramUpdate now uniquenessRatio=%d", uniquenessRatio);
	}
	if (prev_speckleWindowSize != speckleWindowSize) {
		ROS_INFO("paramUpdate now speckleWindowSize=%d", speckleWindowSize);
	}
	if (prev_speckleRange != speckleRange) {
		ROS_INFO("paramUpdate now speckleRange=%d", speckleRange);
	}

#if TM_DEBUG
	auto tm2 = std::chrono::system_clock::now();
	auto getParam_elapsed_msec = std::chrono::duration_cast<std::chrono::milliseconds>(tm2 - tm1);
	ROS_ERROR("paramUpdate getParam time msec=%d", getParam_elapsed_msec.count());
#endif

	ssgbm->setMinDisparity(minDisparity);
	ssgbm->setNumDisparities(numDisparities);
	ssgbm->setBlockSize(blockSize);
	ssgbm->setDisp12MaxDiff(disp12MaxDiff);
	ssgbm->setPreFilterCap(preFilterCap);
	ssgbm->setUniquenessRatio(uniquenessRatio);
	ssgbm->setSpeckleWindowSize(speckleWindowSize);
	ssgbm->setSpeckleRange(speckleRange);

#if TM_DEBUG
	auto tm3 = std::chrono::system_clock::now();
	auto set_elapsed_msec = std::chrono::duration_cast<std::chrono::milliseconds>(tm3 - tm2);
	ROS_ERROR("paramUpdate set time msec=%d", set_elapsed_msec.count());
#endif
}

void outputDisparityDepthPcl2()
{
//	paramUpdate();

	image_geometry::StereoCameraModel model_;
	model_.fromCameraInfo(linfo, rinfo);


	//// output Disparity
	stereo_msgs::DisparityImagePtr disp_msg = boost::make_shared<stereo_msgs::DisparityImage>();

	disp_msg->header         = linfo.header;
	disp_msg->image.header   = linfo.header;

	int border = ssgbm->getBlockSize() / 2;

	int left   = ssgbm->getNumDisparities() + ssgbm->getMinDisparity() + border - 1;
	int wtf   = (ssgbm->getMinDisparity() >= 0) ? border + ssgbm->getMinDisparity() : std::max(border, -ssgbm->getMinDisparity());
	int right  = disp_msg->image.width - 1 - wtf;
	int top    = border;
	int bottom = disp_msg->image.height - 1 - border;

	disp_msg->valid_window.x_offset = left;
	disp_msg->valid_window.y_offset = top;
	disp_msg->valid_window.width    = right - left;
	disp_msg->valid_window.height   = bottom - top;

	cv::Mat_<int16_t> disparity16_;
	ssgbm->compute(l_image, r_image, disparity16_);

	static const int DPP = 16; // disparities per pixel
	static const double inv_dpp = 1.0 / DPP;

	sensor_msgs::Image& disp_image = disp_msg->image;
	disp_image.height = disparity16_.rows;
	disp_image.width = disparity16_.cols;
	disp_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
	disp_image.step = disp_image.width * sizeof(float);
	disp_image.data.resize(disp_image.step * disp_image.height);

	cv::Mat_<float> disp_mat(disp_image.height, disp_image.width, (float*)&disp_image.data[0], disp_image.step);
	disparity16_.convertTo(disp_mat, disp_mat.type(), inv_dpp);

	disp_msg->f = model_.right().fx();
	disp_msg->T = model_.baseline();

	disp_msg->min_disparity = minDisparity;
	disp_msg->max_disparity = minDisparity + numDisparities - 1;
	disp_msg->delta_d = inv_dpp;

	pub_disp.publish(disp_msg);


	//
	cv::Mat_<cv::Vec3f> point_mat;
	model_.projectDisparityImageTo3d(disp_mat, point_mat, true);

	float nan = std::numeric_limits<float>::quiet_NaN();
	float inf = std::numeric_limits<float>::infinity();


	//// output Depth
	sensor_msgs::ImagePtr depth_image = boost::make_shared<sensor_msgs::Image>();

	depth_image->header = disp_msg->header;
	depth_image->width = disp_image.width;
	depth_image->height = disp_image.height;
	depth_image->encoding = disp_image.encoding;
	depth_image->is_bigendian = disp_image.is_bigendian;
	depth_image->step = disp_image.step;
	depth_image->data.resize(depth_image->step * depth_image->height);

	size_t num_pixels = depth_image->width * depth_image->height;

	std::vector<uint8_t>& data = depth_image->data;

	for (size_t i = 0; i < num_pixels; ++i){
		if (isValidPoint(point_mat(i))) {
			memcpy(&data[i * sizeof(float)], &point_mat(i)[2], sizeof(float));
//			ROS_ERROR("depth[%d]=%f", i, point_mat(i)[2]);
		}
		else {
			memcpy(&data[i * sizeof(float)], &inf, sizeof(float));
		}
	}

/*
	float txf = model_.baseline() * model_.right().fx();
	double cx_l = model_.left().cx();
	double cx_r = model_.right().cx();
//	ROS_ERROR("txf=%f, cx_l=%f, cx_r=%f", txf, cx_l, cx_r);

	cv::Mat_<float> depth_mat(depth_image->height, depth_image->width, (float*)&depth_image->data[0], depth_image->step);

	for (int v = 0; v < depth_mat.rows; ++v) {
		for (int u = 0; u < depth_mat.cols; ++u) {
			float& disp = disp_mat(v, u); 
			if (disp > 0 && !std::isinf(disp)) {
				depth_mat(v, u) = txf / (disp - (cx_l - cx_r));
//				ROS_ERROR("disp=%f, depth=%f", disp, depth_mat(v, u));
			}
			else {
				depth_mat(v, u) = image_geometry::StereoCameraModel::MISSING_Z;
			}
		}
	}
*/

	pub_depth.publish(depth_image);


	//// output PointCloud2
	sensor_msgs::PointCloud2Ptr pcl2_msg = boost::make_shared<sensor_msgs::PointCloud2>();
	pcl2_msg->header = disp_msg->header;
	pcl2_msg->height = point_mat.rows;
	pcl2_msg->width  = point_mat.cols;
	pcl2_msg->is_bigendian = disp_image.is_bigendian;
	pcl2_msg->is_dense = false;

	sensor_msgs::PointCloud2Modifier pcl2_modifier(*pcl2_msg);
	pcl2_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

	sensor_msgs::PointCloud2Iterator<float> iter_x(*pcl2_msg, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(*pcl2_msg, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(*pcl2_msg, "z");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*pcl2_msg, "r");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*pcl2_msg, "g");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*pcl2_msg, "b");

	for (int v = 0; v < point_mat.rows; ++v) {
		for (int u = 0; u < point_mat.cols; ++u, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
			if (isValidPoint(point_mat(v,u))) {
				*iter_x = point_mat(v, u)[0];
				*iter_y = point_mat(v, u)[1];
				*iter_z = point_mat(v, u)[2];
			}
			else {
				*iter_x = *iter_y = *iter_z = nan;
			}
			*iter_r = *iter_g = *iter_b = l_image(v,u);
		}
	}

	pub_pcl2.publish(pcl2_msg);
}

void lrectCallback(const sensor_msgs::ImageConstPtr &msg)
{
	seq_lrect = msg->header.seq;
//	ROS_ERROR("lrectCallback. now seq_lrect=%d", seq_lrect);

	l_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;

	if ((seq_lrect == seq_rrect) && (seq_lrect == seq_linfo) && (seq_lrect == seq_rinfo)) {
		outputDisparityDepthPcl2();
	}
}

void rrectCallback(const sensor_msgs::ImageConstPtr &msg)
{
	seq_rrect = msg->header.seq;
//	ROS_ERROR("rrectCallback. now seq_rrect=%d", seq_rrect);

	r_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;

	if ((seq_rrect == seq_lrect) && (seq_rrect == seq_linfo) && (seq_rrect == seq_rinfo)) {
		outputDisparityDepthPcl2();
	}
}

void linfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
	seq_linfo = msg->header.seq;
//	ROS_ERROR("linfoCallback. now seq_linfo=%d", seq_linfo);

	linfo = *msg;

	if ((seq_linfo == seq_lrect) && (seq_linfo == seq_rrect) && (seq_linfo == seq_rinfo)) {
		outputDisparityDepthPcl2();
	}
}

void rinfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
	seq_rinfo = msg->header.seq;
//	ROS_ERROR("rinfoCallback. now seq_rinfo=%d", seq_rinfo);

	rinfo = *msg;

	if ((seq_rinfo == seq_lrect) && (seq_rinfo == seq_rrect) && (seq_rinfo == seq_linfo)) {
		outputDisparityDepthPcl2();
	}
}

bool reload(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res)
{
	res.success=false;
	res.message="";
	paramUpdate();
/*
	ROS_ERROR("reload minDisparity=%d", minDisparity);
	ROS_ERROR("reload numDisparities=%d", numDisparities);
	ROS_ERROR("reload blockSize=%d", blockSize);
	ROS_ERROR("reload disp12MaxDiff=%d", disp12MaxDiff);
	ROS_ERROR("reload preFilterCap=%d", preFilterCap);
	ROS_ERROR("reload uniquenessRatio=%d", uniquenessRatio);
	ROS_ERROR("reload speckleWindowSize=%d", speckleWindowSize);
	ROS_ERROR("reload speckleRange=%d", speckleRange);
*/
	res.success=true;
	res.message="sgbm param ready";
	ROS_INFO("sgbm:reload ok");
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sgbm_node");
	ros::NodeHandle n;
	nh = &n;
	makeSgbm();
	pub_disp = n.advertise<stereo_msgs::DisparityImage>("disparity", 1);
	pub_depth = n.advertise<sensor_msgs::Image>("depth", 1);
	pub_pcl2 = n.advertise<sensor_msgs::PointCloud2>("pcl2", 1);
	ros::Subscriber sub_lr = n.subscribe("left/image_rect", 1, lrectCallback);
	ros::Subscriber sub_rr = n.subscribe("right/image_rect", 1, rrectCallback);
	ros::Subscriber sub_li = n.subscribe("left/camera_info", 1, linfoCallback);
	ros::Subscriber sub_ri = n.subscribe("right/camera_info", 1, rinfoCallback);
	ros::ServiceServer svc0=n.advertiseService("sgbm/reload",reload);
	std_srvs::Trigger::Request req;
	std_srvs::Trigger::Response res;
	reload(req,res);
	if(res.success) ros::spin();
	else ROS_ERROR("sgbm:unmatched parameters");
	return 0;
}
