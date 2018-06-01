#include <ros/ros.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <image_geometry/stereo_camera_model.h>
#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif
#include <cv_bridge/cv_bridge.h>
#include <chrono>

#define TM_DEBUG false 

using namespace sensor_msgs;
using namespace stereo_msgs;

ros::NodeHandle *nh;
ros::Publisher pub_dsp;

int seq_lrect = -1;
int seq_rrect = -1;
int seq_linfo = -1;
int seq_rinfo = -1;

cv::Mat_<uint8_t> l_image;
cv::Mat_<uint8_t> r_image;
CameraInfo linfo;
CameraInfo rinfo;

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
		ROS_ERROR("numDisparities(%d) is set to 16", numDisparities);
		numDisparities = 16;
		nh->setParam("sgbm/numDisparities", numDisparities);
	}
	int ndred = numDisparities % 16;
	if (ndred != 0) {
		numDisparities -= ndred;
		ROS_ERROR("ndred=%d, now numDisparities=%d", ndred, numDisparities);
		nh->setParam("sgbm/numDisparities", numDisparities);
	}

	if (blockSize < 1) {
		ROS_ERROR("blockSize(%d) is set to 1", blockSize);
		blockSize = 1;
		nh->setParam("sgbm/blockSize", blockSize);
	}
	if ((blockSize % 2) == 0) {
		blockSize--;
		ROS_ERROR("blockSize--, now blockSize=%d", blockSize);
		nh->setParam("sgbm/blockSize", blockSize);
	}

	if (uniquenessRatio < 0) {
		ROS_ERROR("uniquenessRatio(%d) is set to 0", uniquenessRatio);
		uniquenessRatio = 0;
		nh->setParam("sgbm/uniquenessRatio", uniquenessRatio);
	}
	else if (uniquenessRatio > 100) {
		ROS_ERROR("uniquenessRatio(%d) is set to 100", uniquenessRatio);
		uniquenessRatio = 100;
		nh->setParam("sgbm/uniquenessRatio", uniquenessRatio);
	}

	if (speckleWindowSize < 0) {
		ROS_ERROR("speckleWindowSize(%d) is set to 0", speckleWindowSize);
		speckleWindowSize = 0;
		nh->setParam("sgbm/speckleWindowSize", speckleWindowSize);
	}

	if (speckleRange < 0) {
		ROS_ERROR("speckleRange(%d) is set to 0", speckleRange);
		speckleRange = 0;
		nh->setParam("sgbm/speckleRange", speckleRange);
	}


	if (prev_minDisparity != minDisparity) {
		ROS_ERROR("paramUpdate now minDisparity=%d", minDisparity);
	}
	if (prev_numDisparities != numDisparities) {
		ROS_ERROR("paramUpdate now numDisparities=%d", numDisparities);
	}
	if (prev_blockSize != blockSize) {
		ROS_ERROR("paramUpdate now blockSize=%d", blockSize);
	}
	if (prev_disp12MaxDiff != disp12MaxDiff) {
		ROS_ERROR("paramUpdate now disp12MaxDiff=%d", disp12MaxDiff);
	}
	if (prev_preFilterCap != preFilterCap) {
		ROS_ERROR("paramUpdate now preFilterCap=%d", preFilterCap);
	}
	if (prev_uniquenessRatio != uniquenessRatio) {
		ROS_ERROR("paramUpdate now uniquenessRatio=%d", uniquenessRatio);
	}
	if (prev_speckleWindowSize != speckleWindowSize) {
		ROS_ERROR("paramUpdate now speckleWindowSize=%d", speckleWindowSize);
	}
	if (prev_speckleRange != speckleRange) {
		ROS_ERROR("paramUpdate now speckleRange=%d", speckleRange);
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

void outputDisparity()
{
	paramUpdate();

	image_geometry::StereoCameraModel model_;
	model_.fromCameraInfo(linfo, rinfo);

	DisparityImagePtr disp_msg = boost::make_shared<DisparityImage>();

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

	sensor_msgs::Image& dimage = disp_msg->image;
	dimage.height = disparity16_.rows;
	dimage.width = disparity16_.cols;
	dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
	dimage.step = dimage.width * sizeof(float);
	dimage.data.resize(dimage.step * dimage.height);

	cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
	disparity16_.convertTo(dmat, dmat.type(), inv_dpp);

	disp_msg->f = model_.right().fx();
	disp_msg->T = model_.baseline();

	disp_msg->min_disparity = minDisparity;
	disp_msg->max_disparity = minDisparity + numDisparities - 1;
	disp_msg->delta_d = inv_dpp;

	pub_dsp.publish(disp_msg);
}

void lrectCallback(const ImageConstPtr &msg)
{
	seq_lrect = msg->header.seq;
//	ROS_ERROR("lrectCallback. now seq_lrect=%d", seq_lrect);

	l_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;

	if ((seq_lrect == seq_rrect) && (seq_lrect == seq_linfo) && (seq_lrect == seq_rinfo)) {
		outputDisparity();
	}
}

void rrectCallback(const ImageConstPtr &msg)
{
	seq_rrect = msg->header.seq;
//	ROS_ERROR("rrectCallback. now seq_rrect=%d", seq_rrect);

	r_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;

	if ((seq_rrect == seq_lrect) && (seq_rrect == seq_linfo) && (seq_rrect == seq_rinfo)) {
		outputDisparity();
	}
}

void linfoCallback(const CameraInfoConstPtr &msg)
{
	seq_linfo = msg->header.seq;
//	ROS_ERROR("linfoCallback. now seq_linfo=%d", seq_linfo);

	linfo = *msg;

	if ((seq_linfo == seq_lrect) && (seq_linfo == seq_rrect) && (seq_linfo == seq_rinfo)) {
		outputDisparity();
	}
}

void rinfoCallback(const CameraInfoConstPtr &msg)
{
	seq_rinfo = msg->header.seq;
//	ROS_ERROR("rinfoCallback. now seq_rinfo=%d", seq_rinfo);

	rinfo = *msg;

	if ((seq_rinfo == seq_lrect) && (seq_rinfo == seq_rrect) && (seq_rinfo == seq_linfo)) {
		outputDisparity();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "disparity_node");
	ros::NodeHandle n;
	nh = &n;
	makeSgbm();
	pub_dsp = n.advertise<stereo_msgs::DisparityImage>("disparity", 1);
	ros::Subscriber sub_lr = n.subscribe("left/image_rect", 1, lrectCallback);
	ros::Subscriber sub_rr = n.subscribe("right/image_rect", 1, rrectCallback);
	ros::Subscriber sub_li = n.subscribe("left/camera_info", 1, linfoCallback);
	ros::Subscriber sub_ri = n.subscribe("right/camera_info", 1, rinfoCallback);
	ros::spin();
	return 0;
}
