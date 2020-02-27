#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "rovi/ImageFilter.h"
#include "rovi/Floats.h"
#include "iCalibBoardRecognizer.hpp"
#include <stdlib.h>


iCalibBoardRecognizer *cboard;


ros::NodeHandle *nh;
std::string paramK("gridboard/K");
static std::vector<double> kvec;
static ros::Publisher *pub1, *pub2, *pub3, *pub4, *pub5;
static double torelance=1.0;
static tf2_ros::TransformBroadcaster *broadcaster;

void solve(sensor_msgs::Image src)
{
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
	int stat = cboard->recognize(cv_ptr1->image, imagePoints);

	// Successでなくても結果画像が出て来るようになったので状態に拘わらず表示
	cv::Mat mat(cv_ptr1->image.size(), CV_8UC3);  
	sensor_msgs::Image img;
	cboard->copy_result_image(mat,stat);
	cv_ptr1->image=mat;
	cv_ptr1->encoding="bgr8";
	cv_ptr1->toImageMsg(img);
	pub1->publish(img);

	
	if (stat != 0) {
		switch (stat) {
		case 1:
			ROS_WARN("CalibBoard::scan imagesize changed"); break;
		case 2:
			ROS_WARN("CalibBoard::scan marker contour not found"); break;
		default:
			ROS_WARN("CalibBoard::scan base marker not found"); break;
		}
		pub4->publish(done);
		return;	  
	}
    	
	
	rovi::Floats buf;
	buf.data.resize(5 * imagePoints.size());
	for (int n = 0, i = 0; n < imagePoints.size(); n++, i += 5) {
		cv::Point3f markerpos = cboard->get_3d_position(n);
		cv::Point2f imagepos = imagePoints[n];
		
		buf.data[i + 0] = markerpos.x;
		buf.data[i + 1] = markerpos.y;
		buf.data[i + 2] = markerpos.z;
		buf.data[i + 3] = imagepos.x;
		buf.data[i + 4] = imagepos.y;
	}
	pub3->publish(buf);

	
	std::vector<cv::Point3f> model;
	std::vector<cv::Point2f> scene;
	cboard->corresponding_points(imagePoints, scene, model);
  
	
	int N = model.size();  
	cv::Mat kmat(kvec);
	cv::Mat Kmat=kmat.reshape(1,3);
	cv::Mat dvec = cv::Mat::zeros(5, 1, cv::DataType<float>::type); // No distortion
	cv::Mat rvec(3, 1, cv::DataType<double>::type);
	cv::Mat tvec(3, 1, cv::DataType<double>::type);
	cv::OutputArray oRvec(rvec), oTvec(tvec);
	if(N>10){
		cv::solvePnP(model, scene, Kmat, dvec, oRvec, oTvec);
	}
	else{
		ROS_WARN("Too few recognized markers");
	}
	
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
	if(N>0) errAve/=N;
	if(errAve<torelance){
		pub2->publish(tf);
		done.data=true;
		geometry_msgs::TransformStamped transformStamped;
		transformStamped.header.stamp = ros::Time::now();
		transformStamped.header.frame_id = "camera";
		transformStamped.child_frame_id = "gridboard";
		transformStamped.transform=tf;
		broadcaster->sendTransform(transformStamped);
	}
	pub4->publish(done);
	ROS_WARN("Ave %f  Max.err %f(%d,%d)",errAve,errMax,errX,errY);
	std_msgs::Float32 stats;
	stats.data=errAve;
	pub5->publish(stats);
}


template <typename T> void load_param(std::string pname, T &value)
{
    std::string param_name = std::string("gridboard/") + pname;
	if (nh->hasParam(param_name)) {
		nh->getParam(param_name, value);
		ROS_INFO("grid_node::param overriden %s %f", pname, (double)value);
	}
	else {
		ROS_WARN("grid_node::param default %s %f", pname, (double)value);
	}
}


void set_PreProcParams(PreProcParams &pp)
{
	load_param("do_reverse_bw", pp.do_reverse_bw);
	load_param("do_equalize_hist", pp.do_equalize_hist);
	load_param("do_smoothing", pp.do_smoothing);
	load_param("bin_type", pp.bin_type);
	load_param("bin_param0", pp.bin_param0);
	load_param("bin_param1", pp.bin_param1);
	load_param("gamma_correction", pp.gamma_correction);
}


void set_CircleMarkerParams(CircleMarkerParams &mp)
{
	load_param("fitscore", mp.fitscore);
	load_param("n_minimum", mp.n_minimum);
	load_param("max_radius", mp.max_radius);
	load_param("min_radius", mp.min_radius);
	load_param("showscale", mp.debug_show_scale);
}


void set_CalibBoardParams(CalibBoardParams & cp)
{
	load_param("unitleng", cp.unit_length);
	load_param("n_circles_x", cp.n_circles_x);
	load_param("n_circles_y", cp.n_circles_y);
	load_param("origin_x", cp.origin_x);
	load_param("origin_y", cp.origin_y);
	load_param("distance_between_circles", cp.distance_between_circles);
}
	

void reload(std_msgs::Bool e)
{
	ROS_INFO("grid_node LD_PATH=%s",getenv("LD_LIBRARY_PATH"));
	
	PreProcParams pprc_param;
	set_PreProcParams(pprc_param);
	
	CircleMarkerParams cmrk_param;
	set_CircleMarkerParams(cmrk_param);
	
	CalibBoardParams cbrd_param;
	set_CalibBoardParams(cbrd_param);
	if (cbrd_param.unit_length == 0.0 || cbrd_param.n_circles_x == 0.0 || cbrd_param.n_circles_y == 0.0 ||
		cbrd_param.origin_x == 0.0 || cbrd_param.origin_y == 0.0) {
		ROS_ERROR("GetGrid::paramer \"CalibBoardParams\" not found");
		return;		
	}
		
	cboard->set_parameters(pprc_param, cmrk_param, cbrd_param);

	if (! nh->getParam(paramK.c_str(), kvec)) {
		ROS_ERROR("GetGrid::paramer \"K\" not found");
		return;
	}
	if (! nh->getParam("gridboard/torelance", torelance)) {
		ROS_WARN("GetGrid::paramer \"torelance\" not found");
	}
	
	ROS_WARN("grid::param::reload %f",torelance);
}



void tf_test(std_msgs::Bool e)
{
	ROS_WARN("gridboard send tf");
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "camera";
	transformStamped.child_frame_id = "gridboard";
	transformStamped.transform.translation.x=0;
	transformStamped.transform.translation.y=0;
	transformStamped.transform.translation.z=0;
	transformStamped.transform.rotation.x=0;
	transformStamped.transform.rotation.y=0;
	transformStamped.transform.rotation.z=0;
	transformStamped.transform.rotation.w=1;
	broadcaster->sendTransform(transformStamped);
}



int main(int argc, char **argv)
{
	if (argc >= 2) {
		paramK = argv[1];
		std::cout << "K=" << paramK << "\n";
	}

	cboard = CreateCalibBoardRecognizer();	
	
	
  
	ros::init(argc, argv, "grid_node");
	ros::NodeHandle n;
	nh = &n;

	broadcaster=new tf2_ros::TransformBroadcaster;

	ros::Subscriber s1=n.subscribe("gridboard/image_in", 1, solve);
	ros::Subscriber s2=n.subscribe("gridboard/reload", 1, reload);
	ros::Subscriber s3=n.subscribe("gridboard/test", 1, tf_test);
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
	ros::Publisher p5 = n.advertise<std_msgs::Float32>("gridboard/stats", 1);
	pub5 = &p5;

	ros::spin();

	cboard->destroy();
	return 0;
}
