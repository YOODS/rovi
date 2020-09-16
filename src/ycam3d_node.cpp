#include <chrono>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "CameraYCAM3D.hpp"
#include "ElapsedTimer.hpp"

namespace {
//============================================= 無名名前空間 start =============================================
//#define DEBUG_DETAIL
#define LOG_HEADER "(ycam3d) "

const std::string FRAME_ID = "camera";
	
enum YCam3DMode {
	Mode_StandBy = 1,
	Mode_Streaming = 2
};
	
ros::NodeHandle *nh = nullptr;

ros::Publisher pub_cam_img_left;
ros::Publisher pub_cam_img_right;
ros::Publisher pub_Y1;

int cam_width = -1;
int cam_height = -1;

const std::string PRM_MODE                = "ycam/Mode";
const std::string PRM_CAM_OPEN_STAT       = "ycam/stat";
const std::string PRM_SW_TRIG_RATE        = "ycam/SoftwareTriggerRate";
const std::string PRM_EXPOSURE_TIME_LEVEL = "ycam/ExposureTimeLevel";
//const std::string PRM_CAM_EXPSR_TM   = "ycam/camera/ExposureTime";
const std::string PRM_CAM_GAIN_D          = "ycam/camera/Gain";
//const std::string PRM_CAM_GAIN_A     = "ycam/camera/GainA";
//const std::string PRM_PROJ_EXPSR_TM  = "ycam/projector/ExposureTime";
const std::string PRM_PROJ_INTENSITY      = "ycam/projector/Intensity";
	
constexpr int PRM_SW_TRIG_RATE_DEFAULT = 2; //Hz
constexpr int PRM_SW_TRIG_RATE_MAX = 6; //Hz
constexpr int PRM_SW_TRIG_RATE_MIN = 1; //Hz

int pre_ycam_mode = (int)Mode_StandBy;

std::string camera_res;

const int YCAM_STAND_BY_MODE_CYCLE = 3; //Hz

std::unique_ptr<CameraYCAM3D> camera_ptr;

std::thread img_trans_thread;
	
ros::Timer mode_mon_timer;
int cur_mode_mon_cyc = YCAM_STAND_BY_MODE_CYCLE; //Hz

bool cam_params_refreshed=false;

int pre_ycam_exposure_level=0;
int pre_cam_gain_d    = 0;
int pre_proj_intensity = 0;

template<typename T>
T get_param(const std::string &key,const T default_val){
	T val = default_val;
	if( ! nh->getParam(key,val) ) {
		ROS_ERROR(LOG_HEADER"error:parameter get failed. key=%s",key.c_str());
	}
	return val;
}

template<typename T>
T get_param(const std::string &key,const T default_val,const T min_val,const T max_val){
	T val = default_val;
	if( ! nh->getParam(key,val) ) {
		ROS_ERROR(LOG_HEADER"error:parameter get failed. key=%s",key.c_str());
		
	}else if( val < min_val){
		val = min_val;
		nh->setParam(key,min_val);
		ROS_ERROR(LOG_HEADER"error:current parameter value is below minimum value. key=%s val=%d min_val=%d",key.c_str(),val,min_val);
		
	}else if( max_val < val){
		val = max_val;
		nh->setParam(key,max_val);
		ROS_ERROR(LOG_HEADER"error:current parameter value is above maximum value. key=%s",key.c_str());
		
	}
	return val;
}

//debug**********
	void update_camera_params();
	
void exec_capture(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("I heard: [%s]", msg->data.c_str());
	
	if( ! camera_ptr ){
		//todo:Y1結果返す
		return;
	}
	
	//camera_ptr->close();
	//camera_ptr->start_auto_connect();
	//camera_ptr->set_exposure_time(30000);
	//camera_ptr->set_gain_digital(80);
	//camera_ptr->capture();
	
	ElapsedTimer tt;
	//update_camera_params();
	
	//camera_ptr->close();
	//camera_ptr->capture();
	camera_ptr->capture_strobe();
	
	//camera_ptr.reset();
}

bool init(){
	bool ret=false;
	if( ! nh->getParam("camera/Width", cam_width) ){
		ROS_ERROR(LOG_HEADER"error:camera width get failed.");
		
	}else if( ! nh->getParam("camera/Height", cam_height) ){
		ROS_ERROR(LOG_HEADER"error:camera height get failed.");
		
	}else if( ! nh->getParam("camera/ID",camera_res) ){
		ROS_ERROR(LOG_HEADER"error:camera resolution value not found.");
		
	}else{
		pre_ycam_mode = get_param<int>(PRM_MODE,(int)Mode_StandBy);
		if(pre_ycam_mode == Mode_Streaming){
			cur_mode_mon_cyc = get_param<int>(PRM_SW_TRIG_RATE,PRM_SW_TRIG_RATE_DEFAULT);
		}else{
			cur_mode_mon_cyc = YCAM_STAND_BY_MODE_CYCLE;
		}
				
		ret=true;
	}
	return ret;
}

void update_camera_params(){
	if( ! cam_params_refreshed ){
		ROS_WARN(LOG_HEADER"camera parameters have not been refreshed.");
		return;
	}
	
	ElapsedTimer tmr;
	if( ! camera_ptr || ! camera_ptr->is_open()){
		ROS_ERROR(LOG_HEADER"error:camera is not opened.");
		return;
	}
	const int cur_ycam_exposure_level = get_param<int>( PRM_EXPOSURE_TIME_LEVEL,
		camera::ycam3d::YCAM_EXPOSURE_TIME_LEVEL_DEFAULT, camera::ycam3d::YCAM_EXPOSURE_TIME_LEVEL_MIN, camera::ycam3d::YCAM_EXPOSURE_TIME_LEVEL_MAX);
	if( pre_ycam_exposure_level != cur_ycam_exposure_level ){
		if( ! camera_ptr->set_exposure_time_level(cur_ycam_exposure_level) ){
			ROS_ERROR(LOG_HEADER"error:'exposure time level' set failed. val=%d",cur_ycam_exposure_level);
			int latest_val=0;
			if( ! camera_ptr->get_exposure_time_level(&latest_val) ){
				ROS_ERROR(LOG_HEADER"error:'exposure time level' get failed.");
			}else{
				ROS_ERROR(LOG_HEADER"error:'exposure time level' set failed. set_val=%d cur_val=%d", cur_ycam_exposure_level, latest_val);
				nh->setParam(PRM_EXPOSURE_TIME_LEVEL, latest_val);
				pre_ycam_exposure_level = latest_val;
			}
		}else{
			ROS_INFO(LOG_HEADER"'exposure time level' paramter changed. old=%d new=%d", pre_ycam_exposure_level, cur_ycam_exposure_level);
			pre_ycam_exposure_level = cur_ycam_exposure_level;
		}
	}
	
	const int cur_cam_gain_d = get_param<int>( PRM_CAM_GAIN_D,
		camera::ycam3d::CAM_DIGITAL_GAIN_DEFAULT, camera::ycam3d::CAM_DIGITAL_GAIN_MIN, camera::ycam3d::CAM_DIGITAL_GAIN_MAX);
	if( pre_cam_gain_d != cur_cam_gain_d ){
		if( ! camera_ptr->set_gain_digital(cur_cam_gain_d) ){
			int latest_val=0;
			if( ! camera_ptr->get_gain_digital(&latest_val) ){
				ROS_ERROR(LOG_HEADER"error:'camera digital gain' get failed.");
			}else{
				ROS_ERROR(LOG_HEADER"error:'camera digital gain' set failed. set_val=%d cur_val=%d", cur_cam_gain_d, latest_val);
				nh->setParam(PRM_CAM_GAIN_D, latest_val);
				pre_cam_gain_d = latest_val;
			}
		}else{
			ROS_INFO(LOG_HEADER"'camera digital gain' paramter changed. old=%d new=%d", pre_cam_gain_d, cur_cam_gain_d);
			pre_cam_gain_d = cur_cam_gain_d;
		}
	}
	
	const int cur_proj_intensty = get_param<int>( PRM_PROJ_INTENSITY,
		camera::ycam3d::PROJ_FLASH_INTERVAL_DEFAULT, camera::ycam3d::PROJ_FLASH_INTERVAL_MIN, camera::ycam3d::PROJ_FLASH_INTERVAL_MAX );	
	if( pre_proj_intensity != cur_proj_intensty ){
		if( ! camera_ptr->set_projector_brightness(cur_proj_intensty) ){
			int latest_val = 0;
			if( ! camera_ptr->get_projector_brightness(&latest_val) ){
				ROS_ERROR(LOG_HEADER"error:'projector intensity' get failed.");
			}else{
				ROS_ERROR(LOG_HEADER"error:'projector intensity' set failed. set_val=%d cur_val=%d", cur_proj_intensty, latest_val);
				nh->setParam(PRM_PROJ_INTENSITY, latest_val);
				pre_proj_intensity = latest_val;
			}
		}else{
			ROS_INFO(LOG_HEADER"'projector intensity' paramter changed. old=%d new=%d", pre_proj_intensity, cur_proj_intensty);
			pre_proj_intensity = cur_proj_intensty;
		}
	}
	
	//ROS_INFO(LOG_HEADER"camera parameter updated. elapsed=%d ms",tmr.elapsed_ms());
}

void mode_monitor_task(const ros::TimerEvent& e){
	const int cur_ycam_mode = get_param<int>(PRM_MODE,(int)Mode_StandBy);
	
	if( ! camera_ptr ){
		return;
	}
	
	//ROS_INFO(LOG_HEADER"ycam mode=%d ",cur_ycam_mode);
	if( ! camera_ptr->is_open() ){
		//ROS_ERROR(LOG_HEADER"error:camera disconnect.");
		//ROS_WARN(LOG_HEADER"error:camera is disconnected. trying to connect ...");
		nh->setParam(PRM_CAM_OPEN_STAT,false);
		return;
	}else{
		nh->setParam(PRM_CAM_OPEN_STAT,true);
	}
	
	//パラメータ取得
	update_camera_params();
	
	if( cur_ycam_mode == Mode_Streaming){
		if(camera_ptr->is_busy()){
			ROS_WARN(LOG_HEADER"camera is busy.!!!!");
		}else{
			int strobe=0;
			if(nh->getParam("ycam/Strobe",strobe) && strobe != 0){
				camera_ptr->capture_strobe();
			}else{
				camera_ptr->capture();
			}
			
		}
	}else{
		
	}
	
	//監視周期変更
	if( pre_ycam_mode != cur_ycam_mode){
		ROS_INFO(LOG_HEADER"switched. pre=%d => cur=%d",pre_ycam_mode,cur_ycam_mode);
		
		std::string mode_name;
		if( cur_ycam_mode == Mode_Streaming ){
			cur_mode_mon_cyc = get_param<int>(PRM_SW_TRIG_RATE, PRM_SW_TRIG_RATE_DEFAULT, PRM_SW_TRIG_RATE_MIN, PRM_SW_TRIG_RATE_MAX);
			mode_name = "Streaming";
			
		}else{
			cur_mode_mon_cyc = YCAM_STAND_BY_MODE_CYCLE;
			mode_name = "StandBy";
			
		}
		
		ROS_INFO(LOG_HEADER"changed to '%s' mode. cycle=%d Hz",mode_name.c_str(),cur_mode_mon_cyc);
		mode_mon_timer.setPeriod(ros::Duration(1/(float)cur_mode_mon_cyc),true);
		
	}else if( cur_ycam_mode == Mode_Streaming ){
		const int mode_mon_cyc = get_param<int>(PRM_SW_TRIG_RATE, PRM_SW_TRIG_RATE_DEFAULT, PRM_SW_TRIG_RATE_MIN, PRM_SW_TRIG_RATE_MAX);
		if( cur_mode_mon_cyc != mode_mon_cyc ){
			ROS_INFO(LOG_HEADER"'Streaming' mode cycle changed. %d Hz => %d Hz",cur_mode_mon_cyc,mode_mon_cyc);
			
			mode_mon_timer.setPeriod(ros::Duration(1/(float)mode_mon_cyc),true);
			cur_mode_mon_cyc = mode_mon_cyc;
		}
	}
	
	pre_ycam_mode = cur_ycam_mode;
}

void phase_shift_capture_task(){
	ROS_INFO(LOG_HEADER"phase shift capture start.");
	ElapsedTimer tmr;
	
	//sensor_msgs::ImagePtr img(new sensor_msgs::Image());
	//pub_cam_img_left.publish(img);
	
	ROS_INFO(LOG_HEADER"phase shift capture finished. elapsed=%d ms",tmr.elapsed_ms());
	
}

void on_camera_open_finished(const bool result){
	ROS_INFO(LOG_HEADER"camera opened. result=%s",(result?"OK":"NG"));
	if( ! result ){
		return;
	}
	

	pre_cam_gain_d = get_param<int>( PRM_CAM_GAIN_D,
		camera::ycam3d::CAM_DIGITAL_GAIN_DEFAULT, camera::ycam3d::CAM_DIGITAL_GAIN_MIN, camera::ycam3d::CAM_DIGITAL_GAIN_MAX);
	
	pre_proj_intensity = get_param<int>( PRM_PROJ_INTENSITY,
		camera::ycam3d::PROJ_FLASH_INTERVAL_DEFAULT, camera::ycam3d::PROJ_FLASH_INTERVAL_MIN, camera::ycam3d::PROJ_FLASH_INTERVAL_MAX );	
	
	ROS_INFO(LOG_HEADER"camera digital gain = %d", pre_cam_gain_d);
	ROS_INFO(LOG_HEADER"projector intensity = %d", pre_proj_intensity);
	
	cam_params_refreshed=true;
}
	
void on_camera_closed(){
	ROS_INFO(LOG_HEADER"camera closed.");
	cam_params_refreshed=false;
}
	
void on_capture_image_received(const bool result,const int proc_tm, const camera::ycam3d::CameraImage &img_l,const camera::ycam3d::CameraImage &img_r){
	ROS_INFO(LOG_HEADER"capture image recevie start. result=%s, proc_tm=%d ms, imgs_l: result=%d size=%d x %d, imgs_r: result=%d size=%d x %d",
		(result?"true":"false"), proc_tm, img_l.result, img_l.width, img_l.height, img_r.result, img_r.width, img_r.height);
	
	const ros::Time now = ros::Time::now();
	
	ElapsedTimer tmr;
	sensor_msgs::Image ros_imgs[2];
	const camera::ycam3d::CameraImage cam_imgs[2] = { img_l, img_r };
	
	if( ! result ){
		ROS_ERROR(LOG_HEADER"error:capture failed.");
	}else{
		
		for (int i = 0 ; i < 2 ; i++ ){
			sensor_msgs::Image *ros_img=&ros_imgs[i];
			const camera::ycam3d::CameraImage *cam_img=&cam_imgs[i];
			ros_img->header.stamp = now;
			ros_img->header.frame_id = FRAME_ID;
			ros_img->width = cam_img->width;
			ros_img->height = cam_img->height;
			ros_img->encoding = sensor_msgs::image_encodings::MONO8;
			ros_img->step = cam_img->step;
			ros_img->is_bigendian = false;
			ros_img->data.resize(cam_img->byte_count());
			memcpy(ros_img->data.data(),cam_img->data.data(),cam_img->byte_count());
		}
	}
	/*
	cv::Mat img = cv_bridge::toCvCopy(ros_imgs[0], sensor_msgs::image_encodings::MONO8)->image;
	cv::imwrite("/home/ros/cam_l.png",img);
	
	img = cv_bridge::toCvCopy(ros_imgs[1], sensor_msgs::image_encodings::MONO8)->image;
	cv::imwrite("/home/ros/cam_r.png",img);
	
	ROS_INFO(LOG_HEADER"ros image convert. elapsed=%d",tmr.elapsed_ms());
	*/
	
	//ROS_INFO(LOG_HEADER"capture image recevie finished. elapsed=%d ms",tmr.elapsed_ms());
	
	pub_cam_img_left.publish(ros_imgs[0]);
	pub_cam_img_right.publish(ros_imgs[1]);
}

void on_pattern_image_received(const bool result,const int proc_tm, const std::vector<camera::ycam3d::CameraImage> &imgs_l,const std::vector<camera::ycam3d::CameraImage> &imgs_r){
	ROS_INFO(LOG_HEADER"pattern image recevied. result=%s, proc_tm=%d ms, imgs_l: num=%d, imgs_r: num=%d",
		(result?"true":"false"), proc_tm, (int)imgs_l.size(),(int)imgs_r.size());
	
	ElapsedTimer tmr;
	tmr.restart();
	
	if( ! result ){
		ROS_ERROR(LOG_HEADER"error:pattern capture failed.");
	}else{
		char path[256];
		//left images 
		for(int i=0;i<imgs_l.size();++i){
			sprintf(path,"/tmp/capt_%02d_0.pgm",i);
			cv::imwrite(path,imgs_l.at(i).to_mat());
		}
		
		//right images 
		for(int i=0;i<imgs_l.size();++i){
			sprintf(path,"/tmp/capt_%02d_1.pgm",i);
			cv::imwrite(path,imgs_r.at(i).to_mat());
		}
	}
	
	ROS_INFO(LOG_HEADER"elapsed tm=%d",tmr.elapsed_ms());
	
	/*
	//ElapsedTimer tmr;
	tmr.restart();
	ElapsedTimer tmr_1;
	const ros::Time now = ros::Time::now();
	
	sensor_msgs::Image ros_img_l;
	ros_img_l.header.stamp = now;
	ros_img_l.header.frame_id = FRAME_ID;
	ros_img_l.width = width;
	ros_img_l.height = height;
	ros_img_l.encoding = sensor_msgs::image_encodings::MONO8;
	ros_img_l.step = width;
	ros_img_l.is_bigendian = false;
	ros_img_l.data.resize(width * height);
	ROS_INFO(LOG_HEADER"make ros image. tm=%d",tmr.elapsed_ms());
	tmr.restart();
	//std::copy((char*)mem,((char*)mem) + width*height,ros_img_l.data.data());
	memcpy(ros_img_l.data.data(),mem,width * height);
	ROS_INFO(LOG_HEADER"copy ros image. tm=%d",tmr.elapsed_ms());
	
	tmr.restart();
	cv::Mat img = cv_bridge::toCvCopy(ros_img_l, sensor_msgs::image_encodings::MONO8)->image;
	ROS_INFO(LOG_HEADER"ros image convert. tm=%d",tmr.elapsed_ms());
	
	tmr.restart();
	cv::imwrite("/home/ros/catkin_ws/src/rovi/test.png",img);
	ROS_INFO(LOG_HEADER"mat image saved. tm=%d",tmr.elapsed_ms());
	
	ROS_INFO(LOG_HEADER"elapsed tm=%d",tmr_1.elapsed_ms());
	*/
}
//============================================= 無名名前空間  end  =============================================
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ycam3d_node");
	ros::NodeHandle n;
	nh = &n;
	
	ROS_INFO(LOG_HEADER"initialization start.");
	if( ! init() ){
		ROS_ERROR(LOG_HEADER"error: initialization failed.");
		return 1;
	}
	ROS_INFO(LOG_HEADER"initialization finished");
	
	ROS_INFO(LOG_HEADER"camera: size= %d x %d, resolution=%s", cam_width, cam_height, camera_res.c_str());
	ROS_INFO(LOG_HEADER"ycam3d: mode=%d, cycle=%d Hz", pre_ycam_mode, cur_mode_mon_cyc);
	
	camera_ptr.reset(new CameraYCAM3D());
	
	if( ! camera_ptr->init(camera_res) ){
		ROS_ERROR(LOG_HEADER"error: camera initialization failed.");
		return 1;
	}
	
	camera_ptr->set_callback_camera_open_finished(on_camera_open_finished);
	camera_ptr->set_callback_camera_closed(on_camera_closed);
	camera_ptr->set_callback_capture_img_received(on_capture_image_received);
	camera_ptr->set_callback_pattern_img_received(on_pattern_image_received);
	
	//ros::Publisher pub1 = n.advertise<sensor_msgs::Image>("left/image_raw", 1);
	//pub_cam_img_left = &pub1;
	pub_cam_img_left = n.advertise<sensor_msgs::Image>("left/image_raw", 1);
	
	//ros::Publisher pub2 = n.advertise<sensor_msgs::Image>("right/image_raw", 1);
	//pub_cam_img_right = &pub2;
	pub_cam_img_right = n.advertise<sensor_msgs::Image>("right/image_raw", 1);

	//ros::Publisher pub3 = n.advertise<sensor_msgs::Image>("Y1", 1);
	//pub_Y1 = &pub3;
	pub_Y1 = n.advertise<sensor_msgs::Image>("Y1", 1);
	
	const ros::Subscriber sub1 = n.subscribe("X1", 1, exec_capture );
	
	mode_mon_timer = n.createTimer(ros::Duration(1/(float)cur_mode_mon_cyc), mode_monitor_task);
	
	//streaming_cycle_timer = n.createTimer(ros::Duration(1 /(float)sw_trig_rate ), streaming_cyc_task);
	
	//live.reset(new CameraLive(nh));
	//live->start();
	//const ros::ServiceServer svc1 = n.advertiseService("ycam3d", ycam3d_service);
	
	camera_ptr->start_auto_connect();
	
	ros::spin();
	
	mode_mon_timer.stop();
	ROS_INFO(LOG_HEADER"mode monitor timer stopped.");
	
	//todo:******画像転送スレッドの終了待ち
	ROS_INFO(LOG_HEADER"image transfer task wait start.");

	ROS_INFO(LOG_HEADER"image transfer task wait finished.");
	
	if( camera_ptr ){
		ROS_INFO(LOG_HEADER"camera close start.");
		camera_ptr.reset();
		ROS_INFO(LOG_HEADER"camera close finished.");
	}
	return 0;
}
