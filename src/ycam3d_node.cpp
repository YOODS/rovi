#include <chrono>
#include <thread>
#include <mutex>
#include <sstream>

#include <condition_variable>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "iPointCloudGenerator.hpp"
#include "CameraYCAM3D.hpp"
#include "ElapsedTimer.hpp"
#include "rovi/Floats.h"
#include "rovi/GenPC.h"
#include "rovi/ImageFilter.h"


//#define DEBUG_DETAIL
//#define DEBUG_STRESS_TEST
//#define DEBUG_PTN_IMG_SAVE

namespace {
//============================================= 無名名前空間 start =============================================
#define LOG_HEADER "(ycam3d) "

const std::string FRAME_ID = "camera";
	
enum YCam3DMode {
	Mode_StandBy = 1,
	Mode_Streaming = 2
};
	
ros::NodeHandle *nh = nullptr;

ros::Publisher pub_img_raws[2];
ros::Publisher pub_Y1;
//ros::Publisher pub_pcount;
ros::Publisher pub_stat;
ros::Publisher pub_error;
ros::Publisher pub_info;

ros::Publisher pub_rects[2];
ros::Publisher pub_rects0[2];
ros::Publisher pub_rects1[2];
ros::Publisher pub_diffs[2];
//ros::Publisher pub_views[2];
ros::Publisher pub_temperature;

ros::ServiceClient svc_genpc;
ros::ServiceClient svc_remap[2];

std::string cam_ipaddr;
int cam_width = -1;
int cam_height = -1;

const std::string PRM_MODE                    = "ycam/Mode";
const std::string PRM_CAM_OPEN_STAT           = "ycam/stat";
const std::string PRM_SW_TRIG_RATE            = "ycam/SoftwareTriggerRate";
const std::string PRM_EXPOSURE_TIME_LEVEL     = "ycam/ExposureTimeLevel";
const std::string PRM_TEMP_MON_INTERVAL       = "ycam/TemperatureMonitorInterval";
const std::string PRM_DRAW_CAMERA_ORIGIN      = "ycam/DrawCameraOrigin";
const std::string PRM_PCGEN_PUBLISH           = "ycam/pcgen_publish";
//const std::string PRM_CAM_EXPSR_TM          = "ycam/camera/ExposureTime";
const std::string PRM_CAM_GAIN_D              = "ycam/camera/Gain";
//const std::string PRM_CAM_GAIN_A            = "ycam/camera/GainA";
//const std::string PRM_PROJ_EXPSR_TM         = "ycam/projector/ExposureTime";
const std::string PRM_PROJ_INTENSITY          = "ycam/projector/Intensity";
const std::string PRM_HDR_ENABLED             = "ycam/hdr/enabled";
const std::string PRM_HDR_PCGEN_PUBLISH       = "ycam/hdr/pcgen_publish";
const std::string PRM_HDR_EXPOSURE_TIME_LEVEL = "ycam/hdr/ExposureTimeLevel";
const std::string PRM_HDR_CAM_GAIN_D          = "ycam/hdr/camera/Gain";
const std::string PRM_HDR_PROJ_INTENSITY      = "ycam/hdr/projector/Intensity";

const std::string PRM_NW_DELAY_MON_ENABLED       = "ycam/nw_delay_monitor/enabled";
const std::string PRM_NW_DELAY_MON_INTERVAL      = "ycam/nw_delay_monitor/interval";
const std::string PRM_NW_DELAY_MON_TIMEOUT       = "ycam/nw_delay_monitor/timeout";
const std::string PRM_NW_DELAY_MON_IGN_UPD_FAIL  = "ycam/nw_delay_monitor/ignore_update_failure";

const std::string PRM_CAPT_TIMEOUT_RESET         = "ycam/CaptureTimeoutReset";

const std::string PRM_CAM_CALIB_MAT_K_LIST[]  = {"left/remap/Kn","right/remap/Kn"};

constexpr int PRM_SW_TRIG_RATE_DEFAULT = 2; //Hz
constexpr int PRM_SW_TRIG_RATE_MAX = 6; //Hz
constexpr int PRM_SW_TRIG_RATE_MIN = 1; //Hz

constexpr int YCAM_STAND_BY_MODE_CYCLE = 3; //Hz

constexpr int TEMP_MON_INTERVAL_DEFAULT = 5; //Sec
	
int pre_ycam_mode = (int)Mode_StandBy;

std::string camera_res;
std::unique_ptr<CameraYCAM3D> camera_ptr;

ros::Timer mode_mon_timer;
int cur_mode_mon_cyc = YCAM_STAND_BY_MODE_CYCLE; //Hz

ros::Timer cam_open_mon_timer;

ros::Timer temp_mon_timer;
int cur_temp_mon_interval = -1;
int temp_acq_failure_count = 0;
const int TEMP_ACQ_FAILURE_MSG_REPEAT_MAX = 3;

void exec_get_ycam_temperature(const ros::TimerEvent& e);

bool cam_params_refreshed=false;

int pre_cam_gain_d    = 0;
int pre_proj_intensity = 0;

std::timed_mutex pc_gen_mutex;
std::mutex ptn_capt_wait_mutex;
std::condition_variable ptn_capt_wait_cv;
std::thread pc_gen_thread;

bool cur_hdr_enabled=false;
	
int g_node_exit_flg = 0;


	
struct RosCaptureParameter: public camera::ycam3d::CaptureParameter{
	bool pcgen_publish = true;
	
	std::string to_string()const{
		std::stringstream ss;
		ss << camera::ycam3d::CaptureParameter::to_string();
		ss << ",pcgen_publish=" << pcgen_publish;
		return ss.str();
	}
	
	bool operator==(const RosCaptureParameter &param)const{
		if( *this != param ||
		    this->pcgen_publish != param.pcgen_publish ){
			return false;
		}
		return true;
	}
};
std::vector<RosCaptureParameter> cur_capt_params;
	
struct PatternImageData{
	std::vector<camera::ycam3d::CameraImage> imgs_l;
	std::vector<camera::ycam3d::CameraImage> imgs_r;
};

std::vector<PatternImageData> ptn_imgs;
//int ptn_capt_num = 1;

struct RosPatternImageData{
	std::vector<sensor_msgs::Image> imgs[2];
};

int expsr_tm_lv_default = -1;
int expsr_tm_lv_min = -1;
int expsr_tm_lv_max = -1;

std::map<PcGenMode,std::string> PCGEN_MODE_MAP = {
	{PCGEN_SGBM,"SGBM"},
	{PCGEN_GRAYPS4,"Gray+4step_PS"},
	{PCGEN_MULTI,"Multi"} 
};
	
//**********DEBUG
#ifdef DEBUG_STRESS_TEST
int debug_pre_strobe = 0;
int debug_capt_watch_count = 0;
#endif
	
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
		ROS_ERROR(LOG_HEADER"error:current parameter value is below minimum value. key=%s val=%d min_val=%d",key.c_str(),val,min_val);
		val = min_val;
		nh->setParam(key,min_val);
		
	}else if( max_val < val){
		val = max_val;
		nh->setParam(key,max_val);
		ROS_ERROR(LOG_HEADER"error:current parameter value is above maximum value. key=%s",key.c_str());
		
	}
	return val;
}
	
void publish_bool(ros::Publisher&pub, const bool val){
	std_msgs::Bool rmsg;
	rmsg.data = val;
	pub.publish(rmsg);
}

void publish_int32(ros::Publisher&pub, const int val){
	std_msgs::Int32 rmsg;
	rmsg.data = val;
	pub.publish(rmsg);
}
	
void publish_float32(ros::Publisher&pub, const float val){
	std_msgs::Float32 rmsg;
	rmsg.data = val;
	pub.publish(rmsg);
}

void publish_string(ros::Publisher&pub, const std::string &val){
	std_msgs::String rmsg;
	rmsg.data = val;
	pub.publish(rmsg);
}


bool init(){
	bool ret=false;
	if( ! nh->getParam("camera/address",cam_ipaddr) ){
		ROS_ERROR(LOG_HEADER"error:camera ipaddr get failed.");
		
	}else if( ! nh->getParam("camera/Width", cam_width) ){
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


bool validate_capt_param(camera::ycam3d::CaptureParameter &capt_param,const std::string prefix=""){
	bool ret=true;
	
	if( capt_param.expsr_lv < 0 ){
		//skipped
	}else if( capt_param.expsr_lv < expsr_tm_lv_min ){
		ret=false;
		ROS_ERROR(LOG_HEADER"%sExposure Time Level is under minimum. cur=%d, min=%d",prefix.c_str(),capt_param.expsr_lv + 1, expsr_tm_lv_min + 1);
	}else if( expsr_tm_lv_max < capt_param.expsr_lv ){
		ret=false;
		ROS_ERROR(LOG_HEADER"%sExposure Time Level is over maximum. cur=%d, max=%d",prefix.c_str(),capt_param.expsr_lv + 1, expsr_tm_lv_max + 1);
	}
	
	if( capt_param.gain < 0 ){
		//skipped
	}else if( capt_param.gain < camera::ycam3d::CAM_DIGITAL_GAIN_MIN ){
		ret=false;
		ROS_ERROR(LOG_HEADER"%sCamera Gain is under minimum. cur=%d, min=%d",prefix.c_str(), capt_param.gain , camera::ycam3d::CAM_DIGITAL_GAIN_MIN );
	}else if( camera::ycam3d::CAM_DIGITAL_GAIN_MAX < capt_param.gain ){
		ret=false;
		ROS_ERROR(LOG_HEADER"%sCamera Gain is over maximum. cur=%d, max=%d",prefix.c_str(), capt_param.gain , camera::ycam3d::CAM_DIGITAL_GAIN_MAX );
	}
	
	if( capt_param.proj_intensity < 0 ){
		//skipped
	}else if( capt_param.proj_intensity < camera::ycam3d::PROJ_INTENSITY_MIN ){
		ret=false;
		ROS_ERROR(LOG_HEADER"%sProjector Intensity is under minimum. cur=%d, min=%d", prefix.c_str(), capt_param.proj_intensity , camera::ycam3d::PROJ_INTENSITY_MIN);
	}else if( camera::ycam3d::PROJ_INTENSITY_MAX < capt_param.proj_intensity ){
		ret=false;
		ROS_ERROR(LOG_HEADER"%sProjector Intensity is over maximum. cur=%d, max=%d", prefix.c_str(), capt_param.proj_intensity , camera::ycam3d::PROJ_INTENSITY_MAX);
	}
	
	return ret;
}

void update_camera_params(){
	if( ! cam_params_refreshed ){
		ROS_WARN(LOG_HEADER"camera parameters have not been refreshed.");
		return;
	}
	
	ElapsedTimer tmr;
	if( ! camera_ptr || ! camera_ptr->is_open() ){
		ROS_ERROR(LOG_HEADER"error:camera is not opened.");
		return;
	}
	
	const int tempMonInterval = get_param<int>(PRM_TEMP_MON_INTERVAL,TEMP_MON_INTERVAL_DEFAULT);
	if( tempMonInterval == cur_temp_mon_interval ){
		//ROS_INFO(LOG_HEADER"temperature monitor interval no change.");
	}else{
		if( temp_mon_timer.isValid() ){
			ROS_INFO(LOG_HEADER"temperature monitor interval changed. befor=%d sec, after=%d sec",cur_temp_mon_interval,tempMonInterval);
			
			temp_mon_timer.stop();
			temp_mon_timer.setPeriod(ros::Duration(tempMonInterval));
			temp_mon_timer.start();
			
			cur_temp_mon_interval = tempMonInterval;
		}
	}

	//撮影パラメータ
	{
		bool capt_params_valid=true;
		std::vector<RosCaptureParameter> capt_params;
		RosCaptureParameter capt_param;
		//first
		{
			capt_param.pcgen_publish = get_param<bool>(PRM_PCGEN_PUBLISH,true);
			
			const int expsr_tm_lv_ui = get_param<int>(PRM_EXPOSURE_TIME_LEVEL,-1) ;
			if( expsr_tm_lv_ui - 1 < expsr_tm_lv_min ){
				capt_params_valid=false;
				ROS_ERROR(LOG_HEADER"Exposure Time Level is under minimum. val=%d min=%d",expsr_tm_lv_ui, expsr_tm_lv_min);
			}else{
				capt_param.expsr_lv = expsr_tm_lv_ui -1;
				capt_param.gain = get_param<int>(PRM_CAM_GAIN_D,-1);
				capt_param.proj_intensity = get_param<int>(PRM_PROJ_INTENSITY,-1);
				if( ! validate_capt_param(capt_param) ){
					capt_params_valid = false;
				}else{
					capt_params.push_back(capt_param);
				}
			}
		}
		
		//HDR
		const bool hdr_enabled = get_param<bool>(PRM_HDR_ENABLED,false);
		if( capt_params_valid && hdr_enabled ){
			RosCaptureParameter hdr_capt_param;
			
			hdr_capt_param.pcgen_publish = get_param<bool>(PRM_HDR_PCGEN_PUBLISH,false);
			if(nh->hasParam(PRM_HDR_EXPOSURE_TIME_LEVEL)){
				const int expsr_tm_lv_ui = get_param<int>(PRM_HDR_EXPOSURE_TIME_LEVEL,-1) ;
				if( expsr_tm_lv_ui - 1 < expsr_tm_lv_min ){
					capt_params_valid=false;
					ROS_ERROR(LOG_HEADER"HDR Exposure Time Level is under minimum. val=%d min=%d",expsr_tm_lv_ui, expsr_tm_lv_min);
				}else{
					hdr_capt_param.expsr_lv = expsr_tm_lv_ui -1;
				}
			}
			
			if(nh->hasParam(PRM_HDR_CAM_GAIN_D)){
				const float hdr_gain = get_param<float>(PRM_HDR_CAM_GAIN_D,-1);
				if( 0 > hdr_gain ){
				    hdr_capt_param.gain =-1;
				}else if( 0 == hdr_gain ){
					hdr_capt_param.gain = 0;
				}else if( 1 > hdr_gain ){
					if( capt_param.gain > 0){
						hdr_capt_param.gain = (int)(capt_param.gain * hdr_gain);
					}
				}else{
					hdr_capt_param.gain = (int)hdr_gain;
				}
			}
			
			if(nh->hasParam(PRM_HDR_PROJ_INTENSITY)){
				const float hdr_proj_intensity = get_param<float>(PRM_HDR_PROJ_INTENSITY,-1);
				if( 0 > hdr_proj_intensity ){
				    hdr_capt_param.proj_intensity =-1;
				}else if( 0 == hdr_proj_intensity ){
					hdr_capt_param.proj_intensity = 0;
				}else if( 1 > hdr_proj_intensity ){
					if( capt_param.proj_intensity > 0){
						hdr_capt_param.proj_intensity = (int)(capt_param.proj_intensity * hdr_proj_intensity);
					}
				}else{
					hdr_capt_param.proj_intensity = (int)hdr_proj_intensity;
				}
			}
				
			if( ! validate_capt_param(hdr_capt_param,"HDR ")){
				capt_params_valid = false;
			}else{
				capt_params.push_back(hdr_capt_param);
			}
			
		}
		
		if( ! capt_params_valid ){
			ROS_ERROR(LOG_HEADER"");
		}else{
			bool capt_params_changed_flg = false;
			if( cur_capt_params.size() != capt_params.size() ){
				capt_params_changed_flg = true;
			}else{
				for( int i = 0 ; i < capt_params.size() ; ++i ){					
					if(  ! ( cur_capt_params.at(i) == capt_params.at(i) ) ){
						capt_params_changed_flg = true;
						break;
					}
				}
			}
			
			if( capt_params_changed_flg ){
				for( int i = 0 ; i < capt_params.size() ; ++i ){
					ROS_INFO(LOG_HEADER"<%d> capt param updated. %s",i, capt_params.at(i).to_string().c_str());
				}
			}
			
			cur_hdr_enabled = hdr_enabled;
			cur_capt_params = capt_params;
			
			{
				ElapsedTimer tt;
				RosCaptureParameter real_capt_param;
				if( ! camera_ptr->get_capture_param( &real_capt_param ) ){
					ROS_WARN(LOG_HEADER"current capt param get failed.");
				}else if( ! real_capt_param.is_different(capt_params.front()) ){
					//same
				}else{
					if( ! camera_ptr->update_capture_param(capt_params.front()) ){
						ROS_ERROR(LOG_HEADER"capture param update failed.");
					}else{
						ROS_INFO(LOG_HEADER"capture param update success.");
					}
				}
			}
		}
	}
	//ROS_INFO(LOG_HEADER"camera parameter updated. elapsed=%d ms",tmr.elapsed_ms());
}

void cam_open_monitor_task(const ros::TimerEvent& e){
	if( ! camera_ptr || ! camera_ptr->is_open()){
		return;
	}
	publish_bool(pub_stat,true);
}

void mode_monitor_task(const ros::TimerEvent& e){
	const int cur_ycam_mode = get_param<int>(PRM_MODE,(int)Mode_StandBy);
	
	if( ! camera_ptr ){
		return;
	}
	
	if( ! pc_gen_mutex.try_lock_for(std::chrono::seconds(0)) ){
		ROS_WARN(LOG_HEADER"genpc is working. live was skipped.");
		return;
	}
	pc_gen_mutex.unlock();
	
	
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
	
	if( cur_ycam_mode == Mode_Streaming ){
		if( camera_ptr->is_busy() ){
			ROS_WARN(LOG_HEADER"camera is busy.!!!!");
		}else{
#ifdef DEBUG_STRESS_TEST
			ROS_WARN(LOG_HEADER"live capture start.");
#endif
			int strobe=0;
			bool doStrobe=false;
			if(nh->getParam("ycam/Strobe",strobe) && strobe != 0){
				doStrobe=true;
			}
			
			camera_ptr->capture(doStrobe);
#ifdef DEBUG_STRESS_TEST
			debug_pre_strobe = strobe;
#endif
		}
	}else{
		//todo ******************* pending *******************
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

void on_camera_open_finished(const bool result){
	ROS_INFO(LOG_HEADER"camera opened. result=%s",(result?"OK":"NG"));
	
	//expsr_tm_lv_ui_default = -1;
	//expsr_tm_lv_ui_min = -1;
	//expsr_tm_lv_ui_max = -1;
	
	if( ! result ){
		return;
	}

	if( ! camera_ptr->get_exposure_time_level_default(&expsr_tm_lv_default) ){
		ROS_ERROR(LOG_HEADER"exposure time level default val get failed.");
	}else if( ! camera_ptr->get_exposure_time_level_min(&expsr_tm_lv_min) ){
		ROS_ERROR(LOG_HEADER"exposure time level min get failed.");
	}else if( ! camera_ptr->get_exposure_time_level_max(&expsr_tm_lv_max) ){
		ROS_ERROR(LOG_HEADER"exposure time level max get failed.");
	}else{		
		ROS_INFO(LOG_HEADER"exposure time level: min=%d max=%d", expsr_tm_lv_min, expsr_tm_lv_max);
	}
	
	pre_cam_gain_d = get_param<int>( PRM_CAM_GAIN_D,
		camera::ycam3d::CAM_DIGITAL_GAIN_DEFAULT, camera::ycam3d::CAM_DIGITAL_GAIN_MIN, camera::ycam3d::CAM_DIGITAL_GAIN_MAX);
	
	pre_proj_intensity = get_param<int>( PRM_PROJ_INTENSITY,
		camera::ycam3d::PROJ_INTENSITY_DEFAULT, camera::ycam3d::PROJ_INTENSITY_MIN, camera::ycam3d::PROJ_INTENSITY_MAX );	
	
	ROS_INFO(LOG_HEADER"camera digital gain = %d", pre_cam_gain_d);
	ROS_INFO(LOG_HEADER"projector intensity = %d", pre_proj_intensity);
	
	cam_params_refreshed=true;
	if( ! result ){
		publish_string(pub_error,"YCAM open failed.");
	}else{
		publish_string(pub_info,"YCAM ready.");
	}
	
	if(result){
		const int tempMonInterval = get_param<int>(PRM_TEMP_MON_INTERVAL,TEMP_MON_INTERVAL_DEFAULT);
		cur_temp_mon_interval = tempMonInterval;
		if( tempMonInterval <= 0 ){
			ROS_INFO(LOG_HEADER"temperature monitor timer disabled.");
			temp_mon_timer.stop();
		}else{
			ROS_INFO(LOG_HEADER"temperature monitor timer restart.");
			temp_mon_timer.stop();
			temp_mon_timer.setPeriod(ros::Duration(tempMonInterval));
			temp_mon_timer.start();
		}
	}else{
		ROS_WARN(LOG_HEADER"temperature monitor timer stop.");
		temp_mon_timer.stop();
	}
}

void on_camera_disconnect(){
	ROS_ERROR(LOG_HEADER"error: camera disconnected.");
	cam_params_refreshed=false;
	
	publish_bool(pub_stat,false);
	publish_string(pub_error,"YCAM disconected");
	
	temp_mon_timer.stop();
	ROS_WARN(LOG_HEADER"temp mon timer stop.");
}
	
void on_camera_closed(){
	ROS_INFO(LOG_HEADER"camera closed.");
	cam_params_refreshed=false;
	
	publish_bool(pub_stat,false);
	publish_string(pub_info,"YCAM closed");
}

	
sensor_msgs::Image to_diff_img(const sensor_msgs::Image &src_img,sensor_msgs::Image &dst_img){
	sensor_msgs::Image diff_img;
	if( src_img.data.empty() ||
		src_img.data.size() != dst_img.data.size()
	){
		ROS_ERROR(LOG_HEADER"error:camera image size is different");
	}else{
		diff_img = src_img;
		for ( int n=0; n < src_img.data.size(); n++ ) {
			int val = dst_img.data[n] - src_img.data[n];
			if (val < 1) { val = 0; }
			else if (val>255) { val=255; }
			
			diff_img.data[n] = val;
		}
	}
	
	return diff_img;
}


sensor_msgs::Image drawCameraOriginCross(sensor_msgs::Image &inputImg,cv::Point &posCross){
	const int width = inputImg.width;
	const int height = inputImg.height;
	if( inputImg.encoding.compare(sensor_msgs::image_encodings::MONO8) != 0){
		ROS_ERROR(LOG_HEADER"error:center cross draw failed. unsupport image encoding.");
		return inputImg;
	}
	
	cv::Mat grayImg = cv_bridge::toCvCopy(inputImg, sensor_msgs::image_encodings::MONO8)->image;
	
	cv_bridge::CvImage colorImg;
	cv::cvtColor(grayImg,colorImg.image, cv::COLOR_GRAY2RGB);
	
	const int size_gcd = std::min(width,height);
	const int cross_width = std::max(size_gcd / 150,1);
	const int cross_len = std::max(size_gcd/30,3);
	
	const int cx = posCross.x;//width /2;
	const int cy = posCross.y;//height/2;
	
	cv::line(colorImg.image, cv::Point(cx - cross_len/2  , cy) , cv::Point(cx + cross_len/2, cy), cv::Scalar(255,0,0),cross_width , cv::LINE_AA);
	cv::line(colorImg.image, cv::Point(cx, cy - cross_len/2),    cv::Point(cx, cy + cross_len/2)   , cv::Scalar(255,0,0),cross_width , cv::LINE_AA);
	
	sensor_msgs::Image outputImg;
	outputImg=*colorImg.toImageMsg();
	outputImg.header = inputImg.header;
	outputImg.encoding = sensor_msgs::image_encodings::RGB8;
	
	return outputImg;
}


void save_ros_img(sensor_msgs::Image &img,const std::string &path){
	cv::Mat grayImg = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8)->image;
	cv::imwrite(path,grayImg);
}

void on_capture_image_received(const bool result,const int elapsed, camera::ycam3d::CameraImage &img_l,const camera::ycam3d::CameraImage &img_r,const bool timeout,const int expsr_lv){
	
#ifdef DEBUG_STRESS_TEST
	ROS_INFO(LOG_HEADER"capture image recevie start. result=%s, timeout=%d img_l: result=%d size=%d x %d, img_r: result=%d size=%d x %d",
		(result?"OK":"NG"), timeout, img_l.result, img_l.width, img_l.height, img_r.result, img_r.width, img_r.height);
#endif
	if( timeout ){
		ROS_ERROR(LOG_HEADER"error:capture timeout occurred.");
		if(get_param<bool>(PRM_CAPT_TIMEOUT_RESET,false) ){
			ROS_WARN(LOG_HEADER"reset ycam3d.");
			g_node_exit_flg = 1;
		}
	}
	if( ! result ){
		ROS_ERROR(LOG_HEADER"error:capture failed.");
		return;
	}
	
	sensor_msgs::Image ros_img_l;
	img_l.to_ros_img(ros_img_l,FRAME_ID);
	
	sensor_msgs::Image ros_img_r;
	img_r.to_ros_img(ros_img_r,FRAME_ID);
	
	pub_img_raws[0].publish(ros_img_l);
	pub_img_raws[1].publish(ros_img_r);

#ifdef DEBUG_STRESS_TEST
	//debug *********
	save_ros_img(ros_img_l,"/tmp/left.pgm");
	save_ros_img(ros_img_r,"/tmp/right.pgm");
	
	const int brightAvg=std::accumulate(std::begin(ros_img_l.data), std::end(ros_img_l.data), 0.0) / ros_img_l.data.size();
	ROS_WARN(LOG_HEADER"[%6d] bright=%d, strobe=%d, expsr_lv=%d(%d)",debug_capt_watch_count++, brightAvg,debug_pre_strobe, expsr_lv,expsr_lv+1);
	if(! debug_pre_strobe ){
		if(expsr_lv== 3 && (brightAvg < 10 || 40 < brightAvg ) ){
			ROS_ERROR(LOG_HEADER"error:wrong image captured.");
			//exit(-1);
		}
	}else{
		if( expsr_lv== 0 && (brightAvg < 20 || 40 < brightAvg ) ){
			ROS_ERROR(LOG_HEADER"error:wrong image captured.");
			exit(-1);
		}else if( expsr_lv== 1 && (brightAvg < 40 || 70 < brightAvg ) ){
			ROS_ERROR(LOG_HEADER"error:wrong image captured.");
			exit(-1);
		}else if( expsr_lv== 2 && (brightAvg < 90 || 130 < brightAvg ) ){
			ROS_ERROR(LOG_HEADER"error:wrong image captured.");
			exit(-1);
		}else if( expsr_lv== 3 && (brightAvg < 140 || 210 < brightAvg ) ){
			ROS_ERROR(LOG_HEADER"error:wrong image captured.");
			exit(-1);
		}else if( expsr_lv== 4 && (brightAvg < 190 || 240 < brightAvg ) ){
			ROS_ERROR(LOG_HEADER"error:wrong image captured.");
			exit(-1);
		}else if( expsr_lv== 5 && (brightAvg < 190 || 255 < brightAvg ) ){
			ROS_ERROR(LOG_HEADER"error:wrong image captured.");
			exit(-1);
		}
	}
#endif
	
	const ros::Time now = ros::Time::now();
	
	ElapsedTimer tmr;
	//sensor_msgs::Image ros_imgs_darks[2];
	sensor_msgs::Image ros_imgs_brights[2];
		
	//const camera::ycam3d::CameraImage cam_imgs_darks[2] = { imgs_l.at(0), imgs_r.at(0) };
	
	const camera::ycam3d::CameraImage cam_imgs_brights[2] = { img_l, img_r };
	const bool drawCameraOriginCrossFlg =  get_param<bool>(PRM_DRAW_CAMERA_ORIGIN,false);
	for (int i = 0 ; i < 2 ; ++i ){
		cam_imgs_brights[i].to_ros_img(ros_imgs_brights[i],FRAME_ID);
		pub_img_raws[i].publish(ros_imgs_brights[i]);
		
		//cam_imgs_darks[i].to_ros_img(ros_imgs_darks[i],FRAME_ID);
		
		
		//remap
		sensor_msgs::Image remap_img_bright;
		sensor_msgs::Image remap_img_dark;
		{
			//remap-bright
			rovi::ImageFilter remap_img_filter;
			remap_img_filter.request.img = ros_imgs_brights[i];
			//ROS_INFO(LOG_HEADER"remap start. camno=%d",i);
			if( ! svc_remap[i].call(remap_img_filter) ){
				ROS_ERROR(LOG_HEADER"error:camera image remap failed. camno=%d",i);
			}else{
				remap_img_bright = remap_img_filter.response.img;
				
				//ROS_INFO(LOG_HEADER"remap end. camno=%d",i);
				
				if(drawCameraOriginCrossFlg && i == 0){
					std::vector<float> matK;
					if( ! nh->getParam(PRM_CAM_CALIB_MAT_K_LIST[i],matK)){
						ROS_ERROR(LOG_HEADER"error:camera calib matrix get failed.");
					}else if( matK.size() != 9 ){
						ROS_ERROR(LOG_HEADER"error:camera calib matrix K is wrong.");
					}
					
					cv::Point pos(matK[2],matK[5]);
					pub_rects[i].publish(drawCameraOriginCross(remap_img_bright,pos));
				}else{
					pub_rects[i].publish(remap_img_bright);
				}
				pub_rects1[i].publish(remap_img_bright);
			}
		}
		
//		{
//			//remap-dark
//			rovi::ImageFilter remap_img_filter;
//			remap_img_filter.request.img = ros_imgs_darks[i];
//			//ROS_INFO(LOG_HEADER"remap start. camno=%d",i);
//			if( ! svc_remap[i].call(remap_img_filter) ){
//				ROS_ERROR(LOG_HEADER"error:camera image remap failed. camno=%d",i);
//			}else{
//				//ROS_INFO(LOG_HEADER"remap end. camno=%d",i);
//				remap_img_dark = remap_img_filter.response.img;
//				pub_rects0[i].publish(remap_img_dark);
//			}
//		}
//		//diff
//		const sensor_msgs::Image diff_img = to_diff_img(remap_img_dark,remap_img_bright);
//		pub_diffs[i].publish(diff_img);
	}
}

void on_pattern_image_received(const bool result,const int proc_tm,const std::vector<camera::ycam3d::CameraImage> &imgs_l,const std::vector<camera::ycam3d::CameraImage> &imgs_r,const bool timeout,const int expsr_lv){
	ROS_INFO(LOG_HEADER"on pattern image recevied. result=%s, proc_tm=%d ms, timeout=%d, imgs_l: num=%d, imgs_r: num=%d",
		(result?"OK":"NG"), proc_tm, timeout, (int)imgs_l.size(),(int)imgs_r.size());
	
	ElapsedTimer tmr;
	tmr.restart();
	
	if( ! result ){
		ROS_ERROR(LOG_HEADER"error:pattern capture failed.");
	}else{
		//ptn_imgs_l = imgs_l;
		//ptn_imgs_r = imgs_r;
		ptn_imgs.push_back({imgs_l,imgs_r});
	}

	//ROS_INFO(LOG_HEADER"elapsed tm=%d",tmr.elapsed_ms());
	
	ptn_capt_wait_cv.notify_one();
	// ********** ptn_capt_wait_cv NOTIFY **********
#ifdef DEBUG_DETAIL
	ROS_INFO(LOG_HEADER"on pattern image recevied. finshed,proc_tm=%d ms",tmr.elapsed_ms());
#endif
	
	if( timeout ){
		ROS_ERROR(LOG_HEADER"error:capture timeout occurred.");
		publish_string(pub_error,"Image streaming timeout");
		if( get_param<bool>(PRM_CAPT_TIMEOUT_RESET,false) ){
			ROS_WARN(LOG_HEADER"reset ycam3d.");
			g_node_exit_flg = 1;
		}
	}
}

bool validate_patten_image_data(const PatternImageData &ptnImgData){
	if( ptnImgData.imgs_l.empty() ){
		ROS_ERROR(LOG_HEADER"error:pattern image left is empty.");
		return false;
		
	}else if( ptnImgData.imgs_r.empty() ){
		ROS_ERROR(LOG_HEADER"error:pattern image right is empty.");
		return false;
		
	}else if( ptnImgData.imgs_l.size() != ptnImgData.imgs_r.size() ){
		ROS_ERROR(LOG_HEADER"error:the number of pattern images is different.");
		return false;
	}
	
	return true;
}

bool exec_point_cloud_generation(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res){
	ROS_INFO(LOG_HEADER"exec_point_cloud_generation");
	ElapsedTimer tmr;

	if( ! pc_gen_mutex.try_lock_for(std::chrono::seconds(0)) ){
		res.success = false;
		ROS_WARN(LOG_HEADER"genpc is busy");
		return true;
	}
#ifdef DEBUG_STRESS_TEST
	ROS_WARN(LOG_HEADER"exec_point_cloud_generation wait end.");
#endif
	int mode=get_param<int>(PRM_MODE,(int)Mode_StandBy);
	//if( Mode_StandBy != mode){
	//	ROS_WARN(LOG_HEADER"standby wait. mode=%d",mode);
	//	usleep(500 * 1000);
	//}
	
	//ptn_imgs_l.clear();
	//ptn_imgs_r.clear();
	ptn_imgs.clear();
	
	ROS_INFO(LOG_HEADER"point cloud generation start.");
	bool result = false;
	std::lock_guard<std::timed_mutex> locker(pc_gen_mutex,std::adopt_lock);
	
	const PcGenMode pc_gen_mode = (PcGenMode)get_param<int>("pshift_genpc/calc/pcgen_mode",(int)PCGEN_GRAYPS4);
	
	ROS_INFO(LOG_HEADER"pcgen_mode=%d (%s)",pc_gen_mode,PCGEN_MODE_MAP[pc_gen_mode].c_str());
	
	std::stringstream  res_msg_str;
	/*
	if( pre_ycam_mode != Mode_StandBy){
		ROS_WARN(LOG_HEADER"mode is not standby");
		
	}else */if( ! camera_ptr ){
		ROS_ERROR(LOG_HEADER"camera is null");
		res_msg_str << "camera is null";
		
	}else if( ! camera_ptr->is_open() ){
		ROS_ERROR(LOG_HEADER"camera is not open.");
		res_msg_str << "camera is not open";
	} else if( cur_capt_params.empty()){
		ROS_ERROR(LOG_HEADER"camera capture setting is wrong.");
		res_msg_str << "camera capture setting is wrong";
	} else {
		
#ifdef DEBUG_STRESS_TEST
		ROS_WARN(LOG_HEADER"pattern capture start.");
#endif
		const int cur_mode = get_param<int>(PRM_MODE,(int)Mode_StandBy);
		bool ptn_capt_success=true;
		const int ptn_capt_num = cur_capt_params.size();
		
		rovi::GenPC genpc_msg;
		std::vector<RosPatternImageData> ros_ptn_imgs;
		genpc_msg.request.ptn_capt_num = ptn_capt_num;
		
		for( int n = 0 ; n < ptn_capt_num ; ++n ){
			tmr.start_lap();
			
			ROS_INFO(LOG_HEADER"<%d> pattern capture start. elapsed=%d ms",n,tmr.elapsed_ms());
			
			std::unique_lock<std::mutex> lock(ptn_capt_wait_mutex);

			const RosCaptureParameter capt_prm = cur_capt_params[n];
			if( ! camera_ptr->update_capture_param(capt_prm) ){
				ROS_ERROR(LOG_HEADER"<%d> error:capture parameter set failed. %s",n,capt_prm.to_string().c_str());
				ptn_capt_success=false;
				break;
			}else{
				ROS_INFO(LOG_HEADER"<%d> capture parameter updated. %s",n,capt_prm.to_string().c_str());
			}
			
			if( ! camera_ptr->capture_pattern( pc_gen_mode == PCGEN_MULTI , cur_mode == Mode_Streaming ) ){
				ROS_ERROR(LOG_HEADER"<%d> pattern catpture failed.",n);
				res_msg_str << "Capture failed";
				ptn_capt_success=false;
				break;
			}else{
				ptn_capt_wait_cv.wait(lock);
#ifdef DEBUG_DETAIL
				ROS_INFO(LOG_HEADER"<%d> ptn capt wait finished.",n);
#endif
				// ********** ptn_capt_wait_cv WAIT **********
				const PatternImageData *ptn_img=ptn_imgs.data() + n;
				
				//if( ptn_imgs_l.size() != ptn_imgs_l.size() ){
				if( ! validate_patten_image_data(*ptn_img) ){
					ptn_capt_success=false;
					ROS_ERROR(LOG_HEADER"<%d> pattern image num is different.",n);
					res_msg_str << "Failed to receive the image";
					break;
				}else{
					ROS_INFO(LOG_HEADER"<%d> pattern capture finished. proc_tm=%d ms",n,tmr.elapsed_lap_ms());
					tmr.start_lap();
					ROS_INFO(LOG_HEADER"<%d> pattern image convert start.", n);
						
					const int capt_num = ptn_img->imgs_l.size();
					std::vector<sensor_msgs::Image> ros_ptn_imgs_l;
					std::vector<sensor_msgs::Image> ros_ptn_imgs_r;
					
					ros_ptn_imgs_l.assign(capt_num,sensor_msgs::Image());
					ros_ptn_imgs_r.assign(capt_num,sensor_msgs::Image());
					
					bool all_recevied = true;
					for( int i = 0 ; i < capt_num ; ++i ){
						
						sensor_msgs::Image img_l;
						sensor_msgs::Image img_r;
						if( ! ptn_img->imgs_l.at(i).to_ros_img(img_l,FRAME_ID) ){
							ROS_ERROR(LOG_HEADER"left pattern image is invalid. no=%d",i);
							all_recevied=false;
							break;
							
						}else if( ! ptn_img->imgs_r.at(i).to_ros_img(img_r,FRAME_ID) ){
							ROS_ERROR(LOG_HEADER"right pattern image is invalid. no=%d",i);
							all_recevied=false;
							break;
						}
						
						ros_ptn_imgs_l[i] = img_l;
						genpc_msg.request.imgL.push_back(img_l);
						
						ros_ptn_imgs_r[i] = img_r;
						genpc_msg.request.imgR.push_back(img_r);
						
#ifdef DEBUG_PTN_IMG_SAVE
						char path[256];
						sprintf(path,"/tmp/%d_ptn_capt_%02d_0.pgm",n,i);
						save_ros_img(ros_ptn_imgs_l[i],path);
						
						sprintf(path,"/tmp/%d_ptn_capt_%02d_1.pgm",n,i);
						save_ros_img(ros_ptn_imgs_r[i],path);
#endif
					}
					if(  ! all_recevied ){
						ROS_ERROR(LOG_HEADER"<%d> pattern image convert failed. proc_tm=%d ms", n, tmr.elapsed_lap_ms());
						res_msg_str << "Failed to receive the image";
						ptn_capt_success=false;
						break;
					}else{
						ROS_INFO(LOG_HEADER"<%d> pattern image convert finished. proc_tm=%d ms", n, tmr.elapsed_lap_ms());
						ros_ptn_imgs.push_back({ros_ptn_imgs_l,ros_ptn_imgs_r});
					}
				}
			}
		}
		
		if( ! ptn_capt_success ){
			ROS_ERROR(LOG_HEADER"error: pattern capture failed.");
		}else{
			ROS_INFO(LOG_HEADER"all pattern capture completed. elapsed=%d ms",tmr.elapsed_ms());
			
			if( ! svc_genpc.call(genpc_msg) ){
				ROS_ERROR(LOG_HEADER"genpc exec failed. elapsed=%d ms", tmr.elapsed_ms());
				res_msg_str << "Failed to generate point cloud.";
				
			}else{
				
				if(ptn_capt_num > 1){
					res_msg_str << "capture " << ptn_capt_num << " times. ";
				}
				int ptnImgNum = 0;
				for(const PatternImageData &ptnImgData:ptn_imgs){
					ptnImgNum += ptnImgData.imgs_l.size();
				}
				res_msg_str << ptnImgNum << " images scan complete.";
				
				if( genpc_msg.response.pc_cnt_r >= 0 ){
					res_msg_str << " Generated PointCloud Count. Left=" << genpc_msg.response.pc_cnt << " Right=" << genpc_msg.response.pc_cnt_r;
				}else{
					res_msg_str << " Generated PointCloud Count=" << genpc_msg.response.pc_cnt;
				}
				result=true;
			}

			for( int n = 0 ; n < ros_ptn_imgs.size() ; ++n ){
				const RosPatternImageData *ros_ptn_img = ros_ptn_imgs.data() + n;
				const std::vector<sensor_msgs::Image> *ros_imgs=&(ros_ptn_img->imgs[0]);
				
				if(cur_capt_params.size() > n){
					if( ! cur_capt_params[n].pcgen_publish ){
						ROS_INFO(LOG_HEADER"<%d> pcgen publish is skipped.",n);
						continue;
					}
				}
				ElapsedTimer tmr_pcgen_publish;
				
				ROS_INFO(LOG_HEADER"<%d> pcgen publish start.",n);
				//画像配信
				for( int camno = 0 ; camno < 2 ; ++camno ){
					pub_img_raws[camno].publish(ros_imgs[camno][1]);
					sensor_msgs::Image remap_ros_img_ptn_0;
					{
						rovi::ImageFilter remap_img_filter;
						remap_img_filter.request.img = ros_imgs[camno][0];
						if( ! svc_remap[camno].call(remap_img_filter) ){
							ROS_ERROR(LOG_HEADER"<%d> error:camera image remap failed. camno=%d, ptn=0",n,camno);
						}else{
							remap_ros_img_ptn_0 = remap_img_filter.response.img;
						}
						pub_rects0[camno].publish(remap_ros_img_ptn_0);
						pub_rects[camno].publish(remap_ros_img_ptn_0);
					}
					
					sensor_msgs::Image remap_ros_img_ptn_1;
					{
						rovi::ImageFilter remap_img_filter;
						remap_img_filter.request.img = ros_imgs[camno][1];
						
						if( ! svc_remap[camno].call(remap_img_filter) ){
							ROS_ERROR(LOG_HEADER"<%d> error:camera image remap failed. camno=%d, ptn=1",n,camno);
						}else{
							remap_ros_img_ptn_1 = remap_img_filter.response.img;
						}
						pub_rects1[camno].publish(remap_ros_img_ptn_1);
						pub_rects[camno].publish(remap_ros_img_ptn_1);
					}
					{
						sensor_msgs::Image diff_img = to_diff_img(remap_ros_img_ptn_0,remap_ros_img_ptn_1);
						pub_diffs[camno].publish(diff_img);
					}
				}
				ROS_INFO(LOG_HEADER"<%d> pcgen publish finished. proc_tm=%d",n,tmr_pcgen_publish.elapsed_ms());
			}
		}
	}
	
	publish_bool(pub_Y1,result);
	
	res.success = result;
	res.message = res_msg_str.str();
	
	ROS_INFO(LOG_HEADER"point cloud generation finished. result=%d tmr=%d ms", result,  tmr.elapsed_ms());

	if( cur_capt_params.size() > 1 ){
		RosCaptureParameter rb_capt_param = cur_capt_params.front();
		ROS_INFO(LOG_HEADER"capture parameter rollback start. %s",rb_capt_param.to_string().c_str());
		if( ! camera_ptr->update_capture_param(rb_capt_param) ){
			ROS_ERROR(LOG_HEADER"error: capture parameter rollback failed.");
		}else{
			ROS_INFO(LOG_HEADER"capture parameter rollback finished.");
		}
	}
	
	// ********** pc_gen_mutex UNLOCKED **********

	return true;
}


void sub_exec_point_cloud_generation(const std_msgs::Bool::ConstPtr &req){
	if( ! camera_ptr || ! camera_ptr->is_open() ){
		ROS_ERROR(LOG_HEADER"camera is not open");
		publish_bool(pub_Y1,false);
		return;
	}
	std_srvs::TriggerRequest trig_req;
	std_srvs::TriggerResponse trig_res;
	const bool ret=exec_point_cloud_generation(trig_req,trig_res);
	ROS_INFO(LOG_HEADER"<subscribe> point cloud generation finished. ret=%d",ret);
}

void exec_get_ycam_temperature(){
	if( ! camera_ptr || ! camera_ptr->is_open() ){
		ROS_ERROR(LOG_HEADER"camera is not open");
		publish_bool(pub_Y1,false);
		return;
	//}else if( camera_ptr->is_busy() ){
	//	ROS_WARN(LOG_HEADER"temperature get skipped. camera is busy.!!!!");
	//	return;
	}
	
	float tempF=NAN;
	int temp=0;
	if( ! camera_ptr->get_temperature(&temp) ){
		if( temp_acq_failure_count > TEMP_ACQ_FAILURE_MSG_REPEAT_MAX ){
			//skip
		}else{
			if( temp_acq_failure_count < TEMP_ACQ_FAILURE_MSG_REPEAT_MAX ){
				ROS_ERROR(LOG_HEADER"Could not get the temperature.");
			}else{
				ROS_ERROR(LOG_HEADER"Stops the output of the temperature acquisition failure message.");
			}
		}
		temp_acq_failure_count++;
	}else{
		temp_acq_failure_count = 0;
		tempF = temp;
	}
	
	publish_float32(pub_temperature, tempF);
	//ROS_INFO(LOG_HEADER"<subscribe> ycam temperature . temperature=%4.1f",tempF);
}
	
void sub_get_ycam_temperature(const std_msgs::Bool::ConstPtr &req){
	if( ! req->data ){
		return;
	}
	
	exec_get_ycam_temperature();
}

void get_ycam_temperature_task(const ros::TimerEvent& e){
	exec_get_ycam_temperature();
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
	
	//camera initialization
	ROS_INFO(LOG_HEADER"camera: size= %d x %d, resolution=%s", cam_width, cam_height, camera_res.c_str());
	ROS_INFO(LOG_HEADER"ycam3d: mode=%d, cycle=%d Hz", pre_ycam_mode, cur_mode_mon_cyc);
	
	camera_ptr.reset(new CameraYCAM3D());
	
	if( ! camera_ptr->init(camera_res) ){
		ROS_ERROR(LOG_HEADER"error: camera initialization failed.");
		return 1;
	}
	
	camera_ptr->set_callback_camera_open_finished(on_camera_open_finished);
	camera_ptr->set_callback_camera_disconnect(on_camera_disconnect);
	camera_ptr->set_callback_camera_closed(on_camera_closed);
	camera_ptr->set_callback_capture_img_received(on_capture_image_received);
	camera_ptr->set_callback_pattern_img_received(on_pattern_image_received);
	
	//publishers
	pub_img_raws[0] = n.advertise<sensor_msgs::Image>("left/image_raw", 1);
	pub_img_raws[1] = n.advertise<sensor_msgs::Image>("right/image_raw", 1);
	pub_Y1 = n.advertise<std_msgs::Bool>("Y1", 1);
	//pub_pcount = n.advertise<std_msgs::Int32>("pcount", 1);
	pub_stat = n.advertise<std_msgs::Bool>("stat", 1);
	pub_error = n.advertise<std_msgs::String>("error", 1);
	pub_info  = n.advertise<std_msgs::String>("message", 1);
	
	pub_rects[0] = n.advertise<sensor_msgs::Image>("left/image_rect", 1);
	pub_rects0[0] = n.advertise<sensor_msgs::Image>("left/image_rect0", 1);
	pub_rects1[0] = n.advertise<sensor_msgs::Image>("left/image_rect1", 1);
	pub_diffs[0] = n.advertise<sensor_msgs::Image>("left/diff_rect", 1);
	//pub_views[0] = n.advertise<sensor_msgs::Image>("left/view",1);
	
	pub_rects[1] = n.advertise<sensor_msgs::Image>("right/image_rect", 1);
	pub_rects0[1] = n.advertise<sensor_msgs::Image>("right/image_rect0", 1);
	pub_rects1[1] = n.advertise<sensor_msgs::Image>("right/image_rect1", 1);
	pub_diffs[1] = n.advertise<sensor_msgs::Image>("right/diff_rect", 1);
	//pub_views[1] = n.advertise<sensor_msgs::Image>("right/view",1);
	
	pub_temperature = n.advertise<std_msgs::Float32>("ycam/temperature",1);
	
	//service servers
	const ros::ServiceServer svs1 = n.advertiseService("pshift_genpc", exec_point_cloud_generation);
	
	//service clients
	svc_genpc = n.serviceClient<rovi::GenPC>("genpc");
	svc_remap[0] = n.serviceClient<rovi::ImageFilter>("left/remap");
	svc_remap[1] = n.serviceClient<rovi::ImageFilter>("right/remap");
	
	//subscribers
	const ros::Subscriber sub1 = n.subscribe<std_msgs::Bool>("X1", 1, sub_exec_point_cloud_generation);
	const ros::Subscriber sub2 = n.subscribe<std_msgs::Bool>("ycam/get_temperature", 1, sub_get_ycam_temperature);
	
	//timers
	mode_mon_timer = n.createTimer(ros::Duration(1/(float)cur_mode_mon_cyc), mode_monitor_task);
	cam_open_mon_timer = n.createTimer(ros::Duration(1), cam_open_monitor_task);
	temp_mon_timer = n.createTimer(ros::Duration(TEMP_MON_INTERVAL_DEFAULT), get_ycam_temperature_task);
	temp_mon_timer.stop();

	camera_ptr->set_callback_ros_error_published([](const std::string message){
		publish_string(pub_error,message);
	});
	camera_ptr->set_callback_auto_con_limit_exceeded([](){
		publish_string(pub_error,"camera auto connect limit exceeded.");
		g_node_exit_flg = 1;
	});
	camera_ptr->start_auto_connect(cam_ipaddr);
	
	bool activeDelyMonitor=false;
	if( get_param<bool>(PRM_NW_DELAY_MON_ENABLED,false) ){
		const int delayMonInterval= get_param<int>(PRM_NW_DELAY_MON_INTERVAL,1);
		const int delayMonTimeout= get_param<int>(PRM_NW_DELAY_MON_TIMEOUT,500);
		const bool delayMonIgnUpdFail = get_param<bool>(PRM_NW_DELAY_MON_IGN_UPD_FAIL,false);
		ROS_INFO("delay monistor start. interval=%d",delayMonInterval);
		camera_ptr->start_nw_delay_monitor_task(delayMonInterval,delayMonTimeout,[](){
			ROS_ERROR(LOG_HEADER"network delay occurred !!!");
			publish_string(pub_error,"network delay occurred.");
			g_node_exit_flg = 1;
		},delayMonIgnUpdFail);
		activeDelyMonitor = true;
	}
	
	ros::Duration interval(0.01);
	//ros::spin();
	while( ros::ok() && g_node_exit_flg == 0 ){
		ros::spinOnce();
		interval.sleep();
	}
	
	
	if( activeDelyMonitor ){
		printf("delay monistor stop.\n");
		camera_ptr->stop_nw_delay_monitor_task();
	}
	
	mode_mon_timer.stop();
	printf(LOG_HEADER"mode monitor timer stopped.\n");
	
	cam_open_mon_timer.stop();
	printf(LOG_HEADER"camera open monitor timer stopped.\n");
	
	//todo:******画像転送スレッドの終了待ち
	printf(LOG_HEADER"image transfer task wait start.\n");

	printf(LOG_HEADER"image transfer task wait finished.\n");
		
	if( camera_ptr ){
		printf(LOG_HEADER"camera terminate start.\n");
		camera_ptr.reset();
		printf(LOG_HEADER"camera terminate finished.\n");
	}
	return 0;
}
