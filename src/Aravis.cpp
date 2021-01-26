#include <stdlib.h>
#include <signal.h>
#include <stdio.h>
#include <ctype.h>
#include <stdarg.h>
#include <syslog.h>
#include <string.h>
#include <errno.h>
#include <string>
#include <vector>
#include <fstream>
#include <math.h>

#include "Aravis.h"

using namespace std;

typedef std::vector<std::string> STRLIST;

#define Sleep(ms) usleep(ms*1e3)

#include "ElapsedTimer.hpp"

//2020/09/25 add by hato -------------------- start --------------------
//#define DEBUG_DETAIL
//2020/09/25 add by hato --------------------  end  --------------------

//2020/10/21 add by hato -------------------- start --------------------
namespace aravis{
namespace ycam3d{
	const ExposureTimeLevelSetting EXPOSURE_TIME_LV_SETTING_SXGA = 
		{ 0, 5, 1, 
			{
				{  4000,  8333, 60 },
				{  8000,  8333, 60 },
				{ 16000, 16000, 60 },
				{ 24000, 24000, 40 },
				{ 32000, 32000, 30 },
				{ 49000, 50000, 20 },
			}
		};
	
	const ExposureTimeLevelSetting EXPOSURE_TIME_LV_SETTING_VGA  = 
		{ 0, 5, 1, 
			{
				{  4000,  8333, 116 },
				{  8000,  8333, 116 },
				{ 16000, 16000,  60 },
				{ 24000, 24000,  40 },
				{ 32000, 32000,  30 },
				{ 49000, 50000,  20 },
			}
		};
}
}

using namespace aravis::ycam3d;
//2020/10/21 add by hato --------------------  end  --------------------

namespace{
	//2020/09/11 add by hato -------------------- start --------------------
	bool msleep(const int msec){
		usleep(msec*1000);
		return true;
	}
	//2020/09/11 add by hato --------------------  end  --------------------
	
	//2020/10/21 add by hato -------------------- start --------------------
	//const int PROJ_TRIGGER_MODE_WAIT = 50;
	//const int PROJ_EXPOSURE_TIME_WAIT = 50;
	const int PROJ_INTENSITY_WAIT = 0;
	const int PROJ_FLASH_INTERVAL_WAIT = 100;
	//const int PROJ_PTN_LOAD_WAIT = 500;
	//2020/10/21 add by hato --------------------  end  --------------------
	
	//2020/11/30 add by hato -------------------- start --------------------
	const int PROJ_STOP_GO_WAIT_TM_SHORT = 400;
	const int PROJ_STOP_GO_WAIT_TM_NORMAL = 400;
	//2020/11/30 add by hato --------------------  end  --------------------
	
	void dprintf(const char *fmt,...){
		va_list args, args2;
		va_start(args, fmt);
		{
			va_copy(args2, args);
		    vsyslog(LOG_INFO, fmt, args2);
			va_end(args2);
		}
	    vfprintf(stderr, fmt, args);
	    fprintf(stderr,"\n");
		va_end(args);
	}

	STRLIST split_str(const string &str, const string &delim) {
		STRLIST result;
		size_t cutAt;
		string tmp(str);
		while ((cutAt = tmp.find_first_of(delim)) != tmp.npos) {
			if (cutAt > 0) {
				result.push_back(tmp.substr(0, cutAt));
			}
			tmp = tmp.substr(cutAt + 1);
		}
		if (tmp.length() > 0) {
			result.push_back(tmp);
		}
		return result;
	}
	//文字列の置換
	void replace_str(string &replacedStr, const string &from, const string &to, bool all = false) {
		size_t pos = replacedStr.find(from);
		const size_t toLen = to.length();
		if (from.empty()) {
			return;
		}
		while ((pos = replacedStr.find(from, pos)) != std::string::npos) {
			replacedStr.replace(pos, from.length(), to);
			pos += toLen;
			if (!all) break;
		}
	}
	//ノード名
	string node_name(const char *tmpl, const char *res, int cam = -1, int idx = -1)
	{
		char buf[512];
		if (cam < 0) snprintf(buf, sizeof(buf), tmpl, res);
		else if (idx < 0) snprintf(buf, sizeof(buf), tmpl, res, cam);
		else snprintf(buf, sizeof(buf), tmpl, res, cam, idx);
		return buf;
	}
}

int Aravis::static_camno_ = 0;

//ncam:ポートを共有するカメラ数(GevSCPD計算用)
Aravis::Aravis(YCAM_RES res, int ncam):resolution_(res),ncam_(ncam)
	//2020/09/18 add by hato -------------------- start --------------------
	,m_expsr_tm_lv_setting_(nullptr),m_expsr_tm_lv(-1)
	//2020/09/18 add by hato --------------------  end --------------------
	//2020/11/05 add by hato -------------------- start --------------------
	,cur_proj_ptn_(YCAM_PROJ_PTN_PHSFT)
	//,cur_proj_enabled_(Proj_Disabled)
	,cur_proj_intensity_(0)
	//2020/11/05 add by hato --------------------  end  --------------------
{
	camno_ = static_camno_++;
	camera_ = nullptr;
	stream_ = nullptr;
	on_image_ = nullptr;
	out_ = nullptr;
	on_lost_ = nullptr;
	payload_=width_=height_=color_=0;
	
	//2020/11/09 comment by hato -------------------- start --------------------
	//trigger_mode_ = YCAM_TRIG_INT;
	//2020/11/09 comment by hato --------------------  end  --------------------
	
	//2020/09/18 add by hato -------------------- start --------------------
	//if (res==YCAM_RES_VGA) {width_=1280,height_=480;}
	//else if (res==YCAM_RES_SXGA) {width_=2560,height_=1024;}
	if (res==YCAM_RES_VGA) {
		width_=1280;
		height_=480;
		m_expsr_tm_lv_setting_ = &EXPOSURE_TIME_LV_SETTING_VGA;
	}else if (res==YCAM_RES_SXGA){
		width_=2560,
		height_=1024;
		m_expsr_tm_lv_setting_ = &EXPOSURE_TIME_LV_SETTING_SXGA;
	}
	//2020/09/18 add by hato --------------------  end  --------------------
	else dprintf("invalid resolution type:%d",res);
	packet_delay_=0;
	lost_=true;
	buffer_status_=ARV_BUFFER_STATUS_SUCCESS;
	//
	pthread_condattr_t attr;
	pthread_condattr_init(&attr);
	pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
	pthread_cond_init(&cap_cond_, &attr);
	//
	pthread_mutex_init(&cap_mutex_, NULL);
	
}

Aravis::~Aravis()
{
	destroy();
}

bool Aravis::openCamera(const char *name, const int packet_size)
{
	*name_='\0';
	dprintf("*** open camera [%s]",name ? name : "");
	
	//2020/09/18 add by hato -------------------- start --------------------
	m_expsr_tm_lv = -1;
	if( ! m_expsr_tm_lv_setting_ ){
		dprintf("error: exposure time level setting is null.");
		return false;
	}
	const int cur_expsr_tm_lv = m_expsr_tm_lv_setting_->default_lv;
	const ExposureTimeLevelSetting::Param * exp_tm_lv_param = m_expsr_tm_lv_setting_->get_param(cur_expsr_tm_lv);
	if( ! exp_tm_lv_param ){
		dprintf("error: exposure time level param is null.");
		return false;
	}
	//2020/09/18 add by hato --------------------  end  --------------------
	
	//
	ArvInterface *arvif=arv_gv_interface_get_instance();
	arv_interface_update_device_list(arvif);
	const int ndev=arv_interface_get_n_devices(arvif);
	if(ndev<=0){
		dprintf(" error: no available cameras");
		return false;
	}
	dprintf(" number of available device = %d",ndev);
	dprintf(" --------------------");
	//
	const char *camera_name=name;
	int ndot=0;
	if(name){
		snprintf(name_,sizeof(name_),"%s",name);
		for(const char *p=name; *p; ++p){
			if(isdigit(*p))continue;
			if(*p=='.') {++ndot;continue;}
			ndot=0;
			break;
		}
	}
	const bool ip= (ndot==3);
	if(ip){
		for(int i=0;i<ndev;++i){
			dprintf("#%d",i);
			const char *ip_addr=arv_interface_get_device_address (arvif, i);
			const char *device_id=arv_interface_get_device_id(arvif,i);
			dprintf(" ip address=%s",ip_addr);
			dprintf(" device id=%s",device_id);
			if(strcmp(name,ip_addr)==0){
				dprintf(" --------------------");
				camera_name=device_id;
				break;
			}
		}
		dprintf(" open device [%s]",camera_name);
	}
	camera_ = arv_camera_new(camera_name);
	if(camera_){
		
		device_=arv_camera_get_device(camera_);
		
		//2020/09/25 add by hato -------------------- start --------------------
		uart_flush();
		//2020/09/25 add by hato --------------------  end  --------------------
		
		//inital values...
		set_node_int("Width", width_);
		set_node_int("Height", height_);
//		reg_write(REG_PWM_FRAME_RATE, resolution_==YCAM_RES_SXGA ? 60 : 110);
		//
		int x,y;
		arv_camera_get_region(camera_, &x, &y, &width_, &height_);
		dprintf(" vendor name  = %s", arv_camera_get_vendor_name(camera_));
		dprintf(" model name   = %s", arv_camera_get_model_name(camera_));
		dprintf(" device id    = %s", arv_camera_get_device_id(camera_));
		dprintf(" region       = %d %d %d %d", x, y, width_, height_);
		dprintf(" pixel format = %s", arv_camera_get_pixel_format_as_string(camera_));
		//帯域計算
		if(packet_size){
			arv_camera_gv_set_packet_size(camera_, packet_size);	//GevSCPSPacketSize
		}
		else{
			//2020/09/11 modified by hato -------------------- start --------------------
			//arv_camera_gv_auto_packet_size(camera_);	//relies on GevSCPSFireTestPacket
			arv_camera_gv_set_packet_size(camera_,8192);
			//2020/09/11 modified by hato --------------------  end  --------------------
		}
		const int pktsz = arv_camera_gv_get_packet_size(camera_);
		dprintf(" packet_size  = %d",pktsz);
		
		//2020/09/14 add by hato -------------------- start --------------------
		dprintf(" --------------------");
		#if 0
		arv_device_set_integer_feature_value(device_,"TriggerMode",1);
		arv_device_set_integer_feature_value(device_,"Height",height);
		arv_device_set_integer_feature_value(device_,"Width",width);
		set_ycam(exposure_time,8300);
		set_ycam(acquisition_fps,width==2560 ? 60: 110);
		arv_camera_gv_set_packet_size(gCamera,8192);
		pset_TrigMode(0);        usleep(500000);
		pset_ExposureTime(8333); usleep(500000);
	 	pset_Intensity(100);     usleep(500000);
	 	pset_Interval(10);       usleep(500000);
	   	pset_PatternLoad(1);     sleep(3);
		pset_TrigMode(1);        usleep(500000);
		#endif
		
#if 0
		setExposureTime(16666);
		std::vector<int> frameRates;
		for(int i=1;i<120;++i){
			if(set_node_int("AcquisitionFrameRate",i)){
				frameRates.push_back(i);
			}
		}
		dprintf("------------------");
		for(const int frameRate:frameRates){
			dprintf("[%03d]",frameRate);
		}
		dprintf("------------------");
#endif
		
		{
			const bool reult_cam_expsr_tm = reg_write(REG_EXPOSURE_TIME,exp_tm_lv_param->cam_exposure_tm);
			dprintf(" setup camera exposure time.    result=%s, set_val=%5d, cur_val=%5d",
				(reult_cam_expsr_tm?"OK":"NG"), exp_tm_lv_param->cam_exposure_tm, exposureTime());
		}
		{
			const bool result_frame_rate = set_node_int("AcquisitionFrameRate", exp_tm_lv_param->cam_frame_rate);
			dprintf(" setup camera frame rate.       result=%s, set_val=%5d, cur_val=%5d",
				(result_frame_rate?"OK":"NG"), exp_tm_lv_param->cam_frame_rate, get_node_int("AcquisitionFrameRate"));
		}
		{
			
			const bool result_gain_d = setGainD(CAM_DIGITAL_GAIN_DEFAULT);
			dprintf(" setup camera digital gain.     result=%s, set_val=%5d, cur_val=%5d",
				(result_gain_d?"OK":"NG"),CAM_DIGITAL_GAIN_DEFAULT, gainD());
		}
		{
			const bool result_gain_a = setGainA(CAM_ANALOG_GAIN_DEFAULT);
			dprintf(" setup camera analog gain.      result=%s, set_val=%5d, cur_val=%5d",
				(result_gain_a?"OK":"NG"),CAM_ANALOG_GAIN_DEFAULT, gainA());
		}
		//2020/11/05 comment out by hato -------------------- start --------------------
		/*
		{
			const bool result_tig_mode = setTriggerMode(YCAM_TRIG_INT);
			dprintf(" setup trigger mode.            result=%s, set_val=%5d, cur_val=%5d",
				(result_tig_mode?"OK":"NG"),YCAM_TRIG_INT, triggerMode());
			msleep(PROJ_TRIGGER_MODE_WAIT);
		}*/
		//2020/11/05 comment out by hato --------------------  end --------------------
		{
			const bool result_proj_expsr_tm = setProjectorExposureTime(exp_tm_lv_param->proj_exposure_tm);
			dprintf(" setup projector exposure time. result=%s, set_val=%5d",
				"--",exp_tm_lv_param->proj_exposure_tm);
			
			//msleep(PROJ_EXPOSURE_TIME_WAIT);
		}
		{
			const bool result_proj_intensity =  setProjectorIntensity(PROJ_INTENSITY_DEFAULT);
			dprintf(" setup projector intensity.     result=%s, set_val=%5d",
				"--",PROJ_INTENSITY_DEFAULT);
			//msleep(PROJ_INTENSITY_WAIT);
		}
		{
			const bool result_proj_flash_interval= setProjectorFlashInterval(PROJ_FLASH_INTERVAL_DEFAULT);
			dprintf(" setup projector interval.      result=%s, set_val=%5d",
				(result_proj_flash_interval?"OK":"NG"),PROJ_FLASH_INTERVAL_DEFAULT);
			//msleep(PROJ_FLASH_INTERVAL_WAIT);
		}
		{
			const bool result_proj_ptn = setProjectorPattern(YCAM_PROJ_PTN_PHSFT,false);
			dprintf(" setup projector pattern.       result=%s, set_val=%5d",
				(result_proj_ptn?"OK":"NG"),YCAM_PROJ_PTN_PHSFT);
			//msleep(PROJ_PTN_LOAD_WAIT);
		}
		//2020/11/05 comment out by hato -------------------- start --------------------
		/*
		{
			const bool result_trig_mode = setTriggerMode(YCAM_TRIG_EXT);
			dprintf(" setup trigger mode.            result=%s, set_val=%5d, cur_val=%5d",
				(result_trig_mode?"OK":"NG"),YCAM_TRIG_EXT,triggerMode());
			msleep(PROJ_TRIGGER_MODE_WAIT);
		}*/
		//2020/11/05 comment out by hato -------------------- start --------------------
		m_expsr_tm_lv = cur_expsr_tm_lv;
		//dprintf(" current exposure time lv =%d",m_expsr_tm_lv);
		//2020/09/14 add by hato --------------------  end  --------------------
		
		dprintf(" --------------------");
		if(1 < ncam_){
			const double trans_time_min = pktsz / (100.0 * 1024.0 * 1024.0);	//最短転送時間 1Gbps=100MB/s(10bits/byte)
			const double trans_time_exp=trans_time_min*ncam_;	//期待転送時間
			double delay_time=trans_time_exp-trans_time_min;
			delay_time*=1e9;	//us
			dprintf(" packet_delay -> %ld",(int64_t)delay_time);
			arv_camera_gv_set_packet_delay(camera_, (int64_t)delay_time);
			packet_delay_=(int64_t)delay_time;
		}
		{
			IPADDR v;
			dprintf(" YCAM3D version    = 0x%0X", reg_read(REG_YCAM_VERSION));
			v.a = reg_read(REG_IP_ADDRESS);
			dprintf(" ip address        = %d.%d.%d.%d", v.b[3], v.b[2], v.b[1], v.b[0]);
			v.a = reg_read(REG_FW_VERSION);
			version_ = v.b[1] * 10 + v.b[0];
			//2020/12/14 modified by hato -------------------- start --------------------
			//dprintf(" firmware          = MicroBlaze[%d.%d] FPGA[%d.%d]", v.b[3], v.b[2], v.b[1], v.b[0]);
			dprintf(" firmware          = MicroBlaze[%X.%X] FPGA[%X.%X]", v.b[3], v.b[2], v.b[1], v.b[0]);
			//2020/12/14 modified by hato --------------------  end  --------------------
			//2020/09/25 add by hato -------------------- start --------------------
			dprintf(" high_speed_version= %s", (isAsync()?"yes":"no"));
			//2020/09/25 add by hato --------------------  end  --------------------
			dprintf(" transfer mode     = %d", reg_read(REG_TRANSFER_MODE));
			dprintf(" pixel clock delay = %d", reg_read(REG_CLOCK_DELAY));
			dprintf(" exposure time     = %d", reg_read(REG_EXPOSURE_TIME));
			dprintf(" analog gain       = %d", reg_read(REG_ANALOG_GAIN));
			dprintf(" ditigal gain      = %d", reg_read(REG_DIGITAL_GAIN));
			dprintf(" YCAM3D SERIAL     = [%s]", get_description("YCam_Serial_No").c_str());
			dprintf(" --------------------");
		}
		//2020/11/09 comment by hato -------------------- start --------------------
		//setTriggerMode(YCAM_TRIG_EXT);
		//2020/11/09 comment by hato --------------------  end  --------------------
		
		//2020/09/25 add by hato -------------------- start --------------------
		uart_flush();
		//2020/09/25 add by hato --------------------  end  --------------------
	}
	else dprintf("error: arv_camera_new");
	lost_ = camera_ ? false : true;
	
	//2020/11/09 add by hato -------------------- start --------------------
	//高速撮影版しか対応しない。
	if( ! isAsync() ){
		lost_ = true;
		dprintf("error: only high speed version supported.");
	}
	//2020/11/09 add by hato --------------------  end  --------------------
	return !lost_;
}

//tmout: パケットタイムアウト(ms)
//nbuf: バッファ数
bool Aravis::openStream(int nbuf)
{
	if(!camera_){
		dprintf("error: camera [%s] not opened",name_);
		return false;
	}
	
	stream_ = arv_camera_create_stream(camera_, NULL, NULL);
	if (!stream_){
		dprintf("error: arv_camera_create_stream [%s]",name_);
		return false;
	}
	if (ARV_IS_GV_STREAM(stream_)) {
		//Packet timeout, in µs.
		//Allowed values: [1000,10000000]
		//Default value: 40000
		g_object_set(stream_, "packet-timeout", (unsigned)(1 * 1e3), NULL);
		g_object_set(stream_, "frame-retention", (unsigned)(200 * 1e3), NULL);
	}
	payload_ = arv_camera_get_payload(camera_);
	//2020/09/14 comment out by hato -------------------- start --------------------
	//dprintf(" create %d buffers...",nbuf);
	//2020/09/14 comment out by hato --------------------  end  --------------------
	for (int i = 0; i < nbuf; i++){
		arv_stream_push_buffer(stream_, arv_buffer_new(payload_, NULL));
	}
//	arv_camera_set_acquisition_mode(camera_, ARV_ACQUISITION_MODE_CONTINUOUS);
//	arv_camera_set_trigger(camera_, "Software");
//	arv_camera_start_acquisition(camera_);
	arv_device_execute_command(device_,"AcquisitionStart");

	Sleep(1000);
	g_signal_connect(stream_, "new-buffer", G_CALLBACK(on_new_buffer), this);
	arv_stream_set_emit_signals(stream_, true);
	g_signal_connect(device_, "control-lost", G_CALLBACK(on_control_lost), this);

	return true;
}

void Aravis::destroy()
{
	dprintf("close camera [%s]",name_);
	if(stream_){
		arv_stream_set_emit_signals(stream_, FALSE);
		g_object_unref (stream_);
		stream_=0;
	}
	if(camera_){
//		arv_camera_stop_acquisition(camera_);
		arv_device_execute_command(device_, "AcquisitionStop");
		g_object_unref(camera_);
		camera_=0;
	}
	lost_=true;
}

const long UMAX=1000000000L;

bool Aravis::capture(unsigned char *data, float timeout_sec)
{
	if(lost_)return false;
	frame_index_ = 0;
	pthread_mutex_lock(&cap_mutex_);
	out_ = data;
//	arv_camera_software_trigger(camera_);
//2020/11/09 comment out by hato -------------------- start --------------------
//	if (isAsync()){
//2020/11/09 comment out by hato --------------------  end  --------------------
//2020/09/25 modified by hato -------------------- start --------------------
		//uart_write('a', 0);	//プロジェクターOFF
		//usleep(100);
		uart_cmd( 'a' , 0 );
//2020/09/25 modified by hato --------------------  end  --------------------
		reg_write(REG_STREAM_NUM, 1);
		
//2020/09/25 add by hato -------------------- start --------------------
		//uart_cmd( 'a' , 1 );
//2020/09/25 add by hato --------------------  end --------------------
//2020/11/09 comment out by hato -------------------- start --------------------
/*
	}
	else{
//2020/09/25 modified by hato -------------------- start --------------------
		//uart_write('o', -1);
		uart_cmd( 'o' , -1);
//2020/09/25 modified by hato --------------------  end  --------------------
	}
*/
//2020/11/09 comment by hato --------------------  end  --------------------
	if(0.0f < timeout_sec){
		struct timespec abs_time;
		time_t s=(time_t)timeout_sec;
		long u=(long)((timeout_sec-(float)s)*UMAX);
//		dprintf("%10ld.%09ld",s,u);
		clock_gettime(CLOCK_MONOTONIC, &abs_time);
		abs_time.tv_sec += s;
		abs_time.tv_nsec += u;
		if(abs_time.tv_nsec>=UMAX){
			abs_time.tv_nsec-=UMAX;
			++abs_time.tv_sec;
		}
		int ret=pthread_cond_timedwait(&cap_cond_, &cap_mutex_, &abs_time);
		for(;;){
			if(ret==0)break;
			if(ret==ETIMEDOUT){
				dprintf("[%s] wait timed out",name_);
				buffer_status_=ARV_BUFFER_STATUS_TIMEOUT;
			}
			else{
				dprintf("error: [%s] pthread_cond_timedwait [%d]", name_,ret);
				buffer_status_=ARV_BUFFER_STATUS_UNKNOWN;
			}
//			lost_=true;
			break;
		}
	}
	else{
		pthread_cond_wait(&cap_cond_, &cap_mutex_);
	}
	bool ret=(buffer_status_==ARV_BUFFER_STATUS_SUCCESS);
	pthread_mutex_unlock(&cap_mutex_);
	return ret;
}

void Aravis::setPacketDelay(int64_t delay)
{
	arv_camera_gv_set_packet_delay(camera_, delay);
}

void Aravis::imageSize(int *width, int *height)
{
	*width=width_;
	*height=height_;
}
//2020/09/16 add by hato -------------------- start --------------------
bool Aravis::get_exposure_time_level(int *val)const{
	if( ! m_expsr_tm_lv_setting_ ){
		dprintf("error: exposure time level setting is null.");
		return false;
	}
	*val = m_expsr_tm_lv;
	return *val >= 0;
}

bool Aravis::get_exposure_time_level_default(int *val)const{
	if( ! m_expsr_tm_lv_setting_ ){
		dprintf("error: exposure time level setting is null.");
		return false;
	}
	*val = m_expsr_tm_lv_setting_->default_lv;
	return *val >= 0;
}

bool Aravis::get_exposure_time_level_min(int *val)const{
	if( ! m_expsr_tm_lv_setting_ ){
		dprintf("error: exposure time level setting is null.");
		return false;
	}
	*val = m_expsr_tm_lv_setting_->min_lv;
	return *val >= 0 ;
}

bool Aravis::get_exposure_time_level_max(int *val)const{
	if( ! m_expsr_tm_lv_setting_ ){
		dprintf("error: exposure time level setting is null.");
		return false;
	}
	*val = m_expsr_tm_lv_setting_->max_lv;
	return *val >= 0;
}

bool Aravis::set_exposure_time_level(const int lv){
	const ExposureTimeLevelSetting::Param * exp_tm_lv_param = nullptr;
	if( ! m_expsr_tm_lv_setting_ || ! ( exp_tm_lv_param = m_expsr_tm_lv_setting_->get_param(lv)) ){
		dprintf("error: exposure time level param is null. lv=%d",lv);
		return false;
	}
	
	dprintf("exposure time lv param. %s", exp_tm_lv_param->to_string().c_str());
	
	bool ret=false;
	
	set_node_int("AcquisitionFrameRate", 10);
	
	{
		const bool ret_expsr = reg_write(REG_EXPOSURE_TIME, exp_tm_lv_param->cam_exposure_tm);
		dprintf("camera exposure time set.    result=%s, set_val=%5d, cur_val=%5d",
			(ret_expsr?"OK":"NG"),exp_tm_lv_param->cam_exposure_tm,exposureTime());
	}
	{
		const bool ret_frame_rate = set_node_int("AcquisitionFrameRate", exp_tm_lv_param->cam_frame_rate);
		dprintf("camera frame rate set.       result=%s, set_val=%5d, cur_val=%5d", 
			(ret_frame_rate?"OK":"NG"),exp_tm_lv_param->cam_frame_rate, get_node_int("AcquisitionFrameRate"));
	}
	
	pset_stopgo(Proj_Disabled);
	int vres=1;
	dprintf("projector exposure time set. result=%s, set_val=%5d",
			"--",exp_tm_lv_param->proj_exposure_tm);
	do {
		const bool ret_prj_expsr = setProjectorExposureTime( exp_tm_lv_param->proj_exposure_tm );
		
		vres=pset_validate();
	} while(vres);
	pset_stopgo(Proj_Enabled);
	
	m_expsr_tm_lv = lv;
	ret = true;
		
	return ret;
}
//2020/09/16 add by hato --------------------  end  --------------------

int Aravis::exposureTime()
{
	return reg_read(REG_EXPOSURE_TIME);
}

//2020/11/09 comment by hato -------------------- start --------------------
//bool Aravis::setExposureTime(int value)
//{
//	bool ret;
//	ret = reg_write(REG_EXPOSURE_TIME, value);
//	if (isAsync()){
//		int hz = value < 1e6 / 60. ? 60 : 30;
//		ret = set_node_int("AcquisitionFrameRate", hz);
//	}
//	return ret;
//}
//2020/11/09 comment by hato --------------------  end  --------------------
int Aravis::gainA()
{
	return reg_read(REG_ANALOG_GAIN);
}

bool Aravis::setGainA(int value)
{
	if(value < CAM_DIGITAL_GAIN_MIN ){
		dprintf("camera gain is under minimum value.");
		return false;
	}else if( CAM_DIGITAL_GAIN_MAX < value){
		dprintf("camera gain is over maximum value.");
		return false;
	}
	return reg_write(REG_ANALOG_GAIN, value);
}

int Aravis::gainD()
{
	return reg_read(REG_DIGITAL_GAIN);
}

bool Aravis::setGainD(int value)
{
	return reg_write(REG_DIGITAL_GAIN, value);
}
//2021/01/26 add by hato -------------------- start --------------------
int Aravis::getHeartBeatTimeout(){
	return reg_read(REG_HEAT_BEAT_TIMEOUT);
}

	bool Aravis::setHeartBeatTimeout(const int val){
	return reg_write(REG_HEAT_BEAT_TIMEOUT,val);
}

//2021/01/26 add by hato --------------------  end  --------------------
	
void Aravis::on_new_buffer(ArvStream *stream, void *arg)
{
	Aravis *a=(Aravis*)arg;
	pthread_mutex_lock(&a->cap_mutex_);
	ArvBuffer *buffer;

	buffer = arv_stream_try_pop_buffer(stream);
	if (buffer) {
		a->buffer_status_ = arv_buffer_get_status(buffer);
		if (a->buffer_status_ == ARV_BUFFER_STATUS_SUCCESS){
    		size_t size;
	    	const void *img = arv_buffer_get_data(buffer, &size);
			if (a->out_) memcpy(a->out_, img, size);
			if (a->on_image_) (*a->on_image_)(a->camno_, a->frame_index_, a->width_, a->height_, a->color_, a->out_);
			++a->frame_index_;
		}
		else {
			if (a->buffer_status_==ARV_BUFFER_STATUS_TIMEOUT) dprintf("timeout");
			else dprintf("[%s] buffer error %d",a->name_,a->buffer_status_);
		}
		arv_stream_push_buffer (stream, buffer);
	}
	else{	//ここは通らないかも
		a->buffer_status_=ARV_BUFFER_STATUS_UNKNOWN;
		dprintf("buffer is null");
	}
	pthread_mutex_unlock(&a->cap_mutex_);
	pthread_cond_signal(&a->cap_cond_);
}

void Aravis::on_control_lost(ArvGvDevice *gv_device, void *arg)
{
	Aravis *a=(Aravis*)arg;
	dprintf("error: control lost: %s",a->name_);
	a->lost_=true;
	if(a->on_lost_){
		(*a->on_lost_)(a->camno_);
	}
}


uint32_t Aravis::reg_read(uint64_t reg)
{
	GError *err = nullptr;
	uint32_t value;
	bool ret = arv_device_read_register(device_, reg, &value, &err);
	if (!ret) value= (uint32_t)-1;
	if (err) {
		dprintf("error: arv_device_read_register[%s]", err->message);
	    g_clear_error (&err);
	}
	return value;
}

bool Aravis::reg_write(uint64_t reg, int value)
{
	GError *err = nullptr;
	bool ret = arv_device_write_register(device_, reg, value, &err);
	if (err) {
		dprintf("error: arv_device_write_register [0x%0X=%d] %s", reg, value, err->message);
	    g_clear_error (&err);
	}	
	return ret;
}

bool Aravis::uart_write(const char *cmd)
{
	for (const char *p = cmd; *p; ++p){
		UART_DATA_FIELD u;
		u.b.data = *p;
		u.w.data_lrc = ~u.w.data;
		if (!reg_write(REG_UART, u.dwData)){
			return false;
		}
	}
	return true;
}

bool Aravis::uart_write(char command, int data)
{
	char d[8];
	snprintf(d, sizeof(d), "%d", data);
	return uart_write(command, d);
}

bool Aravis::uart_write(char command, const char *data)
{
	char cmd[32];
	snprintf(cmd, sizeof(cmd), "%c%s\r", command, data);
	return uart_write(cmd);
}
//2020/11/17 add by hato -------------------- start --------------------
bool Aravis::uart_cmd(const char *cmd,const int sleep_ms){
	bool ret=false;
#ifdef DEBUG_DETAIL
	std::string cmd_str(cmd);
	cmd_str.erase(cmd_str.size()-1,1);
	//cmd_str.append("\\n");
	dprintf(">>> uart_cmd start. cmd='%s'",cmd_str.c_str());
#endif
	uart_flush();
	
	if( ! uart_write( cmd ) ){
		dprintf("error: uart_cmd failed. cmd=%s", cmd);
	}else{
		if( sleep_ms > 0){
#ifdef DEBUG_DETAIL
			dprintf("uart_cmd wait. %d msec",sleep_ms);
#endif
			usleep(sleep_ms * 1000);
		}
		ret=true;
	}
#ifdef DEBUG_DETAIL

	dprintf("<<< uart_cmd end. cmd='%s' ret=%s",cmd_str.c_str(),(ret?"OK":"NG"));
#endif
	return ret;
}
//2020/11/17 add by hato --------------------  end  --------------------
//2020/09/25 add by hato -------------------- start --------------------
bool Aravis::uart_cmd(const char command,const char *val,const int sleep_ms){
	bool ret=false;
#ifdef DEBUG_DETAIL
	dprintf(">>> uart_cmd start. cmd='%c' val=%s",command,val);
#endif
	uart_flush();
	
	if( ! uart_write( command, val ) ){
		dprintf("error: uart_cmd failed. cmd=%c val=%d", command, val);
	}else{
		//2020/11/17 uart_read不要になった為
		/*
#ifdef DEBUG_DETAIL
		std::string reply=uart_read();
		dprintf("---------------- cmd=%c val=%d reply start -----------------",command,val);
		dprintf("%s",reply.c_str());
		dprintf("---------------- cmd=%c val=%d reply  end  -----------------",command,val);
#else
		uart_read();
#endif
		*/
		
		if( sleep_ms > 0){
#ifdef DEBUG_DETAIL
			dprintf("uart_cmd wait. %d msec",sleep_ms);
#endif
			usleep(sleep_ms * 1000);
		}
		ret=true;
	}
#ifdef DEBUG_DETAIL
	dprintf("<<< uart_cmd end.   cmd='%c' val=%s ret=%s",command,val ,(ret?"OK":"NG"));
#endif
	return ret;
}


bool Aravis::uart_cmd(const char command,const int val,const int sleep_ms){
	bool ret=false;
#ifdef DEBUG_DETAIL
	dprintf(">>> uart_cmd start. cmd='%c' val=%d",command,val);
#endif
	uart_flush();
	
	if( ! uart_write( command, val ) ){
		dprintf("error: uart_cmd failed. cmd=%c val=%d", command, val);
	}else{
		//2020/11/17 uart_read不要になった為
		/*
#ifdef DEBUG_DETAIL
		std::string reply=uart_read();
		dprintf("---------------- cmd=%c val=%d reply start -----------------",command,val);
		dprintf("%s",reply.c_str());
		dprintf("---------------- cmd=%c val=%d reply  end  -----------------",command,val);
#else
		uart_read();
#endif
		*/
		
		if( sleep_ms > 0){
#ifdef DEBUG_DETAIL
			dprintf("uart_cmd wait. %d msec",sleep_ms);
#endif
			usleep(sleep_ms * 1000);
		}
		ret=true;
	}
#ifdef DEBUG_DETAIL
	dprintf("<<< uart_cmd end.    cmd='%c' val=%d ret=%s",command,val,(ret?"OK":"NG"));
#endif
	return ret;
}

std::string Aravis::uart_read(){
	string ret;
	const int RMAX = 2048;
	ret.reserve(RMAX);
	const string st = "//cmd:diag";
//2020/11/25 modified by hato -------------------- start  --------------------
	//const string ed = "Dlp.X>";
	const string ed = "\r\n";
//2020/11/25 modified by hato --------------------  end  --------------------
	const size_t nst = st.length();
	const size_t ned = ed.length();
	for (int i = 0; i < RMAX; ++i){
		UART_DATA_FIELD u;
		u.dwData = reg_read(REG_UART);
		if (u.b.data){
			ret.push_back(u.b.data);
			if (nst <= ret.length() && ret.compare(ret.length() - nst, nst, st) == 0){	//開始が出てきたらリセット
				ret.clear();
				ret.append(st);
			}
			else if (ned <= ret.length() && ret.compare(ret.length() - ned, ned, ed) == 0){	//終了
				break;
			}
		}
		usleep(100);
	}
	return ret;
}
//2020/09/25 add by hato --------------------  end --------------------

void Aravis::uart_flush()
{
	for (int i = 0; i < 8192; ++i){
		UART_DATA_FIELD u;
		u.dwData = reg_read(REG_UART);
		if (!u.b.data){
			break;
		}
	}
}

string Aravis::get_node_str(const char *feature)
{
	GError *err = nullptr;
	string ret;
	ArvGcNode *node = arv_device_get_feature(device_, feature);
	if (node && ARV_IS_GC_STRING(node)){
		ret = arv_gc_string_get_value(ARV_GC_STRING(node), &err);
		if (err) {
			dprintf("error: arv_gc_string_get_value[%s][%s]", feature, err->message);
		    g_clear_error (&err);
		}
	}
	return ret;
}

int64_t Aravis::get_node_int(const char *feature)
{
	GError *err = nullptr;
	int64_t ret = -1;
	ArvGcNode *node = arv_device_get_feature(device_, feature);
	if (node && ARV_IS_GC_INTEGER_NODE(node)){
		ret = arv_gc_integer_get_value(ARV_GC_INTEGER(node), &err);
		if (err) {
			dprintf("error: arv_gc_integer_get_value[%s][%s]", feature, err->message);
		    g_clear_error(&err);
		}
	}
	return ret;
}

bool Aravis::set_node_int(const char *feature, int64_t value)
{
	GError *err = nullptr;
	bool ret = false;
	ArvGcNode *node = arv_device_get_feature(device_, feature);
	if (node && ARV_IS_GC_INTEGER_NODE(node)){
		arv_gc_integer_set_value(ARV_GC_INTEGER(node), value, &err);
		if (err) {
			dprintf("error: arv_gc_integer_get_value[%s][%s]", feature, err->message);
		    g_clear_error(&err);
		}
		else ret = true;
	}
	return ret;
}

double Aravis::get_node_float(const char *feature)
{
	GError *err = nullptr;
	double ret = NAN;
	ArvGcNode *node = arv_device_get_feature(device_, feature);
	if (node && ARV_IS_GC_FLOAT_NODE(node)){
		ret = arv_gc_float_get_value(ARV_GC_FLOAT(node), &err);
		if (err) {
			dprintf("error: arv_gc_float_get_value[%s][%s]", feature, err->message);
		    g_clear_error(&err);
		}
	}
	return ret;
}

string Aravis::get_description(const char *feature)
{
	GError *err = nullptr;
	string ret;
	ArvGcNode *node = arv_device_get_feature(device_, feature);
	if (node){
		ret = arv_gc_feature_node_get_description(ARV_GC_FEATURE_NODE(node), &err);
		if (err) {
			dprintf("error: arv_gc_feature_node_get_description[%s][%s]", feature, err->message);
		    g_clear_error(&err);
		}
	}
	return ret;
}

void Aravis::addCallbackImage(OnRecvImage *onRecvImage, uint8_t *out)
{
	on_image_ = onRecvImage;
	out_ = out;
}

void Aravis::addCallbackLost(OnLostCamera *onLost)
{
	on_lost_ = onLost;
}
//2020/11/09 comment by hato -------------------- start --------------------
/*
bool Aravis::setTriggerMode(YCAM_TRIG tm)
{
	bool ret;
	if (isAsync()){	//非同期ver
		//2020/11/05 comment out by hato --------------------  end  --------------------
////2020/09/25 modified by hato -------------------- start --------------------
//		//ret = uart_write('a', tm==YCAM_TRIG_EXT ? 1 : 0);
//		ret = uart_cmd( 'a' , tm==YCAM_TRIG_EXT ? 1 : 0 );
////2020/09/25 modified by hato --------------------  end  --------------------
		//2020/11/05 comment out by hato --------------------  end  --------------------
		
	}
	else{
		ret = reg_write(REG_EXTERNAL_TRIGGER, tm==YCAM_TRIG_EXT ? 1 : 0);
	}
	trigger_mode_ = tm;
	return ret;
}
*/
//2020/11/09 comment by hato --------------------  end  --------------------
//2020/10/09 modified by hato -------------------- start --------------------
//bool Aravis::trigger(YCAM_PROJ_MODE mode)
bool Aravis::trigger(YCAM_PROJ_MODE mode)
//2020/10/09 modified by hato --------------------  end  --------------------
{
	//2020/11/09 comment by hato -------------------- start --------------------
	//if (trigger_mode_ == YCAM_TRIG_INT){
	//	dprintf("warning: current trigger mode is INTERNAL");
	//	return false;
	//}
	//2020/11/09 comment by hato --------------------  end  --------------------
	
	frame_index_ = 0;
	bool ret;
//2020/11/09 comment by hato -------------------- start --------------------
//	if (isAsync()){	//非同期ver
//2020/11/09 comment by hato --------------------  end  --------------------
		
//2020/11/05 modified by hato -------------------- start --------------------
////2020/09/25 modified by hato -------------------- start --------------------
//		//uart_write('a', 1);	//プロジェクターON
//		ret = uart_cmd( 'a' , projectorOn ? 1 : 0);
////2020/09/25 modified by hato --------------------  end  --------------------
//2020/11/05 modified by hato --------------------  end  --------------------
//2020/11/06 modified by hato ------------------ start ------------------
		//int num = (mode == YCAM_PROJ_MODE_CONT) ? PHSFT_CAP_NUM : 1;
		int num = 1;
		if( mode == YCAM_PROJ_MODE_CONT){
			num = getCaptureNum();
		}
//2020/11/06 modified by hato ------------------  end  ------------------
		ret = reg_write(REG_STREAM_NUM, num);
//2020/11/09 comment by hato -------------------- start --------------------
/*
	}
	else{
//2020/09/25 modified by hato -------------------- start --------------------
		//ret = uart_write('o', mode);
		uart_cmd( 'o' , mode);
//2020/09/25 modified by hato --------------------  end  --------------------
	}
*/
//2020/11/09 comment by hato --------------------  end  --------------------
	return ret;
}

//2020/11/06 add by hato -------------------- start --------------------
int Aravis::getCaptureNum()const{
	int num = 1;
	
	if( getProjectorPattern() == YCAM_PROJ_PTN_PHSFT_3 ){
		num = PHSFT3_CAP_NUM;
	}else if(getProjectorPattern() == YCAM_PROJ_PTN_PHSFT){
		num = PHSFT_CAP_NUM;
	}
	return num;
}
//2020/11/06 add by hato --------------------  end  --------------------


//2020/11/05 modified by hato -------------------- start --------------------
/*
bool Aravis::setProjectorEnabled(const bool enabled){
	if( ! enabled ){
		pset_stopgo(Proj_Disabled);
	}else{
		int vres=1;
		do {
			vres=pset_validate();
		} while(vres);
		pset_stopgo(Proj_Enabled);
	}
	return true;
}
*/
//2020/11/05 modified by hato --------------------  end  --------------------

/*** 診断メッセージ　dコマンドで以下フォ－マット
	//cmd:diag
	//diag.hardware status: 0x01
	//diag.system status: 0x01
	//diag.main status: 0x0E
	//diag.input source(2 as flush): 0x02
	//diag.display mode(1 as pattern): 0x01
	//diag.pattern source(3 asflush): 0x03
	//diag.trigger mode(1 as external): 0x01
	//diag.exposure time: 8532992
	//diag.frame time: 8532992
	//diag.rgb: B4.B4.B4
	//diag.lut: 0C.01.0C.01
	//Cycle: 50
	//Trigger Logic: 0
	//diag.temp: 650
	//firmware ver.: 1.00
	cmd:d=1:OK
	Dlp.X>
	***/
string Aravis::uart_dump()
{
	string ret;
	uint32_t dwAdr = REG_UART;
	const int RMAX = 2048;
	ret.reserve(RMAX);

	const string st = "//cmd:diag";
	const string ok = "cmd:d=1:OK";
	const string ed = "Dlp.X>";
	const size_t nst = st.length();
	const size_t nok = ok.length();
	const size_t ned = ed.length();
	uart_flush();
	for (int n = 0; n < 5; ++n){	//最大5回
		ret.clear();
		if (!uart_write('d', "")){
			return ret;
		}
		int range = 0;
		for (int i = 0; i < RMAX; ++i){
			UART_DATA_FIELD u;
			u.dwData = reg_read(REG_UART);
			if (u.b.data){
				ret.push_back(u.b.data);
				if (nst <= ret.length() && ret.compare(ret.length() - nst, nst, st) == 0){	//開始が出てきたらリセット
					ret.clear();
					ret.append(st);
					++range;
				}
				else if (nok <= ret.length() && ret.compare(ret.length() - nok, nok, ok) == 0){	//OK
					ret.append(st);
					++range;
				}
				else if (ned <= ret.length() && ret.compare(ret.length() - ned, ned, ed) == 0){	//終了
					++range;
					break;
				}
			}
		}
		if (range == 3) break;
	}
	//printf("%s", ret.c_str());
	return ret;
}

//直前のコマンドの処理が終わるのを待つ
//return true:OK
//       false:timeout
bool Aravis::projector_wait(){
	bool ret = false;
	for (;;){
		Sleep(100);
		auto dump = uart_dump();
		if (dump.empty()) break;
		auto list = split_str(dump, "\r\n");
		for (auto a : list){
			if (a.find("cmd:d=1:OK") != std::string::npos){	//uart_dumpで見てますが一応
				ret = TRUE;
				break;
			}
		}
		break;
	}
	return ret;
}

int Aravis::projector_value(const char *key_str, std::string *str)
{
	int ret = -1;
	auto dump = uart_dump();
	if (dump.empty()) return -1;
	auto list = split_str(dump, "\r\n");
	for (auto a : list){
		auto items = split_str(a, ":");
		if (1 < items.size()){
			if (items[0].find(key_str) != std::string::npos){
				ret = atoi(items.back().c_str());
				if (str) *str = items.back();
				break;
			}
		}
	}
	return ret;
}

//2020/11/05 modified by hato -------------------- start --------------------
/*
int Aravis::projectorIntensity()
{
	string str;
	int ret = -1;
	for (int n = 0; n < 5; ++n){
		projector_value("diag.rgb", &str);
		ret = strtoul(str.c_str(), NULL, 16);
		if (ret < 0) continue;
		break;
	}
	return ret;
}*/
//2020/11/05 modified by hato --------------------  end  --------------------
	

bool Aravis::setProjectorIntensity(int value)
{
	//2020/12/10 add by hato -------------------- start --------------------
	if(value < PROJ_INTENSITY_MIN){
		dprintf("projector intensity is under minimum value.");
		return false;
	}else if(PROJ_INTENSITY_MAX < value ){
		dprintf("projector intensity is over maximum value.");
		return false;
	}
	//2020/12/10 add by hato -------------------- start --------------------
	
	char d[16];
	snprintf(d, sizeof(d), "%02X%02X%02X", value, value, value);
	//2020/12/10 modified by hato -------------------- start --------------------
	//return uart_write('i', d);
	const bool ret= uart_cmd( 'i' , d , PROJ_INTENSITY_WAIT);
	if( ret ){
		cur_proj_intensity_=value;
	}
	return ret;
	//2020/12/10 modified by hato --------------------  end  --------------------
}
	
//2020/11/30 modified by hato -------------------- start --------------------
bool Aravis::setProjectorPattern(YCAM_PROJ_PTN ptn,const bool shortWait)
//2020/11/30 modified by hato --------------------  end  --------------------
{
	bool ret = false;
//2020/11/09 comment by hato -------------------- start --------------------
//	if (isAsync()){	//非同期ver
//2020/11/09 comment by hato --------------------  end  --------------------
		//2020/11/05 modified by hato -------------------- start --------------------
//		for (;;){
////2020/09/25 modified by hato -------------------- start --------------------
//#if 0
//			if (!uart_write('a', 0)) break;			//外部トリガ発行を受付しない
//			if (!uart_write('z', ptn)) break;		//パターン切り替え
//			Sleep(3000);
//			if (!uart_write('a', 1)) break;			//外部トリガ発行を受付する(cam->proj)
//#endif
//			if ( ! uart_cmd('a', 0) ) break;			//外部トリガ発行を受付しない
//			if ( ! uart_cmd('z', ptn) ) break;		//パターン切り替え
//			Sleep(3000);
//			if ( ! uart_cmd('a', 1) ) break;			//外部トリガ発行を受付する(cam->proj)
////2020/09/25 modified by hato --------------------  end  --------------------
//			ret = true;
//			break;
//		}
		pset_stopgo(Proj_Disabled,shortWait);
		int vres=1;
		do {
			uart_cmd('z', ptn );
			vres=pset_validate();
		} while(vres);
		pset_stopgo(Proj_Enabled,shortWait);
		ret=true;
		//2020/11/05 modified by hato --------------------  end  --------------------
//2020/11/09 comment by hato -------------------- start --------------------
/*
	}
	else{
//2020/09/25 modified by hato -------------------- start --------------------
		//if (uart_write('z', ptn)){
		if( uart_cmd( 'z' , ptn) ){
//2020/09/25 modified by hato --------------------  end  --------------------
			ret = projector_wait();
		}
	}
*/
//2020/11/09 comment by hato --------------------  end  --------------------

	//2020/11/05 modified by hato -------------------- start --------------------
	if(ret){
		cur_proj_ptn_ = ptn;
	}
	//2020/11/05 modified by hato --------------------  end  --------------------
	
	return ret;
}

int Aravis::projectorExposureTime()
{
	int ret = -1;
	ret = projector_value("diag.exposure time");
	if(0 < ret){
		if (!isAsync()){
			ret >>= 10;
		}
	}
	return ret;
}

bool Aravis::setProjectorExposureTime(int value)
{
	
//2020/09/25 modified by hato -------------------- start --------------------
	//if (uart_write('x', value)){
	//	return projector_wait();
	//}
	//return false;
	if( ! uart_cmd( 'x' , value) ){
		dprintf("error: setProjectorExposureTime failed. value=%d",value);
	}	
	return true;
//2020/09/25 modified by hato --------------------  end  --------------------
}


bool Aravis::setProjectorFlashInterval(int value)
{
//2020/09/25 modified by hato -------------------- start --------------------
	//return uart_write('p', value);
	return uart_cmd( 'p', value ,PROJ_FLASH_INTERVAL_WAIT);
//2020/09/25 modified by hato --------------------  end  --------------------
}

int Aravis::projectorFlashInterval()
{
	//return projector_value("Cycle");
	int val=projector_value("Cycle");
	dprintf("projectorFlashInterval=%d",val);
	return val;
}

//2020/11/05 modified by hato -------------------- start --------------------
/* validate setting - execute after changing parameter */
int Aravis::pset_validate(void) {
	uart_cmd("v\n",200);
	return 0x1F & atoi(uart_read().c_str());
}
	
/* stop(0)/go(2) execute after validating */ 
void Aravis::pset_stopgo(ProjectorEnabled n,const bool shortWait) {
	char cmd[8];
	sprintf(cmd,"q%d\n",n);
	uart_cmd(cmd,shortWait?PROJ_STOP_GO_WAIT_TM_SHORT:PROJ_STOP_GO_WAIT_TM_NORMAL);
	//usleep(200000);
	//usleep(200000);
	
	//ElapsedTimer tmr;
	//std::string read_str= uart_read();
	//dprintf("time=%d\n",tmr.elapsed_ms());
	//dprintf("uart:[%s]\n",read_str.c_str());
	//cur_proj_enabled_=n;

}
		
//2020/11/05 modified by hato --------------------  end  --------------------
//2020/11/10 add by hato -------------------- start --------------------
//int pset_gettemp(void) {
//	uart_write("g\n");
//	usleep(400000);
//	return atoi(uart_read().c_str());
//}

int Aravis::getTemperature(){
	uart_cmd("g\n",100);
	//usleep(200000);
	//usleep(400000);
	std::string ret=uart_read();
	//fprintf(stderr,"[%s]\n",ret.c_str());
	return atoi(ret.c_str());
}
//2020/11/10 add by hato --------------------  end  --------------------
	
//////////////////////////////////////////////////////////
//yamlテンプレート
const std::string yaml_tmpl =
"%YAML:1.0" "\n"
"---" "\n"
"cam{cam}_size: [ {size} ]" "\n"
"cam{cam}_K: !!opencv-matrix" "\n"
"   rows: {K_rows}" "\n"
"   cols: {K_cols}" "\n"
"   dt: d" "\n"
"   data: [ {K_data} ]" "\n"
"cam{cam}_R: !!opencv-matrix" "\n"
"   rows: {R_rows}" "\n"
"   cols: {R_cols}" "\n"
"   dt: d" "\n"
"   data: [ {R_data} ]" "\n"
"cam{cam}_T: !!opencv-matrix" "\n"
"   rows: {T_rows}" "\n"
"   cols: {T_cols}" "\n"
"   dt: d" "\n"
"   data: [ {T_data} ]" "\n"
"cam{cam}_D: !!opencv-matrix" "\n"
"   rows: {D_rows}" "\n"
"   cols: {D_cols}" "\n"
"   dt: d" "\n"
"   data: [ {D_data} ]" "\n";

#define NODE_INT(a) ((int)this->get_node_int(node_name(a, res).c_str()))
#define NODE_INT2(a) ((int)this->get_node_int(node_name(a, res, cam).c_str()))
#define NODE_FLT(a) (this->get_node_float(node_name(a, res).c_str()))
#define NODE_FLT2(a) (this->get_node_float(node_name("YCam_%s_" #a "%d", res, n).c_str()))
#define NODE_FLT3(a) (this->get_node_float(node_name("YCam_%s_cam%d_" #a "%d", res, cam, n).c_str()))

template <typename ... Args> std::string format(const std::string& fmt, Args ... args)
{
	size_t len = std::snprintf(nullptr, 0, fmt.c_str(), args ...);
	std::vector<char> buf(len + 1);
	std::snprintf(&buf[0], len + 1, fmt.c_str(), args ...);
	return std::string(&buf[0], &buf[0] + len);
}

bool Aravis::upload_camparam(YCAM_RES reso, YCAM_SIDE side, const char *yaml_path)
{
	bool ret=false;
	char buf[128] = { 0 };
	const char *res = reso == YCAM_RES_SXGA ? "SXGA":"VGA";
	for (;;){
		const int cam = side;
		const char *key[] = {"cp0", "cp1"};
		dprintf("upload %s-%s [%s]", res, cam == YCAM_SIDE_LEFT ? "LEFT":"RITHG", yaml_path);
		string yaml;
		yaml.reserve(4096);
		yaml.append(yaml_tmpl);
		replace_str(yaml, "{cam}", to_string(cam), true);
		{
			char val[64];
			snprintf(val, sizeof(val), "%d, %d", NODE_INT("YCam_%s_Width"), NODE_INT("YCam_%s_Height"));
			replace_str(yaml, "{size}", val);
		}
		{	//K
			const int r = NODE_INT("YCam_K_Rows");
			const int c = NODE_INT("YCam_K_Cols");
			replace_str(yaml, "{K_rows}", to_string(r));
			replace_str(yaml, "{K_cols}", to_string(c));
			string buf;
			buf.reserve(512);
			for (int n = 0; n < (r * c); ++n){
				if (n) buf.append(", ");
				buf.append(format("%.19f", NODE_FLT3(K)));
			}
			replace_str(yaml, "{K_data}", buf);
		}
		{	//R
			const int r = NODE_INT("YCam_R_Rows");
			const int c = NODE_INT("YCam_R_Cols");
			replace_str(yaml, "{R_rows}", to_string(r));
			replace_str(yaml, "{R_cols}", to_string(c));
			string buf;
			if (cam){
				buf.reserve(512);
				for (int n = 0; n < (r * c); ++n){
					if (n) buf.append(", ");
					buf.append(format("%.19f", NODE_FLT2(R)));
				}
			}
			else buf = "1., 0., 0., 0., 1., 0., 0., 0., 1.";
			replace_str(yaml, "{R_data}", buf);
		}
		{	//T
			const int r = NODE_INT("YCam_T_Rows");
			const int c = NODE_INT("YCam_T_Cols");
			replace_str(yaml, "{T_rows}", to_string(r));
			replace_str(yaml, "{T_cols}", to_string(c));
			string buf;
			if (cam){
				buf.reserve(512);
				for (int n = 0; n < (r * c); ++n){
					if (n) buf.append(", ");
					buf.append(format("%.19f", NODE_FLT2(T)));
				}
			}
			else buf = "0., 0., 0.";
			replace_str(yaml, "{T_data}", buf);
		}
		{	//D
			const int r = 1;
			const int c = NODE_INT2("YCam_%s_cam%d_NDistortion");
			replace_str(yaml, "{D_rows}", to_string(r));
			replace_str(yaml, "{D_cols}", to_string(c));
			string buf;
			buf.reserve(512);
			for (int n = 0; n < c; ++n){
				if (n) buf.append(", ");
				buf.append(format("%.19f", NODE_FLT3(D)));
			}
			replace_str(yaml, "{D_data}", buf);
		}
		ofstream ofs(yaml_path);
		if (!ofs){
			dprintf("error: open[%s]", yaml_path);
			break;
		}
		ofs << yaml;
		ret = true;
		break;
	}
	return ret;
}
