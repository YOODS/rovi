#pragma once 

#include <chrono>
#include <string>
#include <vector>
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "Aravis.h"
#include "YCAM3D.h"

class CameraYCAM3D;

namespace camera{
	namespace ycam3d{
		//constexpr int CAM_EXPOSURE_TIME_DEFAULT   = aravis::ycam3d::CAM_EXPOSURE_TIME_DEFAULT;
		//constexpr int CAM_EXPOSURE_TIME_MAX       = aravis::ycam3d::CAM_EXPOSURE_TIME_MAX;
		//constexpr int CAM_EXPOSURE_TIME_MIN       = aravis::ycam3d::CAM_EXPOSURE_TIME_MIN;
		
		constexpr int CAM_DIGITAL_GAIN_DEFAULT   = aravis::ycam3d::CAM_DIGITAL_GAIN_DEFAULT;
		constexpr int CAM_DIGITAL_GAIN_MAX       = aravis::ycam3d::CAM_DIGITAL_GAIN_MAX;
		constexpr int CAM_DIGITAL_GAIN_MIN       = aravis::ycam3d::CAM_DIGITAL_GAIN_MIN;
		
		constexpr int CAM_ANALOG_GAIN_DEFAULT   = aravis::ycam3d::CAM_ANALOG_GAIN_DEFAULT;
		constexpr int CAM_ANALOG_GAIN_MAX       = aravis::ycam3d::CAM_ANALOG_GAIN_MAX;
		constexpr int CAM_ANALOG_GAIN_MIN       = aravis::ycam3d::CAM_ANALOG_GAIN_MIN;
		
		//constexpr int PROJ_EXPOSURE_TIME_DEFAULT  = aravis::ycam3d::PROJ_EXPOSURE_TIME_DEFAULT;
		//constexpr int PROJ_EXPOSURE_TIME_MIN      = aravis::ycam3d::PROJ_EXPOSURE_TIME_MIN;
		//constexpr int PROJ_EXPOSURE_TIME_MAX      = aravis::ycam3d::PROJ_EXPOSURE_TIME_MAX;
		
		constexpr int PROJ_INTENSITY_DEFAULT     = aravis::ycam3d::PROJ_INTENSITY_DEFAULT;
		constexpr int PROJ_INTENSITY_MIN         = aravis::ycam3d::PROJ_INTENSITY_MIN;
		constexpr int PROJ_INTENSITY_MAX         = aravis::ycam3d::PROJ_INTENSITY_MAX;
		
		//constexpr int PROJ_FLASH_INTERVAL_DEFAULT = aravis::ycam3d::PROJ_FLASH_INTERVAL_DEFAULT;
		//constexpr int PROJ_FLASH_INTERVAL_MIN     = aravis::ycam3d::PROJ_FLASH_INTERVAL_MIN;
		//constexpr int PROJ_FLASH_INTERVAL_MAX     = aravis::ycam3d::PROJ_FLASH_INTERVAL_MAX;
		
		
		//constexpr int PATTERN_CAPTURE_NUM = aravis::ycam3d::PATTERN_CAPTURE_NUM;
		
		struct CameraImage {
			bool result =false;
			int width = -1;
			int height = -1;
			int step = -1;
			int color_ch = -1;
			std::chrono::system_clock::time_point dt;
			
			std::vector<unsigned char> data;
			
			CameraImage(){}
			
			CameraImage(const int a_width,const int a_height,const int a_step,const int a_color_ch=1):
				width(a_width),
				height(a_height),
				step(a_step),
				color_ch(a_color_ch)
			{
				
			}
				
			bool valid()const{
				return width > 0 && height > 0 && color_ch > 0 && step > 0 && data.size() == byte_count();
			}
			
			int byte_count()const{
				return step * height;
			}
			
			bool alloc(){
				const int buf_size=byte_count();
				if( buf_size <= 0 ) { return false; }
				data.assign(buf_size,0);
				return data.size() == buf_size;
			}
			
			bool to_mat(cv::Mat &img)const {
				if( ! result || ! valid() ){ return false; }
				
				img=cv::Mat(height,width,CV_8UC1,cv::Scalar(0));
				memcpy(img.data,data.data(),byte_count());
				return true;
			}
			
			bool to_ros_img(sensor_msgs::Image &img,const std::string &frame_id=std::string())const{
				if( ! result || ! valid() ){ return false; }
				
				const auto t0 = std::chrono::time_point<std::chrono::high_resolution_clock>{};
				const auto t1 = this->dt;
				
				const auto tstamp = t1 - t0;
				const int32_t sec = std::chrono::duration_cast<std::chrono::seconds>(tstamp).count();
				const int32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(tstamp).count() % 1000000000UL;
				img.header.stamp = ros::Time(sec, nsec);
				
				//char date[64];
				//time_t t = img.header.stamp.sec;
    			//strftime(date, sizeof(date), "%Y/%m/%d %a %H:%M:%S", localtime(&t));
				//std::cerr << "!!!" <<  date << std::endl;
								
				if( ! frame_id.empty() ){
					img.header.frame_id = frame_id;
				}
				img.width = this->width;
				img.height = this->height;
				img.encoding = sensor_msgs::image_encodings::MONO8;
				img.step = this->step;
				img.is_bigendian = false;
				img.data.resize(this->byte_count());
				memcpy(img.data.data(),this->data.data(),this->byte_count());
				
				return true;
			}
			
			const CameraImage operator-(const CameraImage &b)const{
				CameraImage diff;
				if(this->width == b.width &&
					this->height == b.height && 
					this->step == b.step && 
					this->color_ch == b.color_ch && 
					this->data.size() == b.data.size()
				){
					diff = *this;
					for (int i=0; i < this->data.size(); i++) {
						int val = this->data[i] - b.data[i];
						if (val < 1) { val = 0; }
						else if (val>255) { val=255; }
						
						diff.data[i] = val;
					}
				}
				return diff;
			}
		};
		
		struct CaptureParameter{
			int expsr_lv = -1;
			int gain = -1;
			int proj_intensity = -1;
			
			virtual std::string to_string()const{
				std::stringstream ss;
				ss << "expsr_lv=" << expsr_lv;
				ss << ",gain=" << gain;
				ss << ",proj_intensity=" << proj_intensity;
				return ss.str();
			}
			
			virtual bool operator!=(const CaptureParameter &param)const{
				return ! (*this==param);
			}
			
			virtual bool operator==(const CaptureParameter &param)const{
				if( this->expsr_lv != param.expsr_lv || 
				    this->gain != param.gain || 
				    this->proj_intensity != param.proj_intensity ){
					return false;
				}
				return true;
			}
			bool is_different(const CaptureParameter &param){
				
				bool ret=false;
				if(param.expsr_lv < 0 ){
					//skipped
				}else if( this->expsr_lv < 0){
					ret=true;
				}else if( this->expsr_lv != param.expsr_lv ){
					ret=true;
				}
				
				if(param.gain < 0 ){
					//skipped
				}else if( this->gain < 0){
					ret=true;
				}else if( this->gain != param.gain ){
					ret=true;
				}
				
				if(param.proj_intensity < 0 ){
					//skipped
				}else if( this->proj_intensity < 0){
					ret=true;
				}else if( this->proj_intensity != param.proj_intensity ){
					ret=true;
				}
				return ret;
			}
		};
		
		extern const int YCAM3D_RESET_INTERVAL;
		extern const int YCAM3D_RESET_AFTER_WAIT;
		
		//void start_ycam3d_reset(const char *ipaddr);
		bool reset_ycam3d(const char *ipaddr);
		
		using f_camera_open_finished = std::function<void(const bool result)>;
		using f_camera_disconnect = std::function<void(void)>;
		using f_camera_closed = std::function<void(void)>;
		using f_pattern_img_received = std::function<void(const bool result,const int elapsed, const std::vector<camera::ycam3d::CameraImage> &imgs_l,const std::vector<camera::ycam3d::CameraImage> &imgs_r,const bool timeout,const int expsrLv)>;
		using f_capture_img_received = std::function<void(const bool result,const int elapsed, camera::ycam3d::CameraImage &img_l,const camera::ycam3d::CameraImage &img_r,const bool timeout,const int expsrLv)>;
		using f_network_delayed = std::function<void(void)>;
		using f_auto_con_limit_exceeded=std::function<void(void)>;
		using f_ros_error_published=std::function<void(std::string)>;
	}
}

class CameraYCAM3D {
protected:
	
	enum CaptureStatus {
		CaptStat_Ready,
		CaptStat_Single,
		CaptStat_Pattern
	};
	
	struct CameraImageReceivedCallback : public OnRecvImage
	{
		CameraYCAM3D * const m_self;
		explicit CameraImageReceivedCallback(CameraYCAM3D *obj);
		void operator()(int camno, int frmidx, int width, int height, int color, void *mem) override;
	};
		
	struct CameraDisconnectCallbck : public OnLostCamera
	{
		CameraYCAM3D * const m_self;
		explicit CameraDisconnectCallbck(CameraYCAM3D *obj);
		void operator()(int camno) override;
	};
	
private:
	YCAM_RES m_ycam_res;
	std::unique_ptr<Aravis> m_arv_ptr;
	std::vector<unsigned char> m_arv_img_buf;
	
	std::atomic<bool> m_open_stat;
	std::timed_mutex m_camera_mutex;
	
	bool m_auto_connect_abort;
	std::thread m_auto_connect_thread;
	
	std::timed_mutex m_auto_connect_mutex;
	
	std::string m_auto_connect_ipaddr;
	
	CameraImageReceivedCallback m_on_image_received;
	CameraDisconnectCallbck m_on_disconnect;
	
	int m_capture_timeout_period;
	int m_trigger_timeout_period;
	std::thread m_capture_thread;
	
	int m_pre_heart_beat_val;
	bool m_cancel_delay_mon;
	std::thread m_delay_mon;
	
	camera::ycam3d::f_camera_open_finished m_callback_cam_open_finished;
	camera::ycam3d::f_camera_closed m_callback_cam_closed;
	camera::ycam3d::f_capture_img_received m_callback_capt_img_recv;
	camera::ycam3d::f_pattern_img_received m_callback_trig_img_recv;
	camera::ycam3d::f_network_delayed m_callback_nw_delayed;
	camera::ycam3d::f_auto_con_limit_exceeded m_callback_auto_lm_excd;
	camera::ycam3d::f_ros_error_published m_ros_err_pub;
	
	bool reset_image_buffer(const int capt_num);
	
	bool get_camera_param_int(const std::string &label,std::function<bool(int*)> func,int *val);
	bool set_camera_param_int(const std::string &label,std::function<bool(int)> func,const int val);
protected:
	int m_camno;
		
	camera::ycam3d::f_camera_disconnect m_callback_cam_disconnect;
	
	std::atomic<CaptureStatus> m_capt_stat;
	std::timed_mutex m_capt_finish_wait_mutex;
	
	std::timed_mutex m_img_update_mutex; //m_imgs_left, m_imgs_right, m_img_recv_flags ëŒè€
	std::vector<camera::ycam3d::CameraImage> m_imgs_left;
	std::vector<camera::ycam3d::CameraImage> m_imgs_right;
	std::vector<bool> m_img_recv_flags;
	

public:
	CameraYCAM3D();
	virtual ~CameraYCAM3D();
	
	bool init(const std::string &camera_res);
	
	int width()const;
	int height()const;
	
	bool is_open()const;
	void open();
	void close();
	
	bool is_auto_connect_running();
	
	bool is_busy();
	
	void set_capture_timeout_period(const int timeout);
	
	void set_trigger_timeout_period(const int timeout);
	
	bool capture(const bool strobe);
	
	//bool capture_strobe();
	
	bool capture_pattern(const bool multi,const bool ptnCangeWaitShort);
	
	void start_auto_connect(const std::string ipaddr="");
	
	bool get_exposure_time_level_default(int *val)const;
	
	bool get_exposure_time_level_min(int *val)const;
	bool get_exposure_time_level_max(int *val)const;
	
	bool get_exposure_time_level(int *val);
	bool set_exposure_time_level(const int val);
	
	bool get_exposure_time(int *val);
	//bool set_exposure_time(const int val);
	
	bool get_gain_digital(int *val);
	bool set_gain_digital(const int val);
	
	bool get_gain_analog(int *val);
	bool set_gain_analog(const int val);
	
	bool get_projector_exposure_time(int *val);
	//bool set_projector_exposure_time(const int val);
	
	bool get_projector_intensity(int *val);
	bool set_projector_intensity(const int val);
	
	//bool get_projector_interval(int *val);
	//bool set_projector_interval(const int val);
	//void ser_projector_pattern(int val);
	
	void set_callback_ros_error_published(camera::ycam3d::f_ros_error_published callback);
	void set_callback_auto_con_limit_exceeded(camera::ycam3d::f_auto_con_limit_exceeded callback);
	
	bool get_temperature(int *val);
	
	bool get_capture_param(camera::ycam3d::CaptureParameter *capt_param);
	bool update_capture_param(const camera::ycam3d::CaptureParameter &capt_param);
	
	void start_nw_delay_monitor_task(const int sec,const int timeout,camera::ycam3d::f_network_delayed callback,const bool ignUpdFail);
	void stop_nw_delay_monitor_task();
	
	void set_callback_camera_open_finished(camera::ycam3d::f_camera_open_finished callback);
	
	void set_callback_camera_disconnect(camera::ycam3d::f_camera_disconnect callback);
	
	void set_callback_camera_closed(camera::ycam3d::f_camera_closed callback);
	
	void set_callback_capture_img_received(camera::ycam3d::f_capture_img_received callback);
	
	void set_callback_pattern_img_received(camera::ycam3d::f_pattern_img_received callback);
	
};
