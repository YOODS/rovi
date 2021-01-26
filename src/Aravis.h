#pragma once
#include <arv.h>
#include <stdint.h>
#include <string>
#include <sstream>

#include "YCAM3D.h"


//2020/09/15 add by hato -------------------- start --------------------
namespace aravis{
namespace ycam3d{
	constexpr int CAM_EXPOSURE_TIME_DEFAULT   = 8300;
	constexpr int CAM_EXPOSURE_TIME_MAX       = 30000;
	constexpr int CAM_EXPOSURE_TIME_MIN       = 1000;
	
	constexpr int CAM_DIGITAL_GAIN_DEFAULT   = 0;
	constexpr int CAM_DIGITAL_GAIN_MAX       = 100;
	constexpr int CAM_DIGITAL_GAIN_MIN       = 0;
	
	constexpr int CAM_ANALOG_GAIN_DEFAULT   = 0;
	constexpr int CAM_ANALOG_GAIN_MAX       = 6;
	constexpr int CAM_ANALOG_GAIN_MIN       = 0;
	
	constexpr int PROJ_EXPOSURE_TIME_DEFAULT = 8333;
	constexpr int PROJ_EXPOSURE_TIME_MIN     = 0;      //?
	constexpr int PROJ_EXPOSURE_TIME_MAX     = 9999999;//?
	
	constexpr int PROJ_INTENSITY_DEFAULT = 100;
	constexpr int PROJ_INTENSITY_MIN     = 0;
	constexpr int PROJ_INTENSITY_MAX     = 255;
	
	constexpr int PROJ_FLASH_INTERVAL_DEFAULT = 10;
	//constexpr int PROJ_FLASH_INTERVAL_MIN = 0;  //?
	//constexpr int PROJ_FLASH_INTERVAL_MAX = 255;//?
	
	//constexpr int PATTERN_CAPTURE_NUM  = PHSFT_CAP_NUM;
	
	struct ExposureTimeLevelSetting {
		const int min_lv;
		const int max_lv;
		const int default_lv;
		
		struct Param {
			const int cam_exposure_tm;
			const int proj_exposure_tm;
			const int cam_frame_rate;
			
			Param(const int a_cam_exposure_tm,const int a_proj_exposure_tm,const int a_cam_frame_rate):
				cam_exposure_tm(a_cam_exposure_tm),
				proj_exposure_tm(a_proj_exposure_tm),
				cam_frame_rate(a_cam_frame_rate)
			{
			}
			std::string to_string()const{
				std::stringstream ss;
				ss << "cam_exposure_tm=" << cam_exposure_tm;
				ss << ", proj_exposure_tm=" << proj_exposure_tm;
				ss << ", cam_frame_rate=" << cam_frame_rate;
				return ss.str();
			}
		};
		const std::vector<Param> params;
		
		ExposureTimeLevelSetting(const int a_min_lv,const int a_max_lv,const int a_def_lv,const std::vector<Param> &a_params):
			min_lv(a_min_lv),
			max_lv(a_max_lv),
			default_lv(a_def_lv),
			params(a_params)
		{
		}
		const Param *get_param(const int lv)const{
			if( lv < min_lv || max_lv < lv ){
				return nullptr;
			}
			return params.data()+lv;
		}
		
	};
}
}
//2020/09/15 add by hato --------------------  end  --------------------


struct OnRecvImage {
	/**
	* @brief 画像データコールバック関数型
	* @param[out] camno 0～
	* @param[out] frmidx 0～12
	* @param[out] width 幅
	* @param[out] height 高さ
	* @param[out] color(0:mono,1:color)
	* @param[out] mem データアドレス
	*/
	virtual void operator()(int camno, int frmidx, int width, int height, int color, void *mem) = 0;
};

struct OnLostCamera {
	virtual void operator()(int camno) = 0;
};

class Aravis
{
	static int static_camno_;
	int camno_;
	int resolution_;
	int ncam_;
	ArvCamera *camera_;
	ArvDevice *device_;
	ArvStream *stream_;
	int payload_;		//buffer size (2 cams)
	int width_,height_;	//whole image size (2 cams)
	int color_;
	int frame_index_;
	pthread_cond_t cap_cond_;
	pthread_mutex_t cap_mutex_;

	ArvBufferStatus buffer_status_;
	bool lost_;
	char name_[64];
	int64_t packet_delay_;
	uint16_t version_;
	
	//2020/11/09 comment by hato -------------------- start --------------------
	//YCAM_TRIG trigger_mode_;
	//2020/11/09 comment by hato --------------------  end  --------------------
		
	//2020/09/16 add by hato -------------------- start --------------------
	const aravis::ycam3d::ExposureTimeLevelSetting *m_expsr_tm_lv_setting_;
	int m_expsr_tm_lv;
	//2020/09/16 add by hato --------------------  end  --------------------
	
	//2020/11/05 add by hato -------------------- start --------------------
	YCAM_PROJ_PTN cur_proj_ptn_;
	//2020/11/05 add by hato --------------------  end  --------------------
	
	//Aravis control
	static void on_new_buffer(ArvStream *stream, void *arg);
	static void on_control_lost(ArvGvDevice *gv_device, void *arg);

	//PROJ
	bool uart_write(const char *cmd);
	bool uart_write(char command, int data);
	bool uart_write(char command, const char *data);
	void uart_flush();
	int projector_value(const char *key_str, std::string *str = 0);	//診断メッセージから値を取得
	bool projector_wait();
	//2020/09/25 add by hato -------------------- start --------------------
	bool uart_cmd(const char *cmd,const int sleep_ms = 0);
	bool uart_cmd(const char command,const int val,const int slee_ms = 0);
	bool uart_cmd(const char command,const char *val,const int slee_ms = 0);
	std::string uart_read();
	//2020/09/25 add by hato --------------------  end  --------------------
	
	//YCAM
	uint32_t reg_read(uint64_t reg);
	bool reg_write(uint64_t reg, int value);
	
	//Genicam
	std::string get_node_str(const char *feature);
	int64_t get_node_int(const char *feature);
	bool set_node_int(const char *feature, int64_t value);
	double get_node_float(const char *feature);
	std::string get_description(const char *feature);

	//callback
	OnRecvImage *on_image_;
	uint8_t *out_;
	OnLostCamera *on_lost_;
	
	//2020/11/05 modified by hato -------------------- start --------------------
	enum ProjectorEnabled{
		Proj_Disabled = 0,
		Proj_Enabled = 2
	};
	//ProjectorEnabled cur_proj_enabled_;
	int cur_proj_intensity_;
	
	int pset_validate(void);
	void pset_stopgo(ProjectorEnabled n,const bool shortWait=false);
	//2020/11/05 modified by hato --------------------  end  --------------------
	
	//2020/11/26 moved by hato -------------------- start --------------------
	bool setProjectorExposureTime(int value);
	int projectorExposureTime();
	//2020/11/26 moved by hato --------------------  end  --------------------
	
public:
	Aravis(YCAM_RES res = YCAM_RES_SXGA, int ncam = 1);
	~Aravis();
	bool openCamera(const char *name=0, const int packet_size=0);	//name or ip address
	bool openStream(int nbuf=10);
	void destroy();
	bool capture(unsigned char *data, float timeout_sec=0.0f);
	//
	void setPacketDelay(int64_t delay);	//us
	int64_t packetDelay(){return packet_delay_;}
	//
	char *name(){return name_;}
	int frameSize(){return payload_;}
	void imageSize(int *width, int *height);
	int width(){return width_;}
	int height(){return height_;}
	int  bytesPerPixel(){payload_ / (width_ * height_);}
	int cameraNo()const { return camno_; }
	
	//2020/09/16 add by hato -------------------- start --------------------
	bool get_exposure_time_level(int *val)const;
	bool get_exposure_time_level_default(int *val)const;
	bool get_exposure_time_level_min(int *val)const;
	bool get_exposure_time_level_max(int *val)const;
	bool set_exposure_time_level(const int val);
	//2020/09/16 add by hato --------------------  end  --------------------
		
	int exposureTime();
	int gainA();
	int gainD();
	
	//2021/01/26 add by hato -------------------- start --------------------
	int getHeartBeatTimeout();
	bool setHeartBeatTimeout(const int val);
	//2021/01/26 add by hato --------------------  end  --------------------
	
	//2020/11/09 comment by hato -------------------- start --------------------
	//bool setExposureTime(int value);
	//2020/11/09 comment by hato --------------------  end  --------------------
	bool setGainA(int value);
	bool setGainD(int value);
	//
	bool isLost(){return lost_;}
	bool isAsync(){ return (VER_ACAP <= version_); }
	
	//2020/11/09 comment by hato -------------------- start --------------------
	//YCAM_TRIG triggerMode(){ return trigger_mode_; }
	//bool setTriggerMode(YCAM_TRIG tm);
	//2020/11/09 comment by hato --------------------  end  --------------------
	
	//2020/10/09 modified by hato -------------------- start --------------------
	//bool trigger(YCAM_PROJ_MODE mode);
	bool trigger(YCAM_PROJ_MODE mode);
	//2020/10/09 modified by hato --------------------  end  --------------------
	//
	
	//2020/11/06 add by hato -------------------- start --------------------
	int getCaptureNum()const;
	//2020/11/06 add by hato --------------------  end  --------------------
	
	YCAM_PROJ_PTN getProjectorPattern() const{
		return cur_proj_ptn_;
	}
	
	//2020/11/30 modified by hato -------------------- start --------------------
	bool setProjectorPattern(YCAM_PROJ_PTN ptn,const bool shortWait);
	//2020/11/30 modified by hato --------------------  end  --------------------

	bool setProjectorIntensity(int value);
	//2020/11/05 modified by hato -------------------- start --------------------
	//int projectorIntensity();
	int projectorIntensity()const {
		return cur_proj_intensity_;
	}
	//2020/11/05 modified by hato --------------------  end --------------------

	//2020/11/26 moved by hato -------------------- start --------------------
	//bool setProjectorExposureTime(int value);
	//int projectorExposureTime();
	//2020/11/26 moved by hato --------------------  end  --------------------

	bool setProjectorFlashInterval(int value);
	int projectorFlashInterval();
	
	//2020/11/05 modified by hato -------------------- start --------------------
	//bool isProjectorEnabled()const{
	//	return cur_proj_enabled_== Proj_Enabled;
	//}
	//bool setProjectorEnabled(const bool enabled);
	//2020/11/05 modified by hato --------------------  end  --------------------
	
	//2020/11/10 add by hato -------------------- start --------------------
	int getTemperature();
	//2020/11/10 add by hato --------------------  end  --------------------
	
	//utilities
	std::string uart_dump();
	bool upload_camparam(YCAM_RES reso, YCAM_SIDE side, const char *yaml_path);

	/**
	* @brief 画像データ取得時のコールバック関数設定
	* @param[in] onRecvImage コールバック関数
	* @param[out] out 画像バッファアドレス(固定)
	*/
	void addCallbackImage(OnRecvImage *onRecvImage, uint8_t *out);
	/**
	* @brief カメラロスト時のコールバック関数設定
	* @param[in] onLost コールバック関数
	*/
	void addCallbackLost(OnLostCamera *onLost);
};
