#include "CameraYCAM3D.hpp"

#include <map>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <chrono>

#include <ros/ros.h>
#include "ElapsedTimer.hpp"

#define DEBUG_DETAIL

#define LOG_HEADER "(camera) "

namespace camera{
	namespace ycam3d{
		const unsigned char YCAM3D_RESET_CMD[]      = { 0xD7,0x00,0x40,0x00 };
		const unsigned char YCAM3D_RESET_REPLY_OK[] = { 0xD7,0x01,0x41,0x00 };
		const int YCAM3D_RESET_REPLY_WAIT_INTERVAL = 1000;
		const int YCAM3D_RESET_UDP_PORT = 0xF000;
		const int YCAM3D_RESET_INTERVAL = 5;
		const int YCAM3D_RESET_AFTER_WAIT = 12;
		const int YCAM3D_RESET_TIMEOUT = 1000;
		
		/*
		void start_ycam3d_reset(const char *ipaddr){
			while( ! reset_ycam3d(ipaddr) ){
				sleep(YCAM3D_RESET_INTERVAL);
			}
			
			for(int i=YCAM3D_RESET_AFTER_WAIT;i>0;--i){
				ROS_INFO(LOG_HEADER"camera restarting ...  wait %2d sec",i);
				sleep(1);
			}
		}*/

		bool reset_ycam3d(const char *ipaddr){
			int sock;
			struct sockaddr_in addr;
			sock = socket(AF_INET, SOCK_DGRAM, 0);
			addr.sin_family = AF_INET;
			addr.sin_addr.s_addr = inet_addr(ipaddr);
			addr.sin_port = htons(YCAM3D_RESET_UDP_PORT);

			bind(sock, (const struct sockaddr *)&addr, sizeof(addr));
			//printf("send sock=%d ipaddr=%s port=%d(%X) \n",sock, ipaddr, port, port);

			ROS_INFO(LOG_HEADER"camera reset start. ipaddr=%s",ipaddr);
			const int cmd_size=sizeof(YCAM3D_RESET_CMD);
			const int slen=sendto(sock, YCAM3D_RESET_CMD, cmd_size, 0, (struct sockaddr *)&addr, sizeof(addr));
			if( slen != cmd_size ){
				ROS_ERROR(LOG_HEADER"camera reset command send failed.");
				return false;
			}
			
			//non-blocking
			int val = 1;
			ioctl(sock, FIONBIO, &val);
			
			const int reply_data_len = sizeof(YCAM3D_RESET_REPLY_OK);
			
			unsigned char reply[sizeof(YCAM3D_RESET_REPLY_OK)]={0};
			
			//printf("recv start. rsock=%d buf_size=%d\n",sock, reply_data_len);
			
			struct sockaddr_in from_addr;
			socklen_t sin_size;
			std::chrono::system_clock::time_point start_tm = std::chrono::system_clock::now();
			int elapsed_ms = 0;
			while( true ){
				//const ssize_t rlen=recvfrom(sock, reply, sizeof(reply), 0,(struct sockaddr *)&from_addr, &sin_size);
				const ssize_t rlen=recv(sock, reply, sizeof(reply), 0 );
				//printf("rlen=%ld\n",rlen);
				//for(int i=0;i<reply_data_len;++i){
				//	printf("rbuf[%d] %2X\n",i,reply[i]);
				//}
				if( rlen != reply_data_len ){
					usleep(YCAM3D_RESET_REPLY_WAIT_INTERVAL);
				}else{
					break;
				}
				elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_tm).count();
				//printf("elapsed_ms=%d, timeout=%d\n",elapsed_ms,timeout);
				if(elapsed_ms > YCAM3D_RESET_TIMEOUT ){
					ROS_ERROR(LOG_HEADER"camera reset timeout occured.");
					break;
				}
			}
			int matchCount=0;
			for(int i=0;i<reply_data_len;++i){
				//printf("[%d] %2X %2X\n",i,YCAM3D_RESET_REPLY_OK[i],reply[i]);
				if(YCAM3D_RESET_REPLY_OK[i] == reply[i]){
					matchCount++;
				}else{
					break;
				}
			}
			//printf("matchCount=%d reply_data_len=%d",matchCount,reply_data_len);
			const bool result=matchCount == reply_data_len;
			if( ! result ){
				ROS_ERROR(LOG_HEADER"camera reset failed.");
			}else{
				ROS_INFO(LOG_HEADER"camera reset success.");
			}
			
			//for(int i=0;i<sizeof(reply);++i){
			//	printf("recv [%d] %3d (%2X)\n",i,0xFF & reply[i],0xFF & reply[i]);
			//}
			return result;
		}
	}
}

namespace {
	const std::map<std::string,YCAM_RES> YCAM_RES_MAP = { {"SXGA",YCAM_RES_SXGA},{"VGA",YCAM_RES_VGA} };
	std::map<YCAM_PROJ_PTN,std::string> PROJ_PTN_MAP = {
		{YCAM_PROJ_PTN_FIXED,"Fixed"},
		{YCAM_PROJ_PTN_PHSFT,"PhaseShift"},
		{YCAM_PROJ_PTN_STROBE,"Strobe"},
		{YCAM_PROJ_PTN_FOCUS,"Focus"},
		{YCAM_PROJ_PTN_PHSFT_3,"PhaseShift3"}
	};
	
	//接続試行して次のトライまでの時間。open処理の時間があるので一定間隔にはならない
	const int CAMERA_AUTO_CONNECT_WAIT_TM = 1000 * 1000;//1sec
	const int CAMERA_AUTO_CONNECT_INTERVAL = 5;//sec 
	const int CAMERA_AUTO_CONNECT_RETRY_MAX = 10;
	
	const int CAMERA_STREAM_BUF_SIZE = 50;
	
	const int CAMERA_IMG_COLOR_CH = 1;
	const int CAPTURE_TIMEOUT_PERIOD_DEFAULT = 3; //sec
	const int TRIGGER_TIMEOUT_PERIOD_DEFAULT = 5; //sec
	
	const int STROBE_CAPT_FRAME_INDEX = 1;
	
	
}

CameraYCAM3D::CameraImageReceivedCallback::CameraImageReceivedCallback(CameraYCAM3D *obj):OnRecvImage(),
	m_self(obj)
{
}

void CameraYCAM3D::CameraImageReceivedCallback::operator()(int camno, int frmidx, int lr_width, int height, int color, void *mem)
{
#ifdef DEBUG_DETAIL
	ROS_WARN(LOG_HEADER"on camera image received. camno=%d frmidx=%d lr_width=%d height=%d color=%d\n",camno,frmidx,lr_width, height, color);
#endif
	//デバッグ:raw画像保存
	//cv::Mat img(cv::Size(lr_width, height), CV_8UC1,mem );
	//char path[256];
	//sprintf(path,"/tmp/raw%02d.pgm",frmidx);
	//cv::imwrite(path,img);
	
	const int captNum=m_self->m_imgs_left.size();

	if( m_self->m_camno != camno ){
		fprintf(stderr,LOG_HEADER"#%d different camera no image received. camno=%d\n",m_self->m_camno,camno);
		return;
		
	}else if( color != 0 ){
		fprintf(stderr,LOG_HEADER"#%d error: unsupported image format. color=%d\n",m_self->m_camno,color);
		return;
		
	}else if( lr_width  == m_self->width() * 2 || height != m_self->height() ){
		fprintf(stderr,LOG_HEADER"#%d error: unsupported image size. width=%d height=%d\n",m_self->m_camno,lr_width,height);
		return;
		
	}else if( frmidx < 0 || captNum <= frmidx - 1 ){
		fprintf(stderr,LOG_HEADER"#%d error: frame index is out of range. frmidx=%d capt_num=%d\n",m_self->m_camno,frmidx,captNum);
		return;
		
	}else if( m_self->m_capt_stat.load() == CaptStat_Ready ){
		fprintf(stderr,LOG_HEADER"#%d received untarget capture image. ignored. frmidx=%d\n",m_self->m_camno,frmidx);
		return;
		
	}
	
	ElapsedTimer tmr;
	bool capt_wait_done=false;
	{
		// ********** m_img_update_mutex LOCKED **********
		std::lock_guard<std::timed_mutex> locker(m_self->m_img_update_mutex);
		camera::ycam3d::CameraImage *img_buf_l = m_self->m_imgs_left.data()+ frmidx;
		camera::ycam3d::CameraImage *img_buf_r = m_self->m_imgs_right.data()+ frmidx;
		img_buf_l->dt = img_buf_r->dt = std::chrono::system_clock::now(); 
		
		const int lr_img_step = lr_width;
		const int img_step = lr_img_step / 2;
		
		if( img_step != img_buf_l->step ){
			fprintf(stderr,LOG_HEADER"error: image step size wrong. img_step=%d, img_buf_l_step=%d\n",img_step,img_buf_l->step);
			return;
			// ********** m_img_update_mutex UNLOCKED **********
		}
		
		img_buf_l->result = img_buf_r->result = true;
		
		unsigned char *pmem = (unsigned char*)mem;
		for( int y = 0 ; y < height ; ++y ){
			pmem = ((unsigned char*)mem) + y * lr_img_step;
			memcpy(img_buf_l->data.data() + y * img_step , pmem             ,img_step);
			memcpy(img_buf_r->data.data() + y * img_step , pmem + img_step  ,img_step);
		}
		
		m_self->m_img_recv_flags[frmidx]=true;
		
#ifdef DEBUG_DETAIL
		std::stringstream recv_flags_str;
		for( int i = 0 ; i < m_self->m_img_recv_flags.size() ; ++i ){
			recv_flags_str << (m_self->m_img_recv_flags[i]?"*":"_");
		}
		//ROS_WARN(LOG_HEADER"#%d camera img received. frmidx=%2d, proc_tm=%3d ms,capt_num=%d, recv_flags=%s\n",
		//	m_self->m_camno,frmidx, tmr.elapsed_ms(), captNum, recv_flags_str.str().c_str() );
#endif
		
		if( std::count( m_self->m_img_recv_flags.begin(), m_self->m_img_recv_flags.end(), true ) ==  captNum ){
#ifdef DEBUG_DETAIL
			ROS_WARN(LOG_HEADER"#%d pattern image all received.\n",m_self->m_camno);
#endif
			capt_wait_done=true;
		}
		// ********** m_img_update_mutex UNLOCKED **********
	}
	
	if(capt_wait_done){
#ifdef DEBUG_DETAIL
		ROS_WARN(LOG_HEADER"#%d capture wait done.\n",m_self->m_camno);
#endif
		m_self->m_capt_finish_wait_mutex.unlock();
		// ********** m_capt_finish_wait_mutex UNLOCKED **********
	}

}

CameraYCAM3D::CameraDisconnectCallbck::CameraDisconnectCallbck(CameraYCAM3D *obj):OnLostCamera(),
	m_self(obj)
{
}

void CameraYCAM3D::CameraDisconnectCallbck::operator()(int camno)
{
	if( m_self->m_camno != camno ){
		ROS_ERROR(LOG_HEADER"#%d another camera no has been disconnected. camno=%d",m_self->m_camno,camno);
		return;
	}
	
	m_self->m_open_stat.store(false);
	ROS_ERROR(LOG_HEADER"#%d disconnect !!!",m_self->m_camno);
	m_self->start_auto_connect();
	
	if( m_self->m_callback_cam_disconnect ){
		m_self->m_callback_cam_disconnect();
	}
}

CameraYCAM3D::CameraYCAM3D():
	m_camno(-1),
	m_capt_stat(CaptStat_Ready),
	m_ycam_res(YCAM_RES_SXGA),
	m_auto_connect_abort(false),
	m_open_stat(false),
	m_on_image_received(this),
	m_on_disconnect(this),
	m_capture_timeout_period(CAPTURE_TIMEOUT_PERIOD_DEFAULT),
	m_trigger_timeout_period(TRIGGER_TIMEOUT_PERIOD_DEFAULT),
	m_cancel_delay_mon(false),
	m_pre_heart_beat_val(0)
{
}

CameraYCAM3D::~CameraYCAM3D(){
	printf(LOG_HEADER"destructor start.\n");
	close();
	printf(LOG_HEADER"destructor end.\n");
}

int CameraYCAM3D::width()const{
	if( ! m_arv_ptr ){
		return -1;
	}
	
	return m_arv_ptr->width();
}

int CameraYCAM3D::height()const{
	if( ! m_arv_ptr ){
		return -1;
	}
	
	return m_arv_ptr->height();
}

void CameraYCAM3D::set_capture_timeout_period(const int timeout){
	m_capture_timeout_period = timeout;
}
	
void CameraYCAM3D::set_trigger_timeout_period(const int timeout){
	m_trigger_timeout_period = timeout;
}

bool CameraYCAM3D::reset_image_buffer(const int capt_num){
	std::lock_guard<std::timed_mutex> locker(m_img_update_mutex);
	// ********** m_img_update_mutex LOCKED **********
	
	if(width() < 0 || height() < 0 ){
		ROS_ERROR(LOG_HEADER"#%d error:camera image buffer allocate failed. unkown image size.", m_camno);
	}else{
		m_img_recv_flags.assign(capt_num,false);
	
		camera::ycam3d::CameraImage defaultVal;
		defaultVal.result=false;
		defaultVal.width =width()/2;
		defaultVal.height =height();
		defaultVal.color_ch = CAMERA_IMG_COLOR_CH;
		defaultVal.step = defaultVal.width * defaultVal.color_ch;
		
		if( ! defaultVal.alloc() ){
			ROS_ERROR(LOG_HEADER"#%d error:camera image buffer allocate failed. unknown image size.", m_camno);
		}else{
			m_imgs_left.assign(capt_num,defaultVal);
			m_imgs_right.assign(capt_num,defaultVal);
		}
	}
	
	return true;
	// ********** m_img_update_mutex UNLOCKED **********
}

void CameraYCAM3D::set_callback_camera_open_finished(camera::ycam3d::f_camera_open_finished callback){
	m_callback_cam_open_finished = callback;
}

void CameraYCAM3D::set_callback_camera_disconnect(camera::ycam3d::f_camera_disconnect callback){
	m_callback_cam_disconnect = callback;
}

void CameraYCAM3D::set_callback_camera_closed(camera::ycam3d::f_camera_closed callback){
	m_callback_cam_closed = callback;
}

void CameraYCAM3D::set_callback_capture_img_received(camera::ycam3d::f_capture_img_received callback){
	m_callback_capt_img_recv = callback;
}

void CameraYCAM3D::set_callback_pattern_img_received(camera::ycam3d::f_pattern_img_received callback){
	m_callback_trig_img_recv = callback;
}

bool CameraYCAM3D::init(const std::string &camera_res){
	if( m_arv_ptr ){
		ROS_ERROR(LOG_HEADER"#%d error:already initialized.", m_camno);
		return false;
	}
	
	{
		std::map<std::string, YCAM_RES>::const_iterator ite = YCAM_RES_MAP.find(camera_res);
		if (ite == YCAM_RES_MAP.end()){
			ROS_ERROR(LOG_HEADER"error:camera resolution value not found.");
			return false;
		}
		m_ycam_res = ite->second ;
	}
	
	m_arv_ptr.reset(new Aravis(m_ycam_res));
	
	m_camno = m_arv_ptr->cameraNo();
	
	return true;
}

bool CameraYCAM3D::is_open()const{
	return m_open_stat.load();
}
	
void CameraYCAM3D::open() {
	// ********** m_camera_mutex LOCKED **********
	std::lock_guard<std::timed_mutex> locker(m_camera_mutex);
	if( ! m_arv_ptr ){
		ROS_ERROR(LOG_HEADER"#%d error:camera is null.", m_camno);
		return;
		// ********** m_camera_mutex UNLOCKED **********
	}
	
	//if( ! m_arv_ptr->isLost() ){
	//	ROS_WARN(LOG_HEADER"#%d camera is already connected.", m_camno);
	//	return;
	//	// ********** m_camera_mutex UNLOCKED **********
	//}
	m_open_stat.store(false);
	
	ROS_INFO(LOG_HEADER"#%d open start.", m_camno);
	if( ! m_arv_ptr->openCamera() ){
		ROS_ERROR(LOG_HEADER"#%d error:open failed.", m_camno);
	}else{
		ROS_INFO(LOG_HEADER"#%d open success.", m_camno );
		//ROS_INFO(LOG_HEADER"#%d image: size=%d x %d, frame_size=%d", m_camno, m_arv_ptr->width(), m_arv_ptr->height(), m_arv_ptr->frameSize() );
		//ROS_INFO(LOG_HEADER"#%d params: expsr_tm=%d gain_a=%d gain_d=%d", m_camno,m_arv_ptr->exposureTime(), m_arv_ptr->gainA(), m_arv_ptr->gainD() );
		
		//m_arv_ptr->setProjectorPattern(YCAM_PROJ_PTN_PHSFT);
		
		ROS_INFO(LOG_HEADER"#%d stream open start.", m_camno);
		if( ! m_arv_ptr->openStream(CAMERA_STREAM_BUF_SIZE) ){
			ROS_ERROR(LOG_HEADER"#%d error:stream open failed.", m_camno);
			
		}else{
			ROS_INFO(LOG_HEADER"#%d stream open success.", m_camno);
			if( m_arv_img_buf.empty() ){
				m_arv_img_buf.assign(m_arv_ptr->frameSize(),0);
				
				ROS_INFO(LOG_HEADER"#%d image buffer allocated. size=%d", m_camno, (int)m_arv_img_buf.size());
				
				m_arv_ptr->addCallbackImage(&m_on_image_received, m_arv_img_buf.data());
				m_arv_ptr->addCallbackLost(&m_on_disconnect);
			}
			
			reset_image_buffer(0);
			m_open_stat.store(true);
		}
	}
	
	if( m_callback_cam_open_finished ){
		m_callback_cam_open_finished(m_open_stat.load());
	}
	// ********** m_camera_mutex UNLOCKED **********
}

bool CameraYCAM3D::is_auto_connect_running(){
	bool running=false;
	if( ! m_auto_connect_mutex.try_lock_for(std::chrono::seconds(0))){
		running=true;
	}else{
		m_auto_connect_mutex.unlock();
	}
	return running;
}


void CameraYCAM3D::start_auto_connect(const std::string ipaddr){
	
	if( ! ipaddr.empty() ){
		ROS_INFO("auto connect target ipaddr=%s",ipaddr.c_str());
		m_auto_connect_ipaddr=ipaddr;
	}
	
	if( m_open_stat.load() ){
		ROS_WARN(LOG_HEADER"#%d camera is already connected. [1] ", m_camno);
		
	}else if( ! m_auto_connect_mutex.try_lock_for(std::chrono::seconds(0)) ){
		// ******** auto_connect_mutex UNLOCKED ******** 
		ROS_WARN(LOG_HEADER"#%d auto connect already started.", m_camno);
		
	}else{
		// ******** auto_connect_mutex  LOCKED  ******** 
		ROS_INFO(LOG_HEADER"#%d auto connect start.", m_camno);
		
		m_auto_connect_abort = false;
		m_auto_connect_thread = std::thread([this,ipaddr](){
			ROS_INFO(LOG_HEADER"#%d auto connect loop start. %s", m_camno,ipaddr.c_str());
			if( m_open_stat.load() ){
				ROS_WARN(LOG_HEADER"#%d camera is already connected. [2]", m_camno);
			}else{
				
				if(m_ros_err_pub){ m_ros_err_pub("camera reset start.");}
				
				while( ! camera::ycam3d::reset_ycam3d(ipaddr.c_str()) && ! m_auto_connect_abort ){
						sleep(camera::ycam3d::YCAM3D_RESET_INTERVAL);
				}
				
				if( ! m_auto_connect_abort ){
					if(m_ros_err_pub){ m_ros_err_pub("camera reset success.");}
					for(int i=camera::ycam3d::YCAM3D_RESET_AFTER_WAIT; i > 0  && ! m_auto_connect_abort ; --i){
						ROS_INFO(LOG_HEADER"camera restarting ...  wait %2d sec",i);
						sleep(1);
					}
				}
				
				int retry=0;
				while( ! m_auto_connect_abort ){
					
					retry++;
					
					ROS_WARN(LOG_HEADER"#%d trying to connect the camera... [%d]", m_camno, retry );
					
					if(m_ros_err_pub){ m_ros_err_pub("camera open try start.");}
					open();
					
					
					for(int i = 0 ; i < CAMERA_AUTO_CONNECT_INTERVAL && ! m_auto_connect_abort && ! m_open_stat.load() ; ++i ){
						usleep(CAMERA_AUTO_CONNECT_WAIT_TM);
					}
					
					if( m_auto_connect_abort ){
						ROS_WARN(LOG_HEADER"#%d auto connect aborted.", m_camno);
						break;
					}else if( m_open_stat.load() ){
						if(m_ros_err_pub){ m_ros_err_pub("camera opened.");}
						break;
					}else if( retry >= CAMERA_AUTO_CONNECT_RETRY_MAX ){
						ROS_ERROR(LOG_HEADER"auto connect retry limit has been exceeded.");
						if(m_callback_auto_lm_excd){
							m_callback_auto_lm_excd();
						}
						retry = 0;
					}
				}
			}
			ROS_INFO(LOG_HEADER"#%d auto connect loop finished.", m_camno);
			
			m_auto_connect_mutex.unlock();
			// ******** auto_connect_mutex UNLOCKED ******** 
		});
		m_auto_connect_thread.detach();
	}
}

void CameraYCAM3D::close(){
	{
		stop_nw_delay_monitor_task();
	}
	{
		//自動接続
		ROS_INFO(LOG_HEADER"#%d auto connect finish wait start.", m_camno);
		if( is_auto_connect_running() ){
			m_auto_connect_abort = true;
		}
		std::lock_guard<std::timed_mutex> locker_auto_con(m_auto_connect_mutex);
		// ********** m_auto_connect_mutex LOCKED **********
		ROS_INFO(LOG_HEADER"#%d auto connect finish wait end.", m_camno);
		
		//撮影画像受信完了待ち
		ROS_INFO(LOG_HEADER"#%d capture finish wait start.", m_camno);
		std::lock_guard<std::timed_mutex> locker_capt(m_capt_finish_wait_mutex);
		// ********** m_capt_finish_wait_mutex LOCKED **********
		ROS_INFO(LOG_HEADER"#%d capture finish wait end.", m_camno);
		
		// ********** m_camera_mutex LOCKED **********
		std::lock_guard<std::timed_mutex> locker_cam(m_camera_mutex);
		
		m_open_stat.store(false);
		
		if( m_arv_ptr ){
			ROS_INFO(LOG_HEADER"#%d camera destroy start.", m_camno);
			m_arv_ptr->destroy();
			ROS_INFO(LOG_HEADER"#%d camera destroy end.", m_camno);
		}
		
		// ********** m_auto_connect_mutex UNLOCKED **********
		// ********** m_capt_finish_wait_mutex UNLOCKED **********
		// ********** m_camera_mutex UNLOCKED **********
	}
	
	if( m_callback_cam_closed ){
		m_callback_cam_closed();
	}
}

bool CameraYCAM3D::is_busy(){
	bool busy=false;
	
	if( m_capt_stat.load() != CaptStat_Ready){
		busy = true;
	}else{
		if(m_camera_mutex.try_lock_for(std::chrono::seconds(0))){
			m_camera_mutex.unlock();
		}else{
			busy = true;
		}
	}
	
	return busy;
}

bool CameraYCAM3D::capture(const bool strobe){
	
	if( ! m_arv_ptr ){
		ROS_ERROR(LOG_HEADER"#%d error:camera is null.", m_camno);
		return false;
	}else if( m_arv_img_buf.empty() ){
		ROS_ERROR(LOG_HEADER"#%d error:camera image buffer is null.", m_camno);
		return false;
	}else if( ! is_open() ){
		ROS_ERROR(LOG_HEADER"#%d error:camera is not opened", m_camno);
		return false;
	}
	
	ElapsedTimer capt_tmr;
	m_camera_mutex.lock();
	// ********** m_camera_mutex LOCKED **********
	
	m_capt_stat.store(CaptStat_Single);
	
	m_capture_thread = std::thread([this,capt_tmr,strobe](){
		
#ifdef DEBUG_DETAIL
		ROS_INFO(LOG_HEADER"#%d capture start. timeout=%d sec, strobe=%d", m_camno,m_capture_timeout_period,strobe);
#endif
		std::lock_guard<std::timed_mutex> locker(m_camera_mutex,std::adopt_lock);
		
		m_capt_finish_wait_mutex.lock();
		// ********** m_capt_finish_wait_mutex LOCKED **********
		
		if( m_arv_ptr->getProjectorPattern() != YCAM_PROJ_PTN_STROBE ){
			ROS_INFO(LOG_HEADER"#%d projector pattern change. ptn=strobe",m_camno);
			if( ! m_arv_ptr->setProjectorPattern(YCAM_PROJ_PTN_STROBE,true)){
				ROS_ERROR(LOG_HEADER"#%d error:projector pattern change failed. ptn=%d (%s)", m_camno,YCAM_PROJ_PTN_STROBE, PROJ_PTN_MAP[YCAM_PROJ_PTN_STROBE].c_str() );
			}
		}
		
		const int curProjIntensity = m_arv_ptr->projectorIntensity();
#ifdef DEBUG_DETAIL
		ROS_INFO(LOG_HEADER"#%d cur proj intensity. val=%d",m_camno, curProjIntensity);
#endif
		if( ! strobe ){
			m_arv_ptr->setProjectorIntensity(0);
		}
		
		
		const int captNum=m_arv_ptr->getCaptureNum();
		reset_image_buffer(captNum);
		
		int curExpsrLv=-1;
		m_arv_ptr->get_exposure_time_level(&curExpsrLv);
		
		if( ! m_arv_ptr->trigger(YCAM_PROJ_MODE_CAPT) ){
			ROS_ERROR(LOG_HEADER"#%d error:trigger call failed.", m_camno);
		}
		
#ifdef DEBUG_DETAIL
		ROS_INFO(LOG_HEADER"#%d capture end.", m_camno);
		
		ROS_INFO(LOG_HEADER"#%d capture image received wait start.", m_camno);
#endif
		
		const bool timeout_occured = ! m_capt_finish_wait_mutex.try_lock_for( std::chrono::seconds(m_capture_timeout_period) );
		// ********** m_capt_finish_wait_mutex LOCKED ?? **********		
		if( ! strobe ){
			m_arv_ptr->setProjectorIntensity(curProjIntensity);
		}
		
#ifdef DEBUG_DETAIL
		ROS_INFO(LOG_HEADER"#%d capture image received wait finshed. timeout=%d, elapsed=%d ms",
			m_camno,timeout_occured,capt_tmr.elapsed_ms());
#endif
		if( m_callback_capt_img_recv ){
			camera::ycam3d::CameraImage img_l;
			camera::ycam3d::CameraImage img_r;
			bool result=false;
			{
				std::lock_guard<std::timed_mutex> locker(m_img_update_mutex);
				// ********** m_img_update_mutex LOCKED **********
				if( m_imgs_left.size() == m_imgs_right.size() && m_imgs_left.size() == captNum ){
					img_l = m_imgs_left.at(0);
					img_r = m_imgs_right.at(0);
					if( ! img_l.valid() ){
						ROS_ERROR(LOG_HEADER"#%d error:left capture image is invalid.", m_camno);
					}else if( ! img_r.valid() ){
						ROS_ERROR(LOG_HEADER"#%d error:right capture image is invalid.", m_camno);
					}else{
						result=true;
					}
				}
				// ********** m_img_update_mutex UNLOCKED **********
			}
			
			if( timeout_occured ){
				ROS_ERROR(LOG_HEADER"#%d error:capture timeout.", m_camno);
				m_callback_capt_img_recv(false, capt_tmr.elapsed_ms(), img_l, img_r, true,curExpsrLv);
			}else{
				m_callback_capt_img_recv(result, capt_tmr.elapsed_ms(), img_l, img_r, false,curExpsrLv);
			}
		}
		
		m_capt_finish_wait_mutex.unlock();
		// ********** m_capt_finish_wait_mutex UNLOCKED **********
		m_capt_stat.store(CaptStat_Ready);
#ifdef DEBUG_DETAIL
		ROS_INFO(LOG_HEADER"#%d capture finished. elapsed=%d ms", m_camno,capt_tmr.elapsed_ms());
#endif
		// ********** m_camera_mutex UNLOCKED **********
	});
	m_capture_thread.detach();
	return true;
}

bool CameraYCAM3D::capture_pattern(const bool pcgenModeMulti,const bool ptnCangeWaitShort){
	if( ! m_arv_ptr ){
		ROS_ERROR(LOG_HEADER"#%d error:camera is null.", m_camno);
		return false;
	}else if( m_arv_img_buf.empty() ){
		ROS_ERROR(LOG_HEADER"#%d error:camera image buffer is null.", m_camno);
		return false;
	}else if( ! is_open() ){
		ROS_ERROR(LOG_HEADER"#%d error:camera is not opened", m_camno);
		return false;
	}
	
	//if ( m_capt_stat.load() != CaptStat_Ready ){
	//	ROS_ERROR(LOG_HEADER"#%d error:capture status is not ready. pattern capture failed.", m_camno);
	//	return false;
	//}
	
	ElapsedTimer capt_tmr;
	m_camera_mutex.lock();
	// ********** m_camera_mutex LOCKED **********
	
	m_capt_stat.store(CaptStat_Pattern);
	
	m_capture_thread = std::thread([this,capt_tmr,pcgenModeMulti,ptnCangeWaitShort](){
		
		ROS_INFO(LOG_HEADER"#%d pattern capture start. pcgenModeMulti=%d,timeout=%d sec", m_camno, pcgenModeMulti, m_trigger_timeout_period);
		
		m_capt_finish_wait_mutex.lock();
		// ********** m_capt_finish_wait_mutex LOCKED **********
		
		std::lock_guard<std::timed_mutex> locker(m_camera_mutex,std::adopt_lock);
		YCAM_PROJ_PTN ptn=YCAM_PROJ_PTN_PHSFT;
		if(pcgenModeMulti){
			ptn = YCAM_PROJ_PTN_PHSFT_3;
		}
		
		if( m_arv_ptr->getProjectorPattern() != ptn ){
			ROS_INFO(LOG_HEADER"#%d projector pattern change. ptn=%d (%s)",m_camno, ptn, PROJ_PTN_MAP[ptn].c_str());
			if( ! m_arv_ptr->setProjectorPattern(ptn,ptnCangeWaitShort) ){
				ROS_ERROR(LOG_HEADER"#%d error:projector pattern change failed. ptn=%d (%s) ",
					m_camno, ptn, PROJ_PTN_MAP[ptn].c_str());
			}else{
				ROS_INFO(LOG_HEADER"#%d projector pattern changed. ptn=%d (%s) elapsed=%d",
					m_camno, ptn, PROJ_PTN_MAP[ptn].c_str(),capt_tmr.elapsed_ms());
			}
		}
		
		const int captNum = m_arv_ptr->getCaptureNum();
		reset_image_buffer(captNum);
		ROS_INFO(LOG_HEADER"#%d capture_num=%d",m_camno,captNum);
		
		int curExpsrLv=-1;
		m_arv_ptr->get_exposure_time_level(&curExpsrLv);
		ROS_INFO(LOG_HEADER"#%d cur expsr_lv=%d",m_camno,curExpsrLv);
		
		ROS_INFO(LOG_HEADER"#%d projector trigger start. elapsed=%d ms",m_camno,capt_tmr.elapsed_ms());
		if( ! m_arv_ptr->trigger(YCAM_PROJ_MODE_CONT) ){
			ROS_ERROR(LOG_HEADER"#%d error:trigger call failed.", m_camno);
		}
		ROS_INFO(LOG_HEADER"#%d projector trigger finished. elapsed=%d ms",m_camno, capt_tmr.elapsed_ms());
		
		const bool timeout_occured = ! m_capt_finish_wait_mutex.try_lock_for( std::chrono::seconds(m_trigger_timeout_period) );
		// ********** m_capt_finish_wait_mutex LOCKED ?? **********
		ROS_INFO(LOG_HEADER"#%d pattern capture image received wait finshed. timeout=%d, elapsed=%d ms",
			m_camno, timeout_occured, capt_tmr.elapsed_ms());
		
		if( m_callback_trig_img_recv ){
			std::vector<camera::ycam3d::CameraImage> imgs_l;
			std::vector<camera::ycam3d::CameraImage> imgs_r;
			bool result=false;
			{
				std::lock_guard<std::timed_mutex> locker(m_img_update_mutex);
				// ********** m_img_update_mutex LOCKED **********
				imgs_l = m_imgs_left;
				imgs_r = m_imgs_right;
				int validImgNum = 0;
				if( imgs_l.size() == imgs_r.size() && imgs_l.size() == captNum ){
					
					result=true;
					
					for(int i = 0 ; i < imgs_l.size() ; ++i ){
						const camera::ycam3d::CameraImage *img_l = imgs_l.data() + i;
						const camera::ycam3d::CameraImage *img_r = imgs_r.data() + i;
						if( ! img_l->result || ! img_l->valid()  ){
							ROS_ERROR(LOG_HEADER"#%d error:pattern image is error. side=left, idx=%d", m_camno, i);
							result=false;
							break;
						}else if( ! img_r->result || ! img_r->valid()  ){
							ROS_ERROR(LOG_HEADER"#%d error:pattern image is error. side=right, idx=%d", m_camno, i);
							result=false;
							break;
						}
					}
				}
				// ********** m_img_update_mutex UNLOCKED **********
			}

#ifdef DEBUG_DETAIL
			ROS_INFO(LOG_HEADER"call back trig_img_recv start");
#endif
			if( timeout_occured ){
				ROS_ERROR(LOG_HEADER"#%d error:pattern capture timeout.", m_camno);
				m_callback_trig_img_recv(false, capt_tmr.elapsed_ms(), imgs_l, imgs_r, true ,curExpsrLv);
			}else{
				m_callback_trig_img_recv(result, capt_tmr.elapsed_ms(), imgs_l, imgs_r, false ,curExpsrLv);
			}
#ifdef DEBUG_DETAIL
			ROS_INFO(LOG_HEADER"call back trig_img_recv finished");
#endif
		}
		
		m_capt_finish_wait_mutex.unlock();
		// ********** m_capt_finish_wait_mutex UNLOCKED **********
		
		m_capt_stat.store(CaptStat_Ready);
#ifdef DEBUG_DETAIL
		ROS_INFO(LOG_HEADER"#%d pattern capture finished. elapsed=%d ms", m_camno, capt_tmr.elapsed_ms());
#endif
		// ********** m_camera_mutex UNLOCKED **********
	});
	m_capture_thread.detach();
	
	return true;
}

bool CameraYCAM3D::get_camera_param_int(const std::string &label,std::function<bool(int*)> func,int *val){
	if( ! m_arv_ptr ){
		ROS_ERROR(LOG_HEADER"#%d error:camera is null. name=%s", m_camno,label.c_str());
		return false;
	}else if( ! is_open() ){
		ROS_ERROR(LOG_HEADER"#%d error:camera is not opened. name=%s", m_camno,label.c_str());
		return false;
	}
	
	// ********** m_camera_mutex LOCKED **********
	std::lock_guard<std::timed_mutex> locker(m_camera_mutex);
	bool ret= func(val);
	return ret;
	// ********** m_camera_mutex UNLOCKED **********
}

bool CameraYCAM3D::set_camera_param_int(const std::string &label,std::function<bool(int)> func,const int val){
	if( ! m_arv_ptr ){
		ROS_ERROR(LOG_HEADER"#%d error:camera is null. name=%s", m_camno,label.c_str());
		return false;
	}else if( ! is_open() ){
		ROS_ERROR(LOG_HEADER"#%d error:camera is not opened. name=%s", m_camno,label.c_str());
		return false;
	}
	
	// ********** m_camera_mutex LOCKED **********
	std::lock_guard<std::timed_mutex> locker(m_camera_mutex);
	//return func(val);
	bool ret=func(val);
	return ret;
	// ********** m_camera_mutex UNLOCKED **********
}

bool CameraYCAM3D::get_exposure_time_level_default(int *val)const{
	if( ! m_arv_ptr ){
		ROS_ERROR(LOG_HEADER"#%d error:camera is null.", m_camno);
		return false;
	}
	return m_arv_ptr->get_exposure_time_level_default(val);
}

//exposure_time_level_min/maxだけ関数で取得するのはsxga,vgaで設定の数が異なる
bool CameraYCAM3D::get_exposure_time_level_min(int *val)const{
	if( ! m_arv_ptr ){
		ROS_ERROR(LOG_HEADER"#%d error:camera is null.", m_camno);
		return false;
	}
	return m_arv_ptr->get_exposure_time_level_min(val);
}

bool CameraYCAM3D::get_exposure_time_level_max(int *val)const{
	if( ! m_arv_ptr ){
		ROS_ERROR(LOG_HEADER"#%d error:camera is null.", m_camno);
		return false;
	}
	return m_arv_ptr->get_exposure_time_level_max(val);
}

bool CameraYCAM3D::get_exposure_time_level(int * val){
	return get_camera_param_int("digital_gain",[&](int *l_val) {
		return m_arv_ptr->get_exposure_time_level(l_val);
	},val);
}

bool CameraYCAM3D::set_exposure_time_level(const int val){
	return set_camera_param_int("exposure_time_level",[&](const int l_val) {
		return m_arv_ptr->set_exposure_time_level(l_val);
	},val);
}


#if 0
bool CameraYCAM3D::get_exposure_time(int *val){
	return get_camera_param_int("exposure_time",[&](int *l_val) {
		*l_val = m_arv_ptr->exposureTime();
		return *l_val >= 0;
	},val);
}

bool CameraYCAM3D::set_exposure_time(const int val){
	const bool result = set_camera_param_int("exposure_time",[&](const int l_val) {
		return m_arv_ptr->setExposureTime(l_val);
	},val);
	
	bool ret=false;
	int cval=-1;
	if( result && get_exposure_time(&cval) &&  cval == val){
		ret=true;
	}
	return ret;
}
#endif

bool CameraYCAM3D::get_gain_digital(int *val){
	return get_camera_param_int("digital_gain",[&](int *l_val) {
		*l_val =  m_arv_ptr->gainD();
		return *l_val >= 0;
	},val);
}

bool CameraYCAM3D::set_gain_digital(const int val){
	const bool result = set_camera_param_int("digital_gain",[&](const int l_val) {
		return m_arv_ptr->setGainD(l_val);
	},val);
	
	/* 正しい値が返ってこないのでノーチェック。
	bool ret=false;
	int cval=-1;
	if( result && get_gain_digital(&cval) &&  cval == val){
		ret=true;
	}
	return ret;
	*/
	return true;
}

bool CameraYCAM3D::get_gain_analog(int *val){
	return get_camera_param_int("analog_gain",[&](int *l_val) {
		*l_val =  m_arv_ptr->gainA();
		return *l_val >= 0;
	},val);
}

bool CameraYCAM3D::set_gain_analog(const int val){
	const bool result= set_camera_param_int("analog_gain",[&](const int l_val) {
		return m_arv_ptr->setGainA(l_val);
	},val);
	
	/* 正しい値が返ってこないのでノーチェック。
	bool ret=false;
	int cval=-1;
	if( result && get_gain_analog(&cval) &&  cval == val){
		ret=true;
	}
	return ret;
	*/
	return true;
}

#if 0
bool CameraYCAM3D::get_projector_exposure_time(int *val){
	return get_camera_param_int("proj_exposure_time",[&](int *l_val) {
		*l_val =  m_arv_ptr->gainA();
		return *l_val >= 0;
	},val);
}

bool CameraYCAM3D::set_projector_exposure_time(const int val){
	const bool result = set_camera_param_int("proj_exposure_time",[&](const int l_val) {
		return m_arv_ptr->setProjectorExposureTime(l_val);
	},val);
	
	/* 正しい値が返ってこないのでノーチェック。
	bool ret=false;
	int cval=-1;
	if( get_projector_exposure_time(&cval) &&  cval == val){
		ret=true;
	}
	return ret;
	*/
	return true;
}
#endif

bool CameraYCAM3D::get_projector_intensity(int *val){
	return get_camera_param_int("proj_intensity",[&](int *l_val) {
		*l_val =  m_arv_ptr->projectorIntensity();
		return *l_val >= 0;
	},val);
}

bool CameraYCAM3D::set_projector_intensity(const int val){
	const bool result = set_camera_param_int("proj_intensity",[&](const int l_val) {
		return m_arv_ptr->setProjectorIntensity(l_val);
	},val);
	
	/* 正しい値が返ってこないのでノーチェック。
	bool ret=false;
	int cval=-1;
	if( result && get_projector_intensity(&cval) &&  cval == val){
		ret=true;
	}
	return ret;
	*/
	return true;
}

bool CameraYCAM3D::get_temperature(int *val){
	return get_camera_param_int("led_temperature",[&](int *l_val) {
		*l_val =  m_arv_ptr->getTemperature();
		return *l_val >= 0;
	},val);
}

bool CameraYCAM3D::get_capture_param(camera::ycam3d::CaptureParameter *capt_param){
	bool ret=false;
	if( ! get_exposure_time_level( &capt_param->expsr_lv ) ){
		ROS_ERROR(LOG_HEADER"error:current camera exposure time level get failed.");
		
	}else if( ! get_gain_digital( &capt_param->gain ) ){
		ROS_ERROR(LOG_HEADER"error:current camera gain get failed.");
		
	}else if( ! get_projector_intensity( &capt_param->proj_intensity )){
		ROS_ERROR(LOG_HEADER"error:current projector intensity get failed.");
		
	}else{
		ret=true;
	}
	
	return ret;
}

bool CameraYCAM3D::update_capture_param(const camera::ycam3d::CaptureParameter &capt_param){
	
	ElapsedTimer tmr;
#ifdef DEBUG_DETAIL
	ROS_WARN(LOG_HEADER"update capture param start.");
#endif
	if( capt_param.expsr_lv >= 0 ){
		tmr.start_lap();
		int cur_expsr_lv = -1;
		if( ! get_exposure_time_level( &cur_expsr_lv) ){
			ROS_ERROR(LOG_HEADER"error:current exposure time level get failed.");
			return false;
		}else if( cur_expsr_lv == capt_param.expsr_lv ){
#ifdef DEBUG_DETAIL
			ROS_WARN(LOG_HEADER"exposure time level is same. skipped.");
#endif
		}else if( ! set_exposure_time_level(capt_param.expsr_lv) ){
			ROS_ERROR(LOG_HEADER"error:current exposure time level set failed. value=%d",capt_param.expsr_lv);
			return false;
		}else{
			ROS_INFO(LOG_HEADER"exposure time level updated. val=%d. proc_tm=%d",capt_param.expsr_lv,tmr.elapsed_lap_ms());
		}
	}
	if( capt_param.gain >= 0 ){
		tmr.start_lap();
		int cur_gain = -1;
		if( ! get_gain_digital( &cur_gain ) ){
			ROS_ERROR(LOG_HEADER"error:current camera gain get failed.");
			return false;
		}else if( cur_gain == capt_param.gain ){
#ifdef DEBUG_DETAIL
		ROS_WARN(LOG_HEADER"camera gain is same. skipped.");
#endif
		}else if( ! set_gain_digital( capt_param.gain ) ){
			ROS_ERROR(LOG_HEADER"error:current camera gain set failed. value=%d",capt_param.gain);
			return false;
		}else{
			ROS_INFO(LOG_HEADER"camera gain updated. val=%d. proc_tm=%d",capt_param.gain,tmr.elapsed_lap_ms());
		}
	}
	
	if( capt_param.proj_intensity >= 0 ){
		int cur_proj_intensity = -1;
		if( ! get_projector_intensity( &cur_proj_intensity ) ){
			ROS_ERROR(LOG_HEADER"error:current projector intensity get failed.");
			return false;
		}else if( cur_proj_intensity == capt_param.proj_intensity ){
#ifdef DEBUG_DETAIL
			ROS_WARN(LOG_HEADER"projector intensity is same. skipped.");
#endif
		}else if( ! set_projector_intensity(capt_param.proj_intensity) ){
			ROS_ERROR(LOG_HEADER"error:current projector intensity set failed. value=%d",capt_param.proj_intensity );
			return false;
		}else{
			ROS_INFO(LOG_HEADER"projector intensity updated. val=%d. proc_tm=%d",capt_param.proj_intensity,tmr.elapsed_lap_ms());
		}
	}
#ifdef DEBUG_DETAIL
	ROS_WARN(LOG_HEADER"update capture param finished. proc_tm=%d ms",tmr.elapsed_ms());
#endif
	return true;
}

void CameraYCAM3D::start_nw_delay_monitor_task(const int interval,const int timeout,camera::ycam3d::f_network_delayed callback,const bool ignUpdFail){
	stop_nw_delay_monitor_task();
	
	m_cancel_delay_mon=true;
	
	m_callback_nw_delayed = callback;

	m_delay_mon = std::thread([this,interval,timeout,ignUpdFail](){
		ROS_INFO(LOG_HEADER"network delay monitor task start.");
		std::thread hb_swap_thread;
		{
			std::lock_guard<std::timed_mutex> locker(m_camera_mutex);
			m_pre_heart_beat_val = m_arv_ptr->getHeartBeatTimeout();
			ROS_INFO(LOG_HEADER"network delay monitor: heart beat val=%d",m_pre_heart_beat_val);
		}
		
		
		bool ignore_callback=false;
		while( m_cancel_delay_mon ){
			
			const int next_val =  m_pre_heart_beat_val ^ (1 << 0);
			//ROS_INFO(LOG_HEADER"network delay monitor: heat beat write. cur=%d next=%d",m_pre_heart_beat_val,next_val);
			ElapsedTimer tmr;
			if( m_arv_ptr->isLost() ){
				//ROS_INFO(LOG_HEADER"network delay monitor. camera is lost.");
			}else{
				
				std::lock_guard<std::timed_mutex> locker(m_camera_mutex);
				//撮影や他の処理ですぐにロックが取得できないだろうからロック取得時間は考慮しない。
				tmr.restart();
				if( ! m_arv_ptr->setHeartBeatTimeout(next_val) ){
					//遅延ではなく、接続が切れた場合でもありえるので無視
					ROS_WARN(LOG_HEADER"network delay monitor: heart beat write failed.");
				}else{
					const int cur_val = m_arv_ptr->getHeartBeatTimeout();
					if( next_val != cur_val ){
						if( ignUpdFail ){
						}else{
							//callback!!!
							if( ! ignore_callback ){
								if(m_ros_err_pub){ m_ros_err_pub("network delay monitor: heart beat update failed.");}
							    ROS_ERROR(LOG_HEADER"network delay monitor: heart beat update failed. cur_val=%d, next_val=%d",cur_val,next_val);
								ignore_callback=true;
								m_callback_nw_delayed();
							}
						}
					}else{
						if( ignore_callback ){
							ROS_INFO(LOG_HEADER"network delay monitor: heart beat update success. continue to monitor.");
						}
						ignore_callback=false;
					}
				}
				
				const int elapsed = tmr.elapsed_ms();
				ROS_INFO(LOG_HEADER"network delay monitor: heart beat update finished. proc_tm=%d\n",elapsed);
				if( elapsed > timeout ){
					//callback!!!
					if( ! ignore_callback ){
						if(m_ros_err_pub){ m_ros_err_pub("network delay monitor: heart beat write delay occurred.");}
						ignore_callback=true;
						ROS_ERROR(LOG_HEADER"network delay monitor: heart beat write delay occurred. elapsed=%d, timeout=%d",elapsed,timeout);
						m_callback_nw_delayed();
					}
				}
			}
			std::this_thread::sleep_for(std::chrono::seconds(interval));
	
			m_pre_heart_beat_val = next_val;
		}
		if(hb_swap_thread.joinable()){
			ROS_INFO(LOG_HEADER"delay monitor task thread wait start.\n");
			hb_swap_thread.join();
			ROS_INFO(LOG_HEADER"delay monitor task thread wait end.\n");
		}
	});
}

void CameraYCAM3D::stop_nw_delay_monitor_task(){
	fprintf(stdout,"network delay monitor task stop: start.\n");
	if(m_delay_mon.joinable()){
		m_cancel_delay_mon=false;
		m_delay_mon.join();
	}
	fprintf(stdout,"network delay monitor task stop: end.\n");
}

void CameraYCAM3D::set_callback_ros_error_published(camera::ycam3d::f_ros_error_published callback){
	m_ros_err_pub = callback;
}
	
void CameraYCAM3D::set_callback_auto_con_limit_exceeded(camera::ycam3d::f_auto_con_limit_exceeded callback){
	m_callback_auto_lm_excd=callback;
}
//void CameraYCAM3D::ser_projector_pattern(int val){
//	m_arv_ptr->setProjectorPattern((YCAM_PROJ_PTN)val);
//}
/*
bool CameraYCAM3D::set_projector_enabled(const bool enabled){
	const bool result = set_camera_param_int("proj_enabled",[&](const int l_val) {
		return m_arv_ptr->setProjectorEnabled(l_val?true:false);
	},enabled?1:0);
	return true;
}*/
	
#if 0
bool CameraYCAM3D::get_projector_interval(int *val){
	return get_camera_param_int("proj_interval",[&](int *l_val) {
		*l_val =  m_arv_ptr->projectorFlashInterval();
		return *l_val >= 0;
	},val);
}

bool CameraYCAM3D::set_projector_interval(const int val){
	const bool result = set_camera_param_int("proj_interval",[&](const int l_val) {
		return m_arv_ptr->setProjectorFlashInterval(l_val);
	},val);
	
	/* 正しい値が返ってこないのでノーチェック。
	bool ret=false;
	int cval=-1;
	if( result && get_projector_interval(&cval) &&  cval == val){
		ret=true;
	}
	return ret;
	*/
	return true;
}
#endif

