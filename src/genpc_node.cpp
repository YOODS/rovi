#include <stdio.h>
#include <map>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include "rovi/Floats.h"
#include "rovi/GenPC.h"

#include "iPointCloudGenerator.hpp"
#include "YPCGeneratorUnix.hpp"
#include "YPCData.hpp"
#include "ElapsedTimer.hpp"

#define LOG_HEADER "(genpc) "


namespace {
//============================================= 無名名前空間 start =============================================

#define GET_CAMERA_LABEL(camno) (camno==0?'L':(camno==1?'R':'?'))
constexpr int CAMERA_NUM = 2;

std::unique_ptr<YPCGeneratorUnix> pcgen_ptr;
	
YPCData pcdata;
std::vector<sensor_msgs::PointCloud> pre_ptss(CAMERA_NUM);

int cur_cam_width = -1;
int cur_cam_height = -1;
	
/** 
 * PCGEN_SGBM = 0,  ///< SGBM
 * PCGEN_GRAYPS4,   ///< 位相シフト: Gray + 4step PS
 * PCGEN_MULTI,     ///< 位相シフト: マルチ
 **/
//PcGenMode cur_pc_gen_mode = PCGEN_GRAYPS4;
std::map<PcGenMode,std::string> PCGEN_MODE_MAP = {
	{PCGEN_SGBM,"SGBM"},
	{PCGEN_GRAYPS4,"Gray+4step_PS"},
	{PCGEN_MULTI,"Multi"} 
};

std::string file_dump("/tmp/");
bool isready = false;

ros::NodeHandle *nh = nullptr;
//[0]左カメラ用、[1]右カメラ用
ros::Publisher pub_ps_pointclouds[2];
ros::Publisher pub_ps_floats[2];
ros::Publisher pub_depth_imgs[2];
ros::Publisher pub_ps_alls[2];
ros::Publisher pub_pcounts[2];

ros::Publisher pub_rep;
	
const bool STEREO_CAM_IMGS_DEFAULT_SAVE = true;
const bool PC_DATA_DEFAULT_SAVE = true;
const bool QUANTIZE_POINTS_COUNT_DEFAULT_ENABLED = true;
const bool DEPTH_MAP_IMG_DEFAULT_ENABELED = true;

const bool VOXELIZED_PC_DATA_SAVE_DEFAULT_ENABELED = false;
const float VOXEL_LEAF_MIN_SIZE     = 0.001f;
const float VOXEL_LEAF_DEFAULT_SIZE = 1.0f;
const float VOXEL_LEAF_SIZE_INVALID = 0.0f;

const float RE_VOXEL_DEFAULT_INTERVAL = 0.1;
const float RE_VOXEL_MIN_INTERVAL     = 0.0001;
const bool  RE_VOXEL_DEFAULT_ENABLED  = false;
	
std::vector<double> vecQ;
std::vector<double> cam_K;
bool pre_quantize_points_count_enabled=false;


struct VoxelLeafSize {
	float x=VOXEL_LEAF_SIZE_INVALID;
	float y=VOXEL_LEAF_SIZE_INVALID;
	float z=VOXEL_LEAF_SIZE_INVALID;
	
	bool operator == (const VoxelLeafSize &obj){
		return this->x == obj.x && this->y == obj.y && this->z == obj.z;
	}
	
	bool is_disabled() const{
		return  x == VOXEL_LEAF_SIZE_INVALID && y == VOXEL_LEAF_SIZE_INVALID && z == VOXEL_LEAF_SIZE_INVALID; 
	}
};

VoxelLeafSize pre_vx_leaf_size;


bool cur_re_vx_enabled = false;
float cur_re_vx_interval = RE_VOXEL_DEFAULT_INTERVAL;
ros::Timer re_vx_monitor_timer;


struct CamCalibMat {
	int rows=0;
	int cols=0;
	std::vector<double> values;
	
	bool validate()const {
		if(rows < 0 || cols < 0 || values.empty()) return false;
		return (rows * cols) == values.size();
	}
	
	std::string to_string()const{
		std::ostringstream oss;
		
		oss << "rows=" << rows;
		oss << " cols=" << cols;
		
		oss << " values=";
		std::copy(values.begin(), values.end(),std::ostream_iterator<double>(oss, ", "));
		
		return oss.str();
	}
};

void set_ps_params(std::map<std::string,double> &params,const std::string &src_key,const std::string &dst_key,const double defaultVal){
	double val=defaultVal;
	nh->getParam(src_key,val);
	params[dst_key]=val;
}

template<typename T>
T get_param(const std::string &key,const T defaultVal){
	T val=defaultVal;
	if( ! nh->getParam(key,val) ){
		ROS_ERROR(LOG_HEADER"ros param get failed. key=%s",key.c_str());
	}
	return val;
}

struct XYZW{ float x,y,z,w;};

bool operator<(const XYZW& left, const XYZW& right){ return left.w < right.w;}

bool load_phase_shift_params()
{
	ROS_INFO(LOG_HEADER"phase shift parameter relod start.");
	ElapsedTimer tmr;
	std::map<std::string,double> params;
	{
		int prm_width=0;
		if(nh->getParam("pshift_genpc/calc/image_width",prm_width) && prm_width != cur_cam_width){
			ROS_ERROR(LOG_HEADER"param image width different. cur_cam_width=%d param_width=%d",cur_cam_width,prm_width);
			return false;
		}
		
		int prm_height=0;
		if(nh->getParam("pshift_genpc/calc/image_height",prm_height) && prm_height != cur_cam_height){
			ROS_ERROR(LOG_HEADER"param image height different. cur_cam_height=%d param_height=%d",cur_cam_height,prm_height);
			return false;
		}
	}
	
	set_ps_params(params,"pshift_genpc/calc/method3d", "method3d", 0);
	set_ps_params(params,"pshift_genpc/calc/camera_type", "camera_type", 2);
	set_ps_params(params,"pshift_genpc/calc/bw_diff", "bw_diff", 16);
	set_ps_params(params,"pshift_genpc/calc/brightness", "brightness", 256);
	set_ps_params(params,"pshift_genpc/calc/darkness", "darkness", 16);
	set_ps_params(params,"pshift_genpc/calc/phase_wd_min", "phase_wd_min", 8);
	set_ps_params(params,"pshift_genpc/calc/phase_wd_thr", "phase_wd_thr", 3);
	set_ps_params(params,"pshift_genpc/calc/gcode_variation", "gcode_variation", 2);
	set_ps_params(params,"pshift_genpc/calc/max_ph_diff", "max_ph_diff", 1.0);
	set_ps_params(params,"pshift_genpc/calc/max_parallax", "max_parallax", 300);
	set_ps_params(params,"pshift_genpc/calc/min_parallax", "min_parallax", -200);
	set_ps_params(params,"pshift_genpc/calc/ls_points", "ls_points", 3);
	set_ps_params(params,"pshift_genpc/calc/n_phaseshift", "n_phaseshift", 4);
	set_ps_params(params,"pshift_genpc/calc/n_periods", "n_periods", 3);
	set_ps_params(params,"pshift_genpc/calc/period0", "period0", 9);
	set_ps_params(params,"pshift_genpc/calc/period1", "period1", 10);
	set_ps_params(params,"pshift_genpc/calc/period2", "period2", 11);
	set_ps_params(params,"pshift_genpc/calc/interpolation", "interpolation", 0);
	
	params["image_width"]=cur_cam_width;
	params["image_height"]=cur_cam_height;
	
	for(std::map<std::string,double>::iterator it=params.begin();it!=params.end();++it){
		ROS_INFO(LOG_HEADER"<phsft> %s=%g",it->first.c_str(),it->second);
	}
	
	if ( ! pcgen_ptr->init(params) ) {
		ROS_ERROR(LOG_HEADER"phase shift parameter reload failed.");
		return false;
	}
	
	nh->getParam("genpc/Q", vecQ); 
	if (vecQ.size() != 16){
		ROS_ERROR(LOG_HEADER"Param Q NG");
		return false;
	}
	nh->getParam("left/remap/K", cam_K);
	if (cam_K.size() != 9){
		ROS_ERROR(LOG_HEADER"Param K NG");
		return false;
	}
	{
		int remap_width=0;
		if(nh->getParam("left/remap/width", remap_width) && remap_width != cur_cam_width){
			ROS_ERROR(LOG_HEADER"remap image width different. cur_cam_width=%d remap_width=%d",cur_cam_width,remap_width);
			return false;
		}
		
		int remap_height=0;
		if(nh->getParam("left/remap/height", remap_height) && remap_height != cur_cam_height){
			ROS_ERROR(LOG_HEADER"remap image height different. cur_cam_width=%d remap_height=%d",cur_cam_height,remap_height);
			return false;
		}
	}
	
	if(nh->hasParam("genpc/dump")) nh->getParam("genpc/dump",file_dump);
	else file_dump="";
	
	if (vecQ.size() != 16){
		ROS_ERROR(LOG_HEADER"Param Q NG");
		return false;
	}
	ROS_INFO(LOG_HEADER"phase shift parameter load finished. proc_tm=%d ms",tmr.elapsed_ms());
	return true;
}

bool load_camera_calib_data(){
	ROS_INFO(LOG_HEADER"camera calibration data load start.");
	ElapsedTimer tmr;
	
	bool ret=false;
	
	CamCalibMat Kl;
	CamCalibMat Dl;
	CamCalibMat Kr;
	CamCalibMat Dr;
	CamCalibMat R;
	CamCalibMat T;
	if( ! nh->getParam("left/genpc/K_Cols",Kl.cols) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=left, key=K_Cols");
		
	}else if( ! nh->getParam("left/genpc/K_Rows",Kl.rows) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=left, key=K_Rows");
		
	}else if( ! nh->getParam("left/genpc/K",Kl.values) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=left, key=K");
		
	}else if( ! Kl.validate() ){
		ROS_ERROR(LOG_HEADER"K matrix is wrong. cam=left %s", Kl.to_string().c_str());
		
	}else if( ! nh->getParam("left/genpc/D_Cols",Dl.cols) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=left, key=D_Cols");
		
	}else if( ! nh->getParam("left/genpc/D_Rows",Dl.rows) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=left, key=D_Rows");
		
	}else if( ! nh->getParam("left/genpc/D",Dl.values) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=left, key=D");
		
	}else if( ! Kl.validate() ){
		ROS_ERROR(LOG_HEADER"D matix is wrong. cam=left %s", Dl.to_string().c_str());
		
	}else if( ! nh->getParam("right/genpc/K_Cols",Kr.cols) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=K_Cols");
		
	}else if( ! nh->getParam("right/genpc/K_Rows",Kr.rows) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=K_Rows");
		
	}else if( ! nh->getParam("right/genpc/K",Kr.values) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=K");
		
	}else if( ! Kr.validate() ){
		ROS_ERROR(LOG_HEADER"K matrix is wrong. cam=right %s", Kr.to_string().c_str());
		
	}else if( ! nh->getParam("right/genpc/D_Cols",Dr.cols) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=D_Cols");
		
	}else if( ! nh->getParam("right/genpc/D_Rows",Dr.rows) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=D_Rows");
		
	}else if( ! nh->getParam("right/genpc/D",Dr.values) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=D");
		
	}else if( ! Dr.validate() ){
		ROS_ERROR(LOG_HEADER"D matix is wrong. cam=right %s", Dr.to_string().c_str());
		
	}else if( ! nh->getParam("right/genpc/R_Cols",R.cols) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=R_Cols");
		
	}else if( ! nh->getParam("right/genpc/R_Rows",R.rows) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=R_Rows");
		
	}else if( ! nh->getParam("right/genpc/R",R.values) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=R");
		
	}else if( ! R.validate() ){
		ROS_ERROR(LOG_HEADER"R matrix is wrong. cam=right %s", R.to_string().c_str());
		
	}else if( ! nh->getParam("right/genpc/T_Cols",T.cols) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=T_Cols");
		
	}else if( ! nh->getParam("right/genpc/T_Rows",T.rows) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=T_Rows");
		
	}else if( ! nh->getParam("right/genpc/T",T.values) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=T");
		
	}else if( ! T.validate() ){
		ROS_ERROR(LOG_HEADER"T matix is wrong. %s", T.to_string().c_str());
		
	}
	
	ROS_INFO(LOG_HEADER"(calib) <left>  K %s",Kl.to_string().c_str());
	ROS_INFO(LOG_HEADER"(calib) <left>  D %s",Dl.to_string().c_str());
	ROS_INFO(LOG_HEADER"(calib) <right> K %s",Kr.to_string().c_str());
	ROS_INFO(LOG_HEADER"(calib) <right> D %s",Dr.to_string().c_str());
	
	ROS_INFO(LOG_HEADER"(calib)         R %s",R.to_string().c_str());
	ROS_INFO(LOG_HEADER"(calib)         T %s",T.to_string().c_str());
	
	if( ! pcgen_ptr->create_camera_raw(Kl.values,Kr.values,Dl.values,Dr.values,R.values,T.values) ){
		ROS_ERROR(LOG_HEADER"stereo camera create failed.");
	}else{
		ret=true;
	}
	ROS_INFO(LOG_HEADER"camera calibration data load finished. proc_tm=%d ms",tmr.elapsed_ms() );
	
	return ret;
}

bool load_pattern_images(const rovi::GenPC::Request &req){
	
	const int ptnCaptNum=req.ptn_capt_num < 1 ? 1 : req.ptn_capt_num;
	const int ptnImageNum = req.imgL.size()/ptnCaptNum;
	
	ROS_INFO(LOG_HEADER"pattern capture num = %d, pattern image num = %d",ptnCaptNum,ptnImageNum);
	
	ElapsedTimer tmr;
	bool ret=true;
	const bool ptn_image_save_flg = get_param<bool>("genpc/point_cloud/img_save",STEREO_CAM_IMGS_DEFAULT_SAVE);
	
	FILE *f_captseq =nullptr;
	
	for( int n = 0 ; n < ptnCaptNum ; n++ ){
		std::vector<cv::Mat> ptn_imgs;
		std::vector<unsigned char*> ptn_img_pointers;
		
		tmr.start_lap();
		ROS_INFO(LOG_HEADER"<%d> pattern image convert start.",n);
		
		bool all_img_cnv_flg=true;
		for (int i = 0; i < ptnImageNum ; i++ ){
			const int idx = n * ptnImageNum + i;
			//ROS_WARN(LOG_HEADER"n=%d i=%d idx=%d", n, i, idx);
			
			try {
				cv::Mat img = cv_bridge::toCvCopy(req.imgL[idx], sensor_msgs::image_encodings::MONO8)->image;
				//ROS_INFO(LOG_HEADER"[%d] #%2d left  size=%dx%d",n,i,img.cols,img.rows);
				ptn_imgs.push_back(img);
				ptn_img_pointers.push_back(ptn_imgs.back().data);
				
				img = cv_bridge::toCvCopy(req.imgR[idx], sensor_msgs::image_encodings::MONO8)->image;
				//ROS_INFO(LOG_HEADER"[%d] #%2d right size=%dx%d",n,i,img.cols,img.rows);

				ptn_imgs.push_back(img);
				ptn_img_pointers.push_back(ptn_imgs.back().data);
			}catch( cv_bridge::Exception& e ){
				ROS_ERROR(LOG_HEADER"<%d> error:pattern image convert failed. ptn_img_idx=%d, cv_bridge:exception: %s",
					n,i, e.what());
				all_img_cnv_flg=false;
				break;
			}
		}
		
		if( ! all_img_cnv_flg ){
			break;
		}else{
			ROS_INFO(LOG_HEADER"<%d> pattern image convert finished. proc_tm=%d ms",n,tmr.elapsed_lap_ms());
		}
		
		//撮影画像保存
		if( ! file_dump.empty() ) {
			if( ptn_image_save_flg ){
				tmr.start_lap();
				ROS_INFO(LOG_HEADER"<%d> pattern images save start. save_path=%s",n,file_dump.c_str());
				int img_idx=0;
			
				for (int i = 0 ; i < ptnImageNum ; i++) {
					const int idx = i * 2;
					//ROS_WARN(LOG_HEADER"n=%d i=%d idx=%d", n, i, idx);
					if( ptnCaptNum < 2 ){
						cv::imwrite(cv::format((file_dump + "/capt%02d_0.pgm").c_str(), i), ptn_imgs.at(img_idx) );
					}else{
						cv::imwrite(cv::format((file_dump + "/hdr_%d_capt%02d_0.pgm").c_str(),n, i), ptn_imgs.at(img_idx) );
					}
					img_idx++;
					
					if( ptnCaptNum < 2 ){
						cv::imwrite(cv::format((file_dump + "/capt%02d_1.pgm").c_str(), i), ptn_imgs.at(img_idx) );
					}else{
						cv::imwrite(cv::format((file_dump + "/hdr_%d_capt%02d_1.pgm").c_str(),n,i), ptn_imgs.at(img_idx) );
					}
					img_idx++;
				}
				
				ROS_INFO(LOG_HEADER"<%d> pattern images save finished. proc_tm=%d ms",n,tmr.elapsed_lap_ms());
			}
			
			if( ! f_captseq ){
				f_captseq = fopen((file_dump+"/captseq.log").c_str(), "w");
			}
			//for(int j=0; j < ptnImageNum ; j++) {
			//	fprintf(f,"(%d) %d %d\n", j, req.imgL[j].header.seq, req.imgR[j].header.seq);
			//}
			for (int i = 0; i < ptnImageNum ; i++ ){
				const int idx = n * ptnImageNum + i;
				if( ptnCaptNum < 2 ){
					fprintf(f_captseq,"(%d) %d %d\n", i, req.imgL[idx].header.seq, req.imgR[idx].header.seq);
				}else{
					fprintf(f_captseq,"[%d] (%d) %d %d\n", n, i, req.imgL[idx].header.seq, req.imgR[idx].header.seq);
				}
			}
		}
		
		
		tmr.start_lap();
		ROS_INFO(LOG_HEADER"<%d> point cloud generator pattern image load start.",n);
		if( ! pcgen_ptr->set_images(ptn_img_pointers) ){
			ROS_ERROR(LOG_HEADER"<%d> error: point cloud generator pattern image load failed.",n);
			ret=false;
			break;
		}else{
			ROS_INFO(LOG_HEADER"<%d> point cloud generator pattern image load finished. proc_tm=%d ms",n,tmr.elapsed_lap_ms());
		}
	}
	if( f_captseq ){
		fclose(f_captseq);
	}
	
	if( ! ret ){
		ROS_ERROR(LOG_HEADER"Point cloud generator failed to load pattern image. elapsed=%d ms",tmr.elapsed_ms());
	}else{
		ROS_INFO(LOG_HEADER"point cloud generator has loaded all pattern images. elapsed=%d ms",tmr.elapsed_ms());
	}
	
	return ret;
}

VoxelLeafSize get_voxel_leaf_size(){
	VoxelLeafSize leaf_size;
	leaf_size.x = get_param<float>("genpc/voxelize/leaf_size/x",VOXEL_LEAF_DEFAULT_SIZE);
	leaf_size.y = get_param<float>("genpc/voxelize/leaf_size/y",VOXEL_LEAF_DEFAULT_SIZE);
	leaf_size.z = get_param<float>("genpc/voxelize/leaf_size/z",VOXEL_LEAF_DEFAULT_SIZE);
	
	if( leaf_size.is_disabled() ){
		//voxelization disabled
	}else{
		const float vx_leaf_v = get_param<float>("genpc/voxelize/leaf_size/v",0);
		
		leaf_size.x = std::max( vx_leaf_v , std::max( leaf_size.x , VOXEL_LEAF_MIN_SIZE ) );
		leaf_size.y = std::max( vx_leaf_v , std::max( leaf_size.y , VOXEL_LEAF_MIN_SIZE ) );
		leaf_size.z = std::max( vx_leaf_v , std::max( leaf_size.z , VOXEL_LEAF_MIN_SIZE ) );
	}
	
	return leaf_size;
}

bool count_quantize_points(const sensor_msgs::PointCloud &pts,rovi::Floats &output){
	const int N = pts.points.size();
	if( N <= 0 ){
		return false;
	}
	
	double X0=0,Y0=0,Z0=0;
	
	for (int n = 0 ; n < N ; n++ ) {
		X0 += pts.points[n].x;
		Y0 += pts.points[n].y;
		Z0 += pts.points[n].z;
	}
	X0/=N;
	Y0/=N;
	Z0/=N;
	
	// getting norm from the center and sort by it
	std::vector<XYZW> norm;
	norm.resize(N);
	for (int n = 0; n < N; n++) {
		float dx = (norm[n].x = pts.points[n].x) - X0;
		float dy = (norm[n].y = pts.points[n].y) - Y0;
		float dz = (norm[n].z = pts.points[n].z) - Z0;
		//  norm[n].w=sqrt(dx*dx+dy*dy+dz*dz);
		norm[n].w = sqrt(dx*dx + dy*dy);
	}
	std::sort(norm.begin(), norm.end());
	
	// Quantize points count for Numpy array
	const double gamma=1.1;
	const double kn=floor((log10(N)-1)/log10(gamma));
	const int Qn=N<10? N:floor(10*pow(gamma,kn));
	output.data.resize(3*Qn);
	for (int n = 0; n < Qn; n++) {
		int n3=3*n;
		output.data[n3++] = norm[n].x;
		output.data[n3++] = norm[n].y;
		output.data[n3  ] = norm[n].z;
	}
	
	return true;
}

bool voxelizing_pcdata (const sensor_msgs::PointCloud &src_pcdata,const VoxelLeafSize &vxLeafSize, sensor_msgs::PointCloud &dst_pcdata){
	if( src_pcdata.points.size() < 10 ){
		ROS_ERROR("voxelization failure. too few points.");
		return false;
	}
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcdata_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcdata_pcl->width = src_pcdata.points.size();
	pcdata_pcl->height = 1;
	pcdata_pcl->points.resize(src_pcdata.points.size());
	
	for (int i = 0 ; i < src_pcdata.points.size(); i++) {
		pcl::PointXYZRGB * pos = pcdata_pcl->points.data() + i;
		pos->x = src_pcdata.points[i].x;
		pos->y = src_pcdata.points[i].y;
		pos->z = src_pcdata.points[i].z;
		pos->r = pos->g = pos->b =  src_pcdata.channels[0].values[i] * 255.0;
	}

	pcl::PointCloud<pcl::PointXYZRGB> vx_points;
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud (pcdata_pcl);
	sor.setLeafSize (vxLeafSize.x, vxLeafSize.y, vxLeafSize.z);
	sor.filter (vx_points);
	
	const Eigen::Vector3i minBoxCoord= sor.getMinBoxCoordinates();
	const Eigen::Vector3i maxBoxCoord= sor.getMaxBoxCoordinates();

	bool ret = false;
	if( minBoxCoord[0] == 0 && minBoxCoord[1] == 0 && minBoxCoord[2] == 0 &&
		maxBoxCoord[0] == 0 && maxBoxCoord[1] == 0 && maxBoxCoord[2] == 0){
		ROS_ERROR("voxelization failure. out of memory? leaf_size=(%g, %g, %g)",vxLeafSize.x,vxLeafSize.y,vxLeafSize.z);
		
	}else{
		ret = true;
		sensor_msgs::PointCloud2 pts2_vx;
		pcl::toROSMsg(vx_points, pts2_vx);
		sensor_msgs::convertPointCloud2ToPointCloud(pts2_vx,dst_pcdata);
	}
	return ret;
}
	
bool exec_downsampling (const sensor_msgs::PointCloud &pts,const VoxelLeafSize &vx_leaf_size,const bool quantize_count_exec,rovi::Floats &ds_points,sensor_msgs::PointCloud *out_pts_vx=nullptr){
	
	const int N = pts.points.size();
	
	sensor_msgs::PointCloud pts_ds = pts;
	//voxelization
	if( vx_leaf_size.is_disabled() ){
		ROS_INFO(LOG_HEADER"voxelization disabled.");
		
	}else if(pts_ds.points.empty()){
		ROS_INFO(LOG_HEADER"voxelization skipped. data is empty.");
		
	}else{
		ElapsedTimer tmr_voxel;
		
		ROS_INFO(LOG_HEADER"voxelization start.");
		
		sensor_msgs::PointCloud pts_vx;
		if( ! voxelizing_pcdata( pts_ds, vx_leaf_size, pts_vx) ){
			ROS_ERROR(LOG_HEADER"voxelization failed.");
		}else{
			pts_ds = pts_vx;
			if(out_pts_vx){
				*out_pts_vx=pts_vx;
			}
		}
		
		ROS_INFO(LOG_HEADER"voxelization finished. leaf_size=(%g, %g, %g) count=%d / %d (%.2f%%), proc_tm=%d ms",
			vx_leaf_size.x, vx_leaf_size.y, vx_leaf_size.z,
			(int)pts_vx.points.size(), N, N==0?0:pts_vx.points.size()/(float)N *100,
			tmr_voxel.elapsed_ms());
	}

	//Quantize points count for Numpy array
	if( ! quantize_count_exec ){
		ROS_INFO(LOG_HEADER"quantize points count disabled.");
		
	}else if(pts_ds.points.empty()){
		ROS_INFO(LOG_HEADER"quantize points count skipped. data is empty.");
		
	}else{
		
		ROS_INFO(LOG_HEADER"quantize points count start.");
		ElapsedTimer tmr_norm_calc;
		if( ! count_quantize_points( pts_ds, ds_points ) ){
			ROS_ERROR(LOG_HEADER"quantize points count failed.");
		}
		const int ds_point_count = ds_points.data.size()/3;
		ROS_INFO(LOG_HEADER"quantize points count finished. count=%d / %d (%.2f%%), proc_tm=%d ms",
			ds_point_count, (int)pts_ds.points.size(),ds_point_count /(float)pts_ds.points.size() *100,tmr_norm_calc.elapsed_ms());
	}
	
	if( ds_points.data.empty() ){
		const int pts_ds_count = pts_ds.points.size();
		ds_points.data.resize(pts_ds_count * 3);
		
		for (int i = 0,n=0 ; i < pts_ds_count ; ++i,++n) {
			ds_points.data[  n] = pts_ds.points[i].x;
			ds_points.data[++n] = pts_ds.points[i].y;
			ds_points.data[++n] = pts_ds.points[i].z;
		}
	}
	
	return true;
}

void re_voxelization_monitor(const ros::TimerEvent& e)
{
	//leaf_sizeを個別に変えるとそのたびにpublishされるので一気に指定する方法
	//$rosparam set genpc/voxelize/leaf_size '{"x":3,"y":3,"z":3}'
	
	if( ! get_param<bool>("genpc/voxelize/recalc/enabled",RE_VOXEL_DEFAULT_ENABLED) ){
		//pre_vx_leaf_size = VoxelLeafSize();
		ROS_INFO(LOG_HEADER"re-voxelization is disabled.");
		
	}else{
		VoxelLeafSize vx_leaf_size = get_voxel_leaf_size();
		
		//再ボクセル化の際はボクセル化のサイズが異なる時だけ計算する。
		if( pre_vx_leaf_size == vx_leaf_size ){
			//ROS_INFO(LOG_HEADER"re-voxelization update is nothing.");
		}else{
			ROS_INFO(LOG_HEADER"re-voxelization start.");
			ElapsedTimer tmr_voxel;
			
			sensor_msgs::PointCloud *pre_pc=nullptr;
			for(int camno = 0 ; camno < CAMERA_NUM ; ++camno ){
				pre_pc = &pre_ptss[camno];
				
				ROS_INFO("camno %d",camno);
				
				if( pre_pc->points.empty() ){
					//skip
					continue;
				}
				
				sensor_msgs::PointCloud pts_vx;
				rovi::Floats pc_points;
				if( ! exec_downsampling( *pre_pc, vx_leaf_size, pre_quantize_points_count_enabled, pc_points ) ){
					ROS_ERROR(LOG_HEADER"[%c] downsampling failed.",GET_CAMERA_LABEL(camno));
					continue;
				}
				ROS_INFO("camno %d after",camno);
				const int N = pre_pc->points.size();
				const int ds_point_count=pc_points.data.size()/3;
				ROS_INFO(LOG_HEADER"[%c] re-voxelization finished. leaf_size=(%g, %g, %g), point_num=%d / %d (%.2f%%), proc_tm=%d ms",
					GET_CAMERA_LABEL(camno),
					vx_leaf_size.x, vx_leaf_size.y, vx_leaf_size.z,
					ds_point_count , N, N == 0 ? 0 : ds_point_count / (float)N *100,
					tmr_voxel.elapsed_ms());
				pub_ps_floats[camno].publish(pc_points);
			}
			pre_vx_leaf_size = vx_leaf_size;
		}
	}
	
	const float vx_re_interval= std::max(get_param<float>(
			"genpc/voxelize/recalc/interval",RE_VOXEL_DEFAULT_INTERVAL),RE_VOXEL_MIN_INTERVAL);
	if( vx_re_interval != cur_re_vx_interval ){
		re_vx_monitor_timer.setPeriod(ros::Duration(vx_re_interval));
		ROS_INFO(LOG_HEADER"re-voxelization interval has changed. %g sec -> %g sec",cur_re_vx_interval,vx_re_interval);
		cur_re_vx_interval = vx_re_interval;
	}
}

bool genpc(rovi::GenPC::Request &req, rovi::GenPC::Response &res)
{
    double t02 = ros::Time::now().toSec();
	
	ElapsedTimer tmr_proc;
	ROS_INFO(LOG_HEADER"start: ptn_capt_num=%d, ptn capt left image num=%d, ptncapt left image num=%d",req.ptn_capt_num, (int)req.imgL.size(), (int)req.imgR.size());
	
	re_vx_monitor_timer.stop();
	
	bool result=false;
	res.pc_cnt = 0;
	res.pc_cnt_r = -1;
	
	const bool calc_right_camera_flg =  get_param<bool>("pshift_genpc/calc/calc_right_camera",false);
	if( calc_right_camera_flg ){
		ROS_INFO(LOG_HEADER"calculate right camera");
	}
	
	if( ! pcgen_ptr ){
		pcgen_ptr.reset(new YPCGeneratorUnix());
		//途中で変えないこと
		const PcGenMode pc_gen_mode = (PcGenMode)get_param<int>("pshift_genpc/calc/pcgen_mode",(int)PCGEN_GRAYPS4);
		
		if( ! pcgen_ptr->create_pcgen(pc_gen_mode) ){
			ROS_ERROR(LOG_HEADER"point cloud generator create failed.");
			return false;
		}else{
			ROS_INFO(LOG_HEADER"Point Cloud Generator Created. mode=%d (%s)",pc_gen_mode,PCGEN_MODE_MAP[pc_gen_mode].c_str());
		}
	}
	
	std::vector<YPCData> yds_pcs(CAMERA_NUM);
	pre_ptss.assign(CAMERA_NUM,{});
	std::vector<sensor_msgs::PointCloud> cur_pts(CAMERA_NUM);

	
	pre_vx_leaf_size = VoxelLeafSize();
	
	const int width  = req.imgL[0].width;
	const int height = req.imgL[0].height;
	
	if( cur_cam_width < 0 || cur_cam_height < 0 ){
		cur_cam_width = width;
		cur_cam_height = height;
		
	}else if( cur_cam_width != width ){
		ROS_ERROR(LOG_HEADER"camera width has changed. old_width=%d cur_width=%d",cur_cam_width, width);
		return false;
		
	}else if( cur_cam_height != height ){
		ROS_ERROR(LOG_HEADER"camera height has changed. old_height=%d cur_height=%d",cur_cam_height,height);
		return false;
	}
	
	if( ! load_phase_shift_params() ){
		ROS_ERROR(LOG_HEADER"phase shift parameter load failed.");
		
	}else if ( ! isready ) {
		ROS_INFO(LOG_HEADER"current camera resolution. w=%d h=%d", cur_cam_width, cur_cam_height);
		if( ! load_camera_calib_data() ){
			ROS_ERROR(LOG_HEADER"camera calibration data load failed.");
			
		}else{
			ROS_INFO(LOG_HEADER"camera calibration data loaded.");
			isready=true;
			
		}
	}
	//std::vector<cv::Mat> stereo_imgs;
	//std::vector<unsigned char*> stereo_img_pointers;
	std::vector<sensor_msgs::ImagePtr> depth_imgs;
	depth_imgs.assign(CAMERA_NUM,sensor_msgs::ImagePtr(new sensor_msgs::Image));
	std::vector<sensor_msgs::PointCloud> pts_vxs(CAMERA_NUM);
	std::vector<cv::Mat> depthimg_mats(CAMERA_NUM);
	
	pcgen_ptr->reset();
	
	if( ! isready ){
		ROS_ERROR(LOG_HEADER"camera calibration data load failed. elapsed=%d ms", tmr_proc.elapsed_ms());
	
	}else if( ! load_pattern_images(req) ){
		ROS_ERROR(LOG_HEADER"point cloud generator pattern image load failed.");
		
	}else if ( ! pcgen_ptr->preprocess() ) {
		ROS_ERROR(LOG_HEADER"point cloud data generator preprocess failed.");
		
	}else{
		ROS_INFO(LOG_HEADER"point cloud generation start.");
		
		const bool depthmap_enabled = get_param<bool>("genpc/depthmap_img/enabled",DEPTH_MAP_IMG_DEFAULT_ENABELED);
		const bool quantize_count_enabled = get_param<bool>("genpc/quantize_points_count/enabled",QUANTIZE_POINTS_COUNT_DEFAULT_ENABLED);
		
		result=true;
		
		for( int camno=0; camno < CAMERA_NUM; ++camno ){
			if( camno == 1 && ! calc_right_camera_flg ){
				continue;
			}
			
			ElapsedTimer tmr_genpc;
			sensor_msgs::PointCloud pts;
			pts.header.stamp = ros::Time::now();
			pts.header.frame_id = "/camera";
			rovi::Floats ds_points;

			rovi::Floats pc_points;
			
			YPCData *ypcData=yds_pcs.data()+camno;
			
			if(! pcgen_ptr->execute(camno) ){
				ROS_ERROR(LOG_HEADER"[%c] point cloud generate failed.",GET_CAMERA_LABEL(camno));
				result=false;
				
			}else{
				
				const int N = pcgen_ptr->save_pointcloud(ypcData);
				
				ROS_INFO(LOG_HEADER"[%c] point cloud generation finished. point_num=%d, diparity_tm=%d ms, genpc_tm=%d ms, total_tm=%d ms, elapsed=%d ms",
					GET_CAMERA_LABEL(camno),N, ElapsedTimer::duration_ms(pcgen_ptr->get_elapsed_disparity()), ElapsedTimer::duration_ms(pcgen_ptr->get_elapsed_genpcloud()),
					tmr_genpc.elapsed_ms(), tmr_proc.elapsed_ms());
				
				if( N == 0  ){
					ROS_WARN(LOG_HEADER"[%c] genpc point count 0. elapsed=%d ms",GET_CAMERA_LABEL(camno), tmr_proc.elapsed_ms());
				}
				
				//点群データ変換
				ROS_INFO(LOG_HEADER"[%c] point cloud data convert start.",GET_CAMERA_LABEL(camno));
				ElapsedTimer tmr_pcgen_conv;
				if( ! ypcData->make_point_cloud(pts) ){
					ROS_ERROR(LOG_HEADER"[%c] point cloud data convert failed.",GET_CAMERA_LABEL(camno));
				}else{
					
					cur_pts[camno]=pts;
				}
				
				ROS_INFO(LOG_HEADER"[%c] point cloud data convert finished. proc_tm=%d ms, elapsed=%d ms",
					GET_CAMERA_LABEL(camno),tmr_pcgen_conv.elapsed_ms(), tmr_proc.elapsed_ms());
				
				//downsampling
				{
					VoxelLeafSize vx_leaf_size = get_voxel_leaf_size();
					
					pre_quantize_points_count_enabled = quantize_count_enabled;
					
					ElapsedTimer tmr_downsampling;
					ROS_INFO(LOG_HEADER"[%c] downsampling start.",GET_CAMERA_LABEL(camno));
					sensor_msgs::PointCloud * pts_vx=&pts_vxs[camno];
					if( ! exec_downsampling( pts, vx_leaf_size, quantize_count_enabled, ds_points,pts_vx) ){
						ROS_ERROR(LOG_HEADER"[%c] downsampling failed.",GET_CAMERA_LABEL(camno));
					}else{
						//ROS_INFO(LOG_HEADER"point after downsampling. count=%d (%d)",(int)ds_points.data.size()/3,(int)ds_points.data.size());
					}
					
					pre_vx_leaf_size = vx_leaf_size;
					
					const int ds_point_count=ds_points.data.size()/3;
					ROS_INFO(LOG_HEADER"[%c] downsampling finished. count=%d / %d (%.2f%%), proc_tm=%d ms, elapsed=%d ms",
						GET_CAMERA_LABEL(camno),ds_point_count , N, N == 0 ? 0 : ds_point_count / (float)N *100,
						tmr_downsampling.elapsed_ms(), tmr_proc.elapsed_ms());
				}
				
				//depthmap making
				if( ! depthmap_enabled ){
					ROS_INFO(LOG_HEADER"[%c] depthmap image make skipped.",GET_CAMERA_LABEL(camno));
				}else{
					ROS_INFO(LOG_HEADER"[%c] depthmap image make start.",GET_CAMERA_LABEL(camno));
					ElapsedTimer tmr_depthmap;
					
					cv::Mat *depthimg_mat=&depthimg_mats[camno];
					if( ! ypcData->make_depth_image(*depthimg_mat) ){
						ROS_ERROR(LOG_HEADER"[%c] depthmap image make failed.",GET_CAMERA_LABEL(camno));
					}else{
						try{
							depth_imgs[camno] = cv_bridge::CvImage(std_msgs::Header(),"mono16",*depthimg_mat).toImageMsg();
						}catch (cv_bridge::Exception& e)	{
							ROS_ERROR(LOG_HEADER"[%c] depthmap image cv_bridge:exception: %s",GET_CAMERA_LABEL(camno), e.what());
						}
					}
					ROS_INFO(LOG_HEADER"[%c] depthmap image make finished. proc_tm=%d ms",GET_CAMERA_LABEL(camno), tmr_depthmap.elapsed_ms());
				}
				if( camno ==0 ){
					res.pc_cnt = N;
				}else if( camno == 1 ){
					res.pc_cnt_r = N;
				}
				
				pc_points = ypcData->to_rg_floats();
				
				std_msgs::Int32 pcnt;
				pcnt.data=N;
				
				pub_pcounts[camno].publish(pcnt);
				pub_ps_pointclouds[camno].publish(pts);
				pub_ps_floats[camno].publish(ds_points);
				pub_depth_imgs[camno].publish(depth_imgs[camno]);
				pub_ps_alls[camno].publish(pc_points);
			}
		}
		
		pre_ptss = cur_pts;
	}
	

	{
	    const double t03 = ros::Time::now().toSec();
	    char s[64];
	    sprintf(s,"{'T02':%lf, 'T03':%lf}", t02, t03);
		std_msgs::String tnow;
	    tnow.data=s;
	    pub_rep.publish(tnow);
	}
	
	ROS_INFO(LOG_HEADER "publish finished. elapsed=%d ms", tmr_proc.elapsed_ms());
	
	//データ保存
	if( ! file_dump.empty() ) {
		const bool data_save_flg = get_param<bool>("genpc/point_cloud/data_save",PC_DATA_DEFAULT_SAVE);
		const bool voxel_save_flg = get_param<bool>("genpc/voxelize/data_save",VOXELIZED_PC_DATA_SAVE_DEFAULT_ENABELED);
		const bool depthmap_save_flg = get_param<bool>("genpc/depthmap_img/img_save",DEPTH_MAP_IMG_DEFAULT_ENABELED);
		
		for( int camno=0; camno < CAMERA_NUM; ++camno ){
			if( camno == 1 && ! calc_right_camera_flg ){
				continue;
			}
			
			const YPCData *ypcData=yds_pcs.data()+camno;
			
			if( !  data_save_flg ){
				ROS_INFO(LOG_HEADER"[%c] ply file save skipped.",GET_CAMERA_LABEL(camno));
			}else{
				ElapsedTimer tmr_save_pcdata;
				std::string save_file_path;
				if( camno == 0 ){
					save_file_path = file_dump + "/test.ply";
				}else if( camno == 1 ){
					save_file_path = file_dump + "/test_r.ply";
				}else{
					ROS_ERROR(LOG_HEADER"[%c] ply data save failed. unknown camera no. ",GET_CAMERA_LABEL(camno));
					break;
				}
				
				/*
				PLYSaver plysave(save_file_path);
				if (pcgen_ptr->save_pointcloud(&plysave) == 0) {
					fprintf(stderr, "no points.\n");
					return EXIT_FAILURE;
				}*/
				
				if( ! ypcData->save_ply(save_file_path) ){
					ROS_ERROR(LOG_HEADER"[%c] ply file save failed. proc_tm=%d ms, path=%s",
						GET_CAMERA_LABEL(camno),tmr_save_pcdata.elapsed_ms(), save_file_path.c_str());
				}else{
					ROS_INFO(LOG_HEADER"[%c] ply file save succeeded. proc_tm=%d ms, path=%s",
						GET_CAMERA_LABEL(camno),tmr_save_pcdata.elapsed_ms(), save_file_path.c_str());
				}
			}
			
			//todo:************* pending *************
			//writePLY(file_dump + "/testRG.ply", pcdP, N, pcgenerator->get_rangegrid(), width, height);
			//ROS_INFO("after  outPLY");
			
			sensor_msgs::PointCloud *pts_vx = &pts_vxs[camno];
			
			if( pts_vx->points.empty() || ! voxel_save_flg ){
				ROS_INFO(LOG_HEADER"voxelized point cloud data save skipped.");
			}else{
				ElapsedTimer tmr_save_voxel;
				
				std::string save_file_path;
				if( camno == 0 ){
					save_file_path = file_dump + "/voxel.ply";
				}else if( camno == 1 ){
					save_file_path = file_dump + "/voxel_r.ply";
				}else{
					ROS_ERROR(LOG_HEADER"[%c] voxel data save failed. unknown camera no. ",GET_CAMERA_LABEL(camno));
					break;
				}
				
				sensor_msgs::PointCloud2 pts2_vx;
				sensor_msgs::convertPointCloudToPointCloud2(*pts_vx,pts2_vx);
				pcl::PointCloud<pcl::PointXYZRGB> pts_vx_pcl;
				pcl::fromROSMsg(pts2_vx, pts_vx_pcl);
				
				pcl::PLYWriter ply_writer;
				int save_ret = 0;
				if( save_ret =  ply_writer.write<pcl::PointXYZRGB> (save_file_path, pts_vx_pcl, true) ){
					ROS_ERROR(LOG_HEADER"voxelized point cloud data save failed. ret=%d, proc_tm=%d ms, path=%s",
						save_ret, tmr_save_voxel.elapsed_ms(), save_file_path.c_str());
				}else{
					ROS_INFO(LOG_HEADER"voxelized point cloud data save succeeded. proc_tm=%d ms, path=%s",
						tmr_save_voxel.elapsed_ms(), save_file_path.c_str());
				}
				
				//sample:将来的にはPointCloud2へ
				/*
				pcl::PointCloud<pcl::PointXYZRGB> pts_vx_pcl2;
				pcl::fromROSMsg(*pcdata.get_data(),pts_vx_pcl2);
				ply_writer.write<pcl::PointXYZRGB> ("/tmp/zzzz.ply",pts_vx_pcl2 , true);
				*/
			}
			
			//depthmap image save
			cv::Mat * depthimg_mat = &depthimg_mats[camno];
			if( depthimg_mat->empty() || ! depthmap_save_flg ){
				ROS_INFO(LOG_HEADER"depthmap image save skipped.");
			}else{
				ElapsedTimer tmr_save_depthmap;
				
				std::string save_file_path;
				if( camno == 0 ){
					save_file_path = file_dump + "/depth.png";
				}else if( camno == 1 ){
					save_file_path = file_dump + "/depth_r.png";
				}else{
					ROS_ERROR(LOG_HEADER"[%c] depthmap image save failed. unknown camera no. ",GET_CAMERA_LABEL(camno));
					break;
				}
				
				if( ! cv::imwrite(save_file_path,*depthimg_mat) ){
					ROS_ERROR(LOG_HEADER"depthmap image save failed. proc_tm=%d ms, path=%s", tmr_save_depthmap.elapsed_ms(), save_file_path.c_str());
				}else {
					ROS_INFO(LOG_HEADER"depthmap image make succeeded. proc_tm=%d ms, path=%s", tmr_save_depthmap.elapsed_ms(), save_file_path.c_str());
				}
			}
		}
	}
	
	//再ボクセル化監視
	{
		cur_re_vx_interval = std::max(get_param<float>("genpc/voxelize/recalc/interval",RE_VOXEL_DEFAULT_INTERVAL),RE_VOXEL_MIN_INTERVAL);
		
		ROS_INFO(LOG_HEADER"re-voxelization monitor start. interval=%g sec",cur_re_vx_interval);
		re_vx_monitor_timer.setPeriod(ros::Duration(cur_re_vx_interval));
		re_vx_monitor_timer.start();
	}
	ROS_INFO(LOG_HEADER "node end. elapsed=%d ms", tmr_proc.elapsed_ms());
	return result;
}


//============================================= 無名名前空間  end  =============================================
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "genpc_node");
	ros::NodeHandle n;
	nh = &n;
	
	ros::ServiceServer svc1 = n.advertiseService("genpc", genpc);
	
	pub_ps_pointclouds[0]   = n.advertise<sensor_msgs::PointCloud>("ps_pc", 1);
	pub_ps_floats[0]        = n.advertise<rovi::Floats>("ps_floats", 1);
	pub_depth_imgs[0]       = n.advertise<sensor_msgs::Image>("image_depth", 1);
	pub_ps_alls[0]          = n.advertise<rovi::Floats>("ps_all", 1);
	pub_pcounts[0]          = n.advertise<std_msgs::Int32>("pcount", 1);
		
	pub_ps_pointclouds[1]   = n.advertise<sensor_msgs::PointCloud>("ps_pc_r", 1);
	pub_ps_floats[1]        = n.advertise<rovi::Floats>("ps_floats_r", 1);
	pub_depth_imgs[1]       = n.advertise<sensor_msgs::Image>("image_depth_r", 1);
	pub_ps_alls[1]          = n.advertise<rovi::Floats>("ps_all_r", 1);
	pub_pcounts[1]          = n.advertise<std_msgs::Int32>("pcount_r", 1);
	
	pub_rep              = n.advertise<std_msgs::String>("/report", 1);
	
	re_vx_monitor_timer = nh->createTimer(ros::Duration(RE_VOXEL_DEFAULT_INTERVAL), re_voxelization_monitor);
	re_vx_monitor_timer.stop();
	
	ros::spin();
	
	return 0;
}
