#include <stdio.h>
#include <chrono>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include "rovi/Floats.h"
#include "rovi/GenPC.h"

#include "iPointCloudGenerator.hpp"
#include "YPCGeneratorUnix.hpp"
#include "YPCData.hpp"

#define LOG_HEADER "genpc: "

#define DURATION_TO_MS(duration) (int)std::chrono::duration_cast<std::chrono::milliseconds>(duration).count()
#define ELAPSED_TM(start_tm)   (int)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_tm).count()

namespace {
//============================================= 無名名前空間 start =============================================

YPCGeneratorUnix pcgen;
YPCData pcdata;

int cur_cam_width = -1;
int cur_cam_height = -1;
const PcGenMode PC_GEN_MODE = PCGEN_GRAYPS4;
//const bool IS_INTERPO = false;
bool is_interpo=false;
std::string file_dump("/tmp/");
bool isready = false;

const float MIN_VOXEL_LEAF_SIZE = 0.001;
bool vx_enabled=false;
float vx_leaf_x = 1.0;
float vx_leaf_y = 1.0;
float vx_leaf_z = 1.0;

ros::NodeHandle *nh = nullptr;
ros::Publisher *pub1 = nullptr;
ros::Publisher *pub2 = nullptr;
ros::Publisher *pub3 = nullptr;
ros::Publisher *pub4 = nullptr;
ros::Publisher *pub5 = nullptr;

std::vector<double> vecQ;
std::vector<double> cam_K;

//int depth_base=400;
//int depth_unit=1;


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


bool get_ps_params(ros::NodeHandle *nh,std::map<std::string,double> &params,const std::string &src_key,const std::string &dst_key){
	bool ret=false;
	
	double val=0;
	if(nh->getParam(src_key,val)){
		params[dst_key]=val;
		ret=true;
	}
	return ret;
}


struct XYZW{ float x,y,z,w;};

bool operator<(const XYZW& left, const XYZW& right){ return left.w < right.w;}

union PACK{ float d[3]; char a[12];};
std::string base64encode(const std::vector<geometry_msgs::Point32> data) {
  static const char sEncodingTable[] = {
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
    'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
    'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
    'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
    'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
    'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
    'w', 'x', 'y', 'z', '0', '1', '2', '3',
    '4', '5', '6', '7', '8', '9', '+', '/'
  };
  size_t in_len=data.size();
  size_t out_len=sizeof(float)*3*in_len*4/3;
  std::string ret(out_len,'\x0');
  PACK u;
  for(int i=0,j=0;i<in_len;i++){
    u.d[0]=data[i].x;
    u.d[1]=data[i].y;
    u.d[2]=data[i].z;
    for(int k=0;k<12;k+=3){
      ret[j++]=sEncodingTable[(u.a[k]>>2)&0x3F];
      ret[j++]=sEncodingTable[((u.a[k]&0x3)<<4) | ((int)(u.a[k+1]&0xF0)>>4)];
      ret[j++]=sEncodingTable[((u.a[k+1]&0xF)<<2) | ((int)(u.a[k+2]&0xC0)>>6)];
      ret[j++]=sEncodingTable[u.a[k+2]&0x3F];
    }
  }
  return ret;
}

bool load_phase_shift_params()
{
	ROS_INFO(LOG_HEADER"phase shift parameter relod start.");
	
	const auto proc_start = std::chrono::high_resolution_clock::now() ;
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
	
	if( ! get_ps_params(nh,params,"pshift_genpc/calc/method3d","method3d")) { return -1; }
	if( ! get_ps_params(nh,params,"pshift_genpc/calc/camera_type","camera_type")) { return -1; }
	if( ! get_ps_params(nh,params,"pshift_genpc/calc/bw_diff","bw_diff")) { return -1; }
	if( ! get_ps_params(nh,params,"pshift_genpc/calc/brightness","brightness")) { return -1; }
	if( ! get_ps_params(nh,params,"pshift_genpc/calc/darkness","darkness")) { return -1; }
	if( ! get_ps_params(nh,params,"pshift_genpc/calc/phase_wd_min","phase_wd_min")) { return -1; }
	if( ! get_ps_params(nh,params,"pshift_genpc/calc/phase_wd_thr","phase_wd_thr")) { return -1; }
	if( ! get_ps_params(nh,params,"pshift_genpc/calc/gcode_variation","gcode_variation")) { return -1; }
	if( ! get_ps_params(nh,params,"pshift_genpc/calc/max_ph_diff","max_ph_diff")) { return -1; }
	if( ! get_ps_params(nh,params,"pshift_genpc/calc/max_parallax","max_parallax")) { return -1; }
	if( ! get_ps_params(nh,params,"pshift_genpc/calc/min_parallax","min_parallax")) { return -1; }
	if( ! get_ps_params(nh,params,"pshift_genpc/calc/ls_points","ls_points")) { return -1; }
	if( ! get_ps_params(nh,params,"pshift_genpc/calc/interpolation","interpolation")) { return -1; }
	
	params["image_width"]=cur_cam_width;
	params["image_height"]=cur_cam_height;
	
	for(std::map<std::string,double>::iterator it=params.begin();it!=params.end();++it){
		ROS_INFO(LOG_HEADER"(phsft) %s=%g",it->first.c_str(),it->second);
	}
	if( ! params.count("interpolation") ){
		is_interpo = false;
	}else{
		is_interpo = ((int)params["interpolation"]) ;
	}
	
	if ( ! pcgen.init(params) ) {
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
	ROS_INFO(LOG_HEADER"phase shift parameter load finished. proc_tm=%d ms",ELAPSED_TM(proc_start));
	
	return true;
}

bool load_camera_calib_data(){
	ROS_INFO(LOG_HEADER"camera calibration data load start.");
	const auto proc_start = std::chrono::high_resolution_clock::now() ;
	
	bool ret=false;
	
	CamCalibMat Kl;
	CamCalibMat Dl;
	CamCalibMat Kr;
	CamCalibMat Dr;
	CamCalibMat R;
	CamCalibMat T;
	if( ! nh->getParam("/rovi/left/genpc/K_Cols",Kl.cols) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=left, key=K_Cols");
		
	}else if( ! nh->getParam("/rovi/left/genpc/K_Rows",Kl.rows) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=left, key=K_Rows");
		
	}else if( ! nh->getParam("/rovi/left/genpc/K",Kl.values) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=left, key=K");
		
	}else if( ! Kl.validate() ){
		ROS_ERROR(LOG_HEADER"K matrix is wrong. cam=left %s", Kl.to_string().c_str());
		
	}else if( ! nh->getParam("/rovi/left/genpc/D_Cols",Dl.cols) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=left, key=D_Cols");
		
	}else if( ! nh->getParam("/rovi/left/genpc/D_Rows",Dl.rows) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=left, key=D_Rows");
		
	}else if( ! nh->getParam("/rovi/left/genpc/D",Dl.values) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=left, key=D");
		
	}else if( ! Kl.validate() ){
		ROS_ERROR(LOG_HEADER"D matix is wrong. cam=left %s", Dl.to_string().c_str());
		
	}else if( ! nh->getParam("/rovi/right/genpc/K_Cols",Kr.cols) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=K_Cols");
		
	}else if( ! nh->getParam("/rovi/right/genpc/K_Rows",Kr.rows) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=K_Rows");
		
	}else if( ! nh->getParam("/rovi/right/genpc/K",Kr.values) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=K");
		
	}else if( ! Kr.validate() ){
		ROS_ERROR(LOG_HEADER"K matrix is wrong. cam=right %s", Kr.to_string().c_str());
		
	}else if( ! nh->getParam("/rovi/right/genpc/D_Cols",Dr.cols) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=D_Cols");
		
	}else if( ! nh->getParam("/rovi/right/genpc/D_Rows",Dr.rows) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=D_Rows");
		
	}else if( ! nh->getParam("/rovi/right/genpc/D",Dr.values) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=D");
		
	}else if( ! Dr.validate() ){
		ROS_ERROR(LOG_HEADER"D matix is wrong. cam=right %s", Dr.to_string().c_str());
		
	}else if( ! nh->getParam("/rovi/right/genpc/R_Cols",R.cols) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=R_Cols");
		
	}else if( ! nh->getParam("/rovi/right/genpc/R_Rows",R.rows) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=R_Rows");
		
	}else if( ! nh->getParam("/rovi/right/genpc/R",R.values) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=R");
		
	}else if( ! R.validate() ){
		ROS_ERROR(LOG_HEADER"R matrix is wrong. cam=right %s", R.to_string().c_str());
		
	}else if( ! nh->getParam("/rovi/right/genpc/T_Cols",T.cols) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=T_Cols");
		
	}else if( ! nh->getParam("/rovi/right/genpc/T_Rows",T.rows) ){
		ROS_ERROR(LOG_HEADER"param read failed. cam=right, key=T_Rows");
		
	}else if( ! nh->getParam("/rovi/right/genpc/T",T.values) ){
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
	
	if( ! pcgen.create_camera_raw(Kl.values,Kr.values,Dl.values,Dr.values,R.values,T.values) ){
		ROS_ERROR(LOG_HEADER"stereo camera create failed.");
	}else{
		ret=true;
	}
	ROS_INFO(LOG_HEADER"camera calibration data load finished. proc_tm=%d ms",ELAPSED_TM(proc_start));
	
	return ret;
}
	
bool convert_stereo_camera_images(const rovi::GenPC::Request &req,std::vector<cv::Mat> &imgs,std::vector<unsigned char*> &img_pointers){
	bool ret=false;
	try {
		for (int i = 0, n = 0; i < 13; i++, n += 2 ) {
			cv::Mat img = cv_bridge::toCvCopy(req.imgL[i], sensor_msgs::image_encodings::MONO8)->image;
			//ROS_INFO(LOG_HEADER"[%d] left  size=%dx%d",i,img.cols,img.rows);
			imgs.push_back(img);
			img_pointers.push_back(imgs.back().data);
			
			if(file_dump.size() > 0) {
				cv::imwrite(cv::format((file_dump + "/capt%02d_0.pgm").c_str(), i), img);
			}
			
			img = cv_bridge::toCvCopy(req.imgR[i], sensor_msgs::image_encodings::MONO8)->image;
			//ROS_INFO(LOG_HEADER"[%d] right size=%dx%d",i,img.cols,img.rows);
			if(file_dump.size() > 0) {
				cv::imwrite(cv::format((file_dump + "/capt%02d_1.pgm").c_str(), i), img);
			}
			imgs.push_back(img);
			img_pointers.push_back(imgs.back().data);
		}
		if(file_dump.size() > 0) {
			FILE *f = fopen((file_dump+"/captseq.log").c_str(), "w");
			for(int j=0; j < 13; j++) {
				fprintf(f,"(%d) %d %d\n", j, req.imgL[j].header.seq, req.imgR[j].header.seq);
			}
			fclose(f);
		}
		ret=true;
	}catch (cv_bridge::Exception& e)	{
		ROS_ERROR(LOG_HEADER"genpc:cv_bridge:exception: %s", e.what());
	}
	return ret;
}


bool calc_point_cloud_norm(const sensor_msgs::PointCloud &pts,rovi::Floats &output){
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
	double gamma=1.1;
	double kn=floor((log10(N)-1)/log10(gamma));
	int Qn=N<10? N:floor(10*pow(gamma,kn));
	output.data.resize(3*Qn);
	for (int n = 0; n < Qn; n++) {
		int n3=3*n;
		output.data[n3++] = norm[n].x;
		output.data[n3++] = norm[n].y;
		output.data[n3  ] = norm[n].z;
	}
	
	return true;
}

bool updateVoxelParams(bool *updated=nullptr){
	bool ret=true;
	
	if(updated){
		*updated=false;
	}
	
	bool wk_vx_enabled=false;
	float wk_vx_leaf_x=1.0;
	float wk_vx_leaf_y=1.0;
	float wk_vx_leaf_z=1.0;
	if( ! nh->getParam("/rovi/genpc/voxelization/enabled",wk_vx_enabled)){
		wk_vx_enabled=false;
		ROS_ERROR(LOG_HEADER"voxelization param is wrong. key='/rovi/genpc/voxelization/enabled'");
		
	}else if( ! nh->getParam("/rovi/genpc/voxelization/leaf_size/x",wk_vx_leaf_x)){
		wk_vx_enabled=false;
		ROS_ERROR(LOG_HEADER"voxelization param is wrong. key='/rovi/genpc/voxelization/leaf_size/x'");
		
	}else if( ! nh->getParam("/rovi/genpc/voxelization/leaf_size/y",wk_vx_leaf_y)){
		wk_vx_enabled=false;
		ROS_ERROR(LOG_HEADER"voxelization param is wrong. key='/rovi/genpc/voxelization/leaf_size/y'");
		
	}else if( ! nh->getParam("/rovi/genpc/voxelization/leaf_size/z",wk_vx_leaf_z)){
		wk_vx_enabled=false;
		ROS_ERROR(LOG_HEADER"voxelization param is wrong. key='/rovi/genpc/voxelization/leaf_size/z'");
		
	}
	
	if(updated){
		if( wk_vx_enabled != vx_enabled || 
			wk_vx_leaf_x != vx_leaf_x || 
			wk_vx_leaf_y != vx_leaf_y || 
			wk_vx_leaf_z != vx_leaf_z ){
			*updated=true;
		}
	}
	
	vx_enabled = wk_vx_enabled;
	vx_leaf_x = std::max(wk_vx_leaf_x,MIN_VOXEL_LEAF_SIZE);
	vx_leaf_y = std::max(wk_vx_leaf_y,MIN_VOXEL_LEAF_SIZE);
	vx_leaf_z = std::max(wk_vx_leaf_z,MIN_VOXEL_LEAF_SIZE);
	
	ROS_INFO(LOG_HEADER"(voxel) enabled=%d leaf_x=%g leaf_y=%g leaf_z=%g",vx_enabled,vx_leaf_x,vx_leaf_y,vx_leaf_z);
	
	return true;
}

bool genpc(rovi::GenPC::Request &req, rovi::GenPC::Response &res)
{
	const auto node_start = std::chrono::high_resolution_clock::now() ;
	ROS_INFO(LOG_HEADER"start: image_width=%d image_height=%d", (int)req.imgL.size(), (int)req.imgR.size());
	
	pcdata = YPCData();
	
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
	
	
	sensor_msgs::PointCloud pts;
	pts.header.stamp = ros::Time::now();
	pts.header.frame_id = "/camera";
	std_msgs::String b64;
	rovi::Floats buf;
	res.pc_cnt = 0;
	sensor_msgs::PointCloud pts_vx;
	
	if( ! load_phase_shift_params() ){
		ROS_ERROR(LOG_HEADER"phase shift parameter load failed.");
		
	}else if( ! updateVoxelParams() ){
		ROS_ERROR(LOG_HEADER"voxel parameter update failed.");
		
	}else if ( ! isready ) {
		ROS_INFO(LOG_HEADER"current camera resolution. w=%d h=%d", cur_cam_width, cur_cam_height);
		if( ! load_camera_calib_data() ){
			ROS_ERROR(LOG_HEADER"camera calibration data load failed.");
		}else{
			ROS_INFO(LOG_HEADER"camera calibration data loaded.");
			isready=true;
		}
	}
	
	std::vector<cv::Mat> imgs;
	std::vector<unsigned char*> img_pointers;
	sensor_msgs::ImagePtr depthimg;
	cv::Mat depthimg_mat;
	
	if( ! isready ){
		ROS_ERROR(LOG_HEADER"camera calibration data load failed. elapsed=%d",ELAPSED_TM(node_start));
		
	}else if( ! convert_stereo_camera_images(req,imgs,img_pointers) ){
		ROS_ERROR(LOG_HEADER"stereo camera image convert failed. elapsed=%d",ELAPSED_TM(node_start));
		
	}else{
		ROS_INFO(LOG_HEADER"point cloud generation start. interpolation=%s",is_interpo?"enabled":"disabled");
		const auto genpc_start = std::chrono::high_resolution_clock::now() ;
		const int N = pcgen.generate_pointcloud(img_pointers,is_interpo,&pcdata);
		
		ROS_INFO(LOG_HEADER"point cloud generation finished. point_num=%d, diparity_tm=%d, ms genpc_tm=%d ms, total_tm=%d ms, elapsed=%d ms",
			N, DURATION_TO_MS(pcgen.get_elapsed_disparity()), DURATION_TO_MS(pcgen.get_elapsed_genpcloud()),
			ELAPSED_TM(genpc_start), ELAPSED_TM(node_start));
		
		if( pcdata.is_empty() ){
			ROS_INFO(LOG_HEADER"genpc point count 0. elapsed=%d ms", ELAPSED_TM(node_start));
			
		}else{
			//点群データ変換
			if( true ){
				ROS_INFO(LOG_HEADER"point cloud data convert start.");
				const auto conv_start = std::chrono::high_resolution_clock::now();
				if( ! pcdata.make_point_cloud(pts) ){
					ROS_ERROR(LOG_HEADER"point cloud data convert failed.");
				}
				ROS_INFO(LOG_HEADER"point cloud data convert finished.proc_tm=%d ms, elapsed=%d ms",
					ELAPSED_TM(conv_start), ELAPSED_TM(node_start));
			}
			
			//normalization
			if( true ){
				ROS_INFO(LOG_HEADER"point cloud clac normalization start.");
				
				const auto norm_start = std::chrono::high_resolution_clock::now() ;
				if( ! calc_point_cloud_norm(pts,buf) ){
					ROS_ERROR(LOG_HEADER"point cloud clac normalization failed.");
				}
				ROS_INFO(LOG_HEADER"point cloud clac normalization finished. proc_tm=%d ms, elapsed=%d ms",
					ELAPSED_TM(norm_start), ELAPSED_TM(node_start));
			}
			
			//voxelization
			if( ! vx_enabled ){
				ROS_INFO(LOG_HEADER"point cloud voxelization skipped.");
			}else{
				ROS_INFO(LOG_HEADER"point cloud voxelization start.");
		
				const auto voxel_start = std::chrono::high_resolution_clock::now() ;
				
				if ( ! pcdata.voxelization( vx_leaf_x,vx_leaf_y,vx_leaf_z,pts_vx ) ){
					ROS_ERROR(LOG_HEADER"point cloud voxelization failed. elapsed=%d",ELAPSED_TM(node_start));
				}
				
				ROS_INFO(LOG_HEADER"point cloud voxelization finished. point_num=%d / %d(%.1f%%), proc_tm=%d ms, elapsed=%d ms",
					(int)pts_vx.points.size(), N, N==0?0:pts_vx.points.size()/(float)N *100,
					ELAPSED_TM(voxel_start), ELAPSED_TM(node_start));
			}
			
			//depthmap making
			if( true ){
				ROS_INFO(LOG_HEADER"depthmap image create start.");
				const auto depthimg_start = std::chrono::high_resolution_clock::now() ;
				
				if( ! pcdata.make_depth_image(depthimg_mat) ){
					ROS_ERROR(LOG_HEADER"depthmap image make failed.");
				}else{
					try{
						depthimg = cv_bridge::CvImage(std_msgs::Header(),"mono16",depthimg_mat).toImageMsg();
					}catch (cv_bridge::Exception& e)	{
						ROS_ERROR(LOG_HEADER"depthmap image cv_bridge:exception: %s", e.what());
					}
				}
				
				ROS_INFO(LOG_HEADER"depthmap image create finished. proc_tm=%d ms",ELAPSED_TM(depthimg_start));
			}
			b64.data=base64encode(pts.points);
			res.pc_cnt = N;
		}
	}
	
	pub1->publish(pts);
	pub2->publish(b64);
	pub3->publish(buf);
	pub4->publish(depthimg);
	pub5->publish(pts_vx);
	
	ROS_INFO(LOG_HEADER "publish finished. elapsed=%d ms",ELAPSED_TM(node_start));
	
	if( ! pcdata.is_empty() &&  ! file_dump.empty() ) {
		if( true ){
			const auto save_start = std::chrono::high_resolution_clock::now() ;
			
			const std::string save_file_path=file_dump + "/test.ply";
			if( ! pcdata.save_ply(save_file_path) ){
				ROS_ERROR(LOG_HEADER"ply file save failed. proc_tm=%d ms, path=%s",
					ELAPSED_TM(save_start), save_file_path.c_str());
			}else{
				ROS_INFO(LOG_HEADER"ply file save succeeded. proc_tm=%d ms, path=%s",
					ELAPSED_TM(save_start), save_file_path.c_str());
			}
		}
		//todo:************* pending *************
		//writePLY(file_dump + "/testRG.ply", pcdP, N, pcgenerator->get_rangegrid(), width, height);
		//ROS_INFO("after  outPLY");
		
		if( ! pts_vx.points.empty() ){
			const auto save_start = std::chrono::high_resolution_clock::now() ;
			
			const std::string save_file_path=file_dump + "/voxel.ply";
			sensor_msgs::PointCloud2 pts2_vx;
			sensor_msgs::convertPointCloudToPointCloud2(pts_vx,pts2_vx);
			pcl::PointCloud<pcl::PointXYZRGB> pts_vx_pcl;
			pcl::fromROSMsg(pts2_vx, pts_vx_pcl);
			
			pcl::PLYWriter ply_writer;
			int save_ret=0;
			if( save_ret =  ply_writer.write<pcl::PointXYZRGB> (save_file_path, pts_vx_pcl, true) ){
				ROS_ERROR(LOG_HEADER"voxelized point cloud data save failed. ret=%d, proc_tm=%d ms, path=%s",
					save_ret, ELAPSED_TM(save_start), save_file_path.c_str());
			}else{
				ROS_INFO(LOG_HEADER"voxelized point cloud data save succeeded. proc_tm=%d ms, path=%s",
					ELAPSED_TM(save_start), save_file_path.c_str());
			}
		}
		
		//depthmap image save
		if( ! depthimg_mat.empty() ){
			const auto save_start = std::chrono::high_resolution_clock::now() ;
			std::string save_file_path = file_dump + "/depth.png";
			if( ! cv::imwrite(save_file_path,depthimg_mat) ){
				ROS_ERROR(LOG_HEADER"depthmap image save failed. proc_tm=%d ms, path=%s",ELAPSED_TM(save_start), save_file_path.c_str());
			}else {
				ROS_INFO(LOG_HEADER"depthmap image make succeeded. proc_tm=%d ms, path=%s",ELAPSED_TM(save_start), save_file_path.c_str());
			}
		}
		
	}

	ROS_INFO(LOG_HEADER "node end. elapsed=%d ms",ELAPSED_TM(node_start));
	return true;
}

	
//============================================= 無名名前空間  end  =============================================
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "genpc_node");
	ros::NodeHandle n;
	nh = &n;
	
	if( ! pcgen.create_pcgen(PC_GEN_MODE) ){
		ROS_ERROR(LOG_HEADER"point cloud generator create failed.");
		return 2;
	}
  
	ros::ServiceServer svc1 = n.advertiseService("genpc", genpc);
	ros::Publisher p1 = n.advertise<sensor_msgs::PointCloud>("ps_pc", 1);
	pub1 = &p1;
	ros::Publisher p2 = n.advertise<std_msgs::String>("ps_base64", 1);
	pub2 = &p2;
	ros::Publisher p3 = n.advertise<rovi::Floats>("ps_floats", 1);
	pub3 = &p3;
	ros::Publisher p4 = n.advertise<sensor_msgs::Image>("image_depth", 1);
	pub4 = &p4;
	ros::Publisher p5 = n.advertise<sensor_msgs::PointCloud>("ps_pc_vx", 1);
	pub5 = &p5;
	ros::spin();
	
	return 0;
}
