#include "YPCData.hpp"

#include <numeric>
#include <ros/types.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/image_encodings.h>

#include "YPCGenerator.hpp"

namespace {
	const int DEPTH_BASE=400;
	const int DEPTH_UNIT=1;
	int isLittleEndian(void){
	    unsigned i = 1;
	    return *((char *)&i);
	}
	/*
#pragma pack(1)
	struct VertexXYZRGB{
		//float x=0;
		//float y=0;
		//float z=0;
		//float rgb=0;
		float x=NAN;
		float y=NAN;
		float z=NAN;
		float rgb=0;
	} __attribute__((__packed__)) __attribute__((aligned(1)));
#pragma pack()
	*/
}

YPCData::YPCData():
	image(nullptr),
	width(0),
	height(0),
	n_valid(0)
{
}

YPCData::~YPCData(){
	
}

bool YPCData::is_empty()const{
	return n_valid <= 0 ;
}

int YPCData::count()const{
	return n_valid;
}

void YPCData::operator()(
	unsigned char *image, const size_t step,
	const int width, const int height, 
	std::vector<Point3d> &points, const int n_valid){
	
	//ROS_WARN("pcgen callback. step=%d, width=%d, height=%d, points_size=%d, n_valid=%d",
	//	(int)step,width,height,(int)points.size(),n_valid);
		
	this->image=image;
	this->step=step;
	this->width=width;
	this->height=height;
	this->points=points;
	this->n_valid=n_valid;
}


sensor_msgs::Image YPCData::texture_image()const{
	sensor_msgs::Image img;
	img.header.stamp     = ros::Time::now();
	img.header.frame_id  = "camera";
	img.height           = height;
	img.width            = width;
	img.encoding         = sensor_msgs::image_encodings::MONO8;
	img.is_bigendian     = false;
	img.step             = width;
	const int len=width*height;
	img.data.assign(len,0);
	std::memcpy(img.data.data(),image,len);
	return img;
}
	
bool YPCData::make_point_cloud(sensor_msgs::PointCloud &pts,const bool dense){
	
	int point_count = this->n_valid;
	if( ! dense ){
		point_count = std::max(0, width * height);
	}
	
	pts.header.stamp     = ros::Time::now();
	pts.header.frame_id  = "camera";
	pts.points.resize(point_count);

	pts.channels.resize(1);
	pts.channels[0].name = "rgb";
	pts.channels[0].values.resize(point_count);
	
	if( point_count <= 0 ){
		return true;
	}
	
	float *pChRGB=pts.channels[0].values.data();
	uint32_t rgb=0;
	
	const Point3d *org_points = this->points.data();
	if( dense ){
		for (int i = 0,n = 0 ; i < this->points.size(); i++) {
			const Point3d * org_point = org_points + i;
			const unsigned char * pixel = this->image + i ;
			
			if( n  < point_count  && ! std::isnan(org_point->x) ){
				pts.points[n].x = org_point->x;
				pts.points[n].y = org_point->y;
				pts.points[n].z = org_point->z;

				rgb = ( *pixel << 16 | *pixel << 8 | *pixel);
				//pts.channels[0].values[n] =*reinterpret_cast<float*>(&rgb);
				*(pChRGB + n) = *reinterpret_cast<float*>(&rgb);
				n++;
			}
		}
	}else{
		for (int i = 0; i < this->points.size(); i++) {
			const Point3d * org_point = org_points + i;
			const unsigned char * pixel = this->image + i ;
			
			pts.points[i].x = org_point->x;
			pts.points[i].y = org_point->y;
			pts.points[i].z = org_point->z;

			rgb = ( *pixel << 16 | *pixel << 8 | *pixel);
			//pts.channels[0].values[n] =*reinterpret_cast<float*>(&rgb);
			*(pChRGB + i) = *reinterpret_cast<float*>(&rgb);
		}
	}
	
	return true;
}

bool YPCData::make_point_cloud2(sensor_msgs::PointCloud2 &pts,const bool dense){

	
	pts = sensor_msgs::PointCloud2();
	pts.header.stamp = ros::Time::now();
	pts.header.frame_id = "camera";
	
	const float field_data_size= 4;
	const int field_num= 4;
	
	if( sizeof(float) != field_data_size ){
		ROS_ERROR("sensor_msgs::PointCloud2 point field data size is different.");
		return false;
	}
	
	
	pts.fields.resize(field_num);
	pts.fields[0].name = "x";
	pts.fields[0].offset = 0;
	pts.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
	pts.fields[0].count = 1;
	pts.fields[1].name = "y";
	pts.fields[1].offset = field_data_size;
	pts.fields[1].datatype =  sensor_msgs::PointField::FLOAT32;
	pts.fields[1].count = 1;
	pts.fields[2].name = "z";
	pts.fields[2].offset = field_data_size * 2;
	pts.fields[2].datatype =  sensor_msgs::PointField::FLOAT32;
	pts.fields[2].count = 1;
	pts.fields[3].name = "rgb";
	pts.fields[3].offset = field_data_size * 3;
	pts.fields[3].datatype =  sensor_msgs::PointField::FLOAT32;
	pts.fields[3].count = 1;
	
	pts.point_step = field_data_size * field_num ;
		
	int point_count = 0;
	if( ! dense ){
		point_count = std::max(0, width * height);
		pts.width = width;
		pts.height = height;
		pts.row_step = width * pts.point_step;
	}else{
		point_count = this->n_valid;
		pts.width = point_count;
		pts.height = 1;
		pts.row_step = point_count * pts.point_step;
	}
	
	pts.is_dense = dense;
	pts.is_bigendian = isLittleEndian()?false:true;
	
	pts.data.assign( point_count * pts.point_step,{});
		
	const Point3d * pPoints = this->points.data();
	const Point3d * pPoint = nullptr;
	unsigned char pixel = 0;
	float *vertices=(float*)pts.data.data();
	int32_t rgb=0;
	if( dense ){
		for ( int i = 0,n = 0 ; i < this->points.size(); i++ ) {
			pPoint = pPoints + i;
			pixel = *(this->image + i);
			
			if( n  < point_count  && ! std::isnan(pPoint->x) ){
				float * vtx = vertices + n * field_num;
				*(vtx) = pPoint->x;
				*(vtx+1) = pPoint->y;
				*(vtx+2) = pPoint->z;
				
				rgb = (pixel << 16) | (pixel << 8) | pixel; 
				*(vtx+3) = *reinterpret_cast<float*>(&rgb);
				n++;
			}
		}
	}else{
		for ( int i = 0 ; i < this->points.size(); i++ ) {
			pPoint = pPoints + i;
			pixel = *(this->image + i);
			
			float * vtx = vertices + i * field_num;
			*(vtx) = pPoint->x;
			*(vtx+1) = pPoint->y;
			*(vtx+2) = pPoint->z;
			
			rgb = (pixel << 16) | (pixel << 8) | pixel; 
			*(vtx+3) = *reinterpret_cast<float*>(&rgb);
		}
	}
	
	//std::cerr << "fileds=" <<  p2.fields.size() << std::endl;
	return true;
}

bool YPCData::make_depth_image(cv::Mat &img){
	//fprintf(stderr,"width=%d height=%d\n",this->height,this->width);

	const long base=DEPTH_BASE*256;
	img=cv::Mat(this->height,this->width,CV_16UC1,cv::Scalar(std::numeric_limits<unsigned short>::max()));
	
	if(this->n_valid <= 0 ){
		return true;
	}
	
	int n = 0;
	for (int j = 0; j < this->height; j++) {
		unsigned short *dP = img.ptr<unsigned short>(j);
		for (int i = 0; i < this->width; i++, n++) {
			if (std::isnan(this->points[n].x)) continue;
			long d = std::round((DEPTH_UNIT * this->points[n].z - DEPTH_BASE)*256);
			if(d<0) dP[i] = 0;
			else if(d<65536L) dP[i]=d;
		}
	}
#if 0

// 奥行きの最小最大を求める
	float min_ = std::numeric_limits<float>::max();
	float max_ = -min_;
	for (auto p : points) {
		if (std::isnan(p.x)) continue;
		if (min_ > p.z) min_ = p.z;
		if (max_ < p.z) max_ = p.z;
	}

	// 正規化用の係数を求める. 255は、測定できなかった点に入れる
	float scale = 254.0f / (max_ - min_);
	
	img = cv::Mat(this->height,this->width, CV_8U);
	for (int j = 0, n = 0; j < height; j++) {
		uchar *dP = img.ptr<uchar>(j);
		for (int i = 0; i < width; i++, n++) {
			if (std::isnan(points[n].x)) {
				dP[i] = 255;
				continue;
			}
			
			dP[i] = ((int) (scale * (points[n].z - min_) + 0.5f));
		}
	}
#endif
	
	return true;
}

bool YPCData::save_ply(const std::string &file_path,const bool dense)const{
	//PLYSaver saver(file_path);
	//saver(this->image, this->step, this->width, this->height, this->points, this->n_valid);
	//return saver.is_ok();
	
	std::ofstream ofs(file_path, std::ios::binary);
	if ( ! ofs.is_open() ) {
		return false;
	}
	
	ofs << "ply\n";
	ofs << "format binary_little_endian 1.0\n";
	//ofs << "comment VCGLIB generated\n";
	ofs << "obj_info num_cols " << width << std::endl;
	ofs << "obj_info num_rows " << height << std::endl;
	ofs << "element vertex " << ( dense ? n_valid : points.size()) << std::endl;
	ofs << "property float x\n";
	ofs << "property float y\n";
	ofs << "property float z\n";
	ofs << "property uchar red\n";
	ofs << "property uchar green\n";
	ofs << "property uchar blue\n";
	ofs << "element face 0\n";
	ofs << "end_header\n";

	unsigned char *pimage=image;
	
	int count=0;
	if(dense){
		for (int j = 0, n = 0; j < height; j++) {
			unsigned char *iP = pimage;
			for (int i = 0; i < width; i++, n++) {
				float pos[3] = {0, 0, 0};
				unsigned char col[3] = {iP[i], iP[i], iP[i]};
					
				if ( ! std::isnan(points[n].x) ) {
					pos[0] = points[n].x;
					pos[1] = points[n].y;
					pos[2] = points[n].z;
					
					ofs.write((char *)pos, sizeof(float) * 3);
					ofs.write((char *)col, 3);
					count++;
				}
			}
			pimage += step;
		}
	}else{
		for (int j = 0, n = 0; j < height; j++) {
			unsigned char *iP = pimage;
			for (int i = 0; i < width; i++, n++) {
				float pos[3] = {0, 0, 0};
				unsigned char col[3] = {iP[i], iP[i], iP[i]};
					
				pos[0] = points[n].x;
				pos[1] = points[n].y;
				pos[2] = points[n].z;
				
				ofs.write((char *)pos, sizeof(float) * 3);
				ofs.write((char *)col, 3);
				count++;
			}
			pimage += step;
		}
	}
	

	ofs.close();
	bool ret=false;
	if(dense){
		ret = count == n_valid;
	}else{
		ret = count == points.size();
	}
	return ret;
}

rovi::Floats YPCData::to_rg_floats()const{
	rovi::Floats pt_floats;
	
	if( ! this->points.empty() ){
		const int num = this->points.size();
		pt_floats.data.assign( num * 3 , 0);
		const Point3d *pts = this->points.data();
		for( int i = 0,n = 0 ; i < num ; ++i){
			const Point3d *pt = pts + i;
			if( std::isnan(pt->x) ){
				n+=3;
			}else{
				pt_floats.data[  n] = pt->x;
				pt_floats.data[++n] = pt->y;
				pt_floats.data[++n] = pt->z;
				++n;
			}
		}
	}
	
	return pt_floats;
}
