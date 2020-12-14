#include "YPCData.hpp"

#include <ros/types.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "YPCGenerator.hpp"

namespace {
	const int DEPTH_BASE=400;
	const int DEPTH_UNIT=1;
	
	//sample:将来的にはPointCloud2へ
	/*
#pragma pack(1)
	struct VertexXYZRGB{
		float x=0;
		float y=0;
		float z=0;
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

//sample:将来的にはPointCloud2へ
/*
const sensor_msgs::PointCloud2 * YPCData::get_data() const{
	return &pcdata;
}
*/

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


bool YPCData::make_point_cloud(sensor_msgs::PointCloud &pts){
	
	const int N = this->n_valid;
	
	pts.points.resize(N);
	pts.channels.resize(3);
	pts.channels[0].name = "r";
	pts.channels[0].values.resize(N);
	pts.channels[1].name = "g";
	pts.channels[1].values.resize(N);
	pts.channels[2].name = "b";
	pts.channels[2].values.resize(N);
	
	if(N <= 0 ){
		return true;
	}
	
	// building point cloud, getting center of points, and getting norm from the center
	const Point3d *org_points = this->points.data();
	for (int i = 0,n = 0 ; i < this->points.size(); i++) {
		const Point3d * org_point = org_points + i;
		const unsigned char * pixel = this->image + i ;
		
		if( n  < N  && ! std::isnan(org_point->x) ){
			pts.points[n].x = org_point->x;
			pts.points[n].y = org_point->y;
			pts.points[n].z = org_point->z;
			
			//グレースケールしか対応していない
			pts.channels[0].values[n] = pts.channels[1].values[n] = pts.channels[2].values[n] =  *pixel / 255.0;
			n++;
		}
	}
	//sample:将来的にはPointCloud2へ
	/*
	pcdata = sensor_msgs::PointCloud2();
	pcdata.header.stamp = ros::Time::now();
	pcdata.fields.resize(4);
	pcdata.fields[0].name = "x";
	pcdata.fields[0].offset = 0;
	pcdata.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
	pcdata.fields[0].count = 1;
	pcdata.fields[1].name = "y";
	pcdata.fields[1].offset = 4;
	pcdata.fields[1].datatype =  sensor_msgs::PointField::FLOAT32;
	pcdata.fields[1].count = 1;
	pcdata.fields[2].name = "z";
	pcdata.fields[2].offset = 8;
	pcdata.fields[2].datatype =  sensor_msgs::PointField::FLOAT32;
	pcdata.fields[2].count = 1;
	pcdata.fields[3].name = "rgb";
	pcdata.fields[3].offset = 12;
	pcdata.fields[3].datatype =  sensor_msgs::PointField::FLOAT32;
	pcdata.fields[3].count = 1;
	
	pcdata.point_step = 16;
	pcdata.width = N;
	pcdata.height = 1;
	pcdata.row_step = N;
	pcdata.is_dense= true;
	pcdata.is_bigendian = true;
	const int data_size = sizeof(VertexXYZRGB);
	pcdata.data.assign(N * data_size,0);

	std::cerr << "pcdata_size=" << pcdata.data.size() << std::endl;
	
	VertexXYZRGB *vertices=(VertexXYZRGB*)pcdata.data.data();
	bool once=false;
	for (int i = 0,n = 0 ; i < this->points.size(); i++) {
		const Point3d * org_point = org_points + i;
		const unsigned char pixel = *(this->image + i);
		
		if( n  < N  && ! std::isnan(org_point->x) ){
			VertexXYZRGB * vtx = vertices + n;
			vtx->x = org_point->x;
			vtx->y = org_point->y;
			vtx->z = org_point->z;
			
			int32_t rgb = (pixel << 16) | (pixel << 8) | pixel; 
			vtx->rgb = *(float *)(&rgb);
			n++;
		}
	}
	
	//std::cerr << "fileds=" <<  p2.fields.size() << std::endl;
	*/
	
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

bool YPCData::save_ply(const std::string &file_path)const{
	//PLYSaver saver(file_path);
	//saver(this->image, this->step, this->width, this->height, this->points, this->n_valid);
	//return saver.is_ok();
	
	std::ofstream ofs(file_path, std::ios::binary);
	if ( ! ofs.is_open() ) {
		return false;
	}
	
	ofs << "ply\n";
	ofs << "format binary_little_endian 1.0\n";
	ofs << "comment VCGLIB generated\n";
	ofs << "element vertex " << n_valid << std::endl;
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

	ofs.close();
	
	return count == n_valid;
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
