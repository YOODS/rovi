#include "YPCData.hpp"

#include <ros/types.h>
#include <ros/ros.h>
#include <pcl_ros/filters/voxel_grid.h>
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
	
	//ROS_INFO(LOG_HEADER"point cloud data generated. step=%d, width=%d, height=%d, points_size=%d, n_valid=%d",
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
	return true;
}

bool YPCData::voxelization(const float leaf_x,const float leaf_y,const float leaf_z, sensor_msgs::PointCloud &pts){
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcdata_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcdata_pcl->width = this->n_valid;
	pcdata_pcl->height = 1;
	pcdata_pcl->points.resize(this->n_valid);
	
	const std::vector<PointCloudCallback::Point3d> &pcdP=this->points;
	for (int i = 0,n = 0 ; i < this->points.size(); i++) {
		if( n  < this->n_valid  && ! std::isnan(pcdP[i].x) ){
			pcl::PointXYZRGB * pos = pcdata_pcl->points.data() + n;
			pos->x = pcdP[i].x;
			pos->y = pcdP[i].y;
			pos->z = pcdP[i].z;
			pos->r = pos->g = pos->b =  this->image[i];
			n++;
		}
	}
	//const sensor_msgs::PointCloud2ConstPtr input2;
	//sensor_msgs::PointCloud2::Ptr pts2;
	//pcl::PointCloud<pcl::PointXYZ> cloud2;
	//pcl::fromROSMsg (*pts2, cloud2);

	pcl::PointCloud<pcl::PointXYZRGB> vx_points;
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud (pcdata_pcl);
	sor.setLeafSize (leaf_x, leaf_y, leaf_z);
	sor.filter (vx_points);
	
	const Eigen::Vector3i minBoxCoord= sor.getMinBoxCoordinates();
	const Eigen::Vector3i maxBoxCoord= sor.getMaxBoxCoordinates();

	bool ret = false;
	if( minBoxCoord[0] == 0 && minBoxCoord[1] == 0 && minBoxCoord[2] == 0 &&
		maxBoxCoord[0] == 0 && maxBoxCoord[1] == 0 && maxBoxCoord[2] == 0){
		std::cerr << "voxelization failure. out of memory? leaf_size=(" << leaf_x << ", " << leaf_y << ", " << leaf_z << ")" << std::endl;
	
	}else{
		ret = true;
		sensor_msgs::PointCloud2 pts2_vx;
		pcl::toROSMsg(vx_points, pts2_vx);
		sensor_msgs::convertPointCloud2ToPointCloud(pts2_vx,pts);
	}
	
	return ret;
}

bool YPCData::save_ply(const std::string &file_path){
	PLYSaver saver(file_path);
	saver(this->image, this->step, this->width, this->height, this->points, this->n_valid);
	return saver.is_ok();
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
