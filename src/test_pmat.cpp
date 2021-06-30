#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <opencv2/opencv.hpp>
#include <stdlib.h>


#define GET_CAMERA_LABEL(camno) (camno==0?'L':(camno==1?'R':'?'))
constexpr int CAMERA_NUM = 2;

ros::NodeHandle *nh;
static std::vector<std::string> paramP = {"test/left/P", "test/right/P"};
static std::vector<double> pvec[CAMERA_NUM];


void reload(std_msgs::Bool e)
{
	ROS_INFO("test_node LD_PATH=%s",getenv("LD_LIBRARY_PATH"));
	
	for( int camno=0; camno < CAMERA_NUM; ++camno ){
    	if (! nh->getParam(paramP[camno].c_str(), pvec[camno])) {
    		ROS_ERROR("test_node::parameter \"P(%c)\" not found",GET_CAMERA_LABEL(camno));
    		return;
    	}
        std::cout << "test_node::parameter pvec(" << GET_CAMERA_LABEL(camno) << ")=";
        for (size_t i = 0; i < pvec[camno].size(); ++i) {
            std::cout << pvec[camno][i] << "; ";
        }
        std::cout << std::endl;
	}
}


int main(int argc, char **argv)
{
	for( int camno=0; camno < CAMERA_NUM; ++camno ){
        if (argc >= camno + 2) {
    		paramP[camno] = argv[camno+1];
    	}
        std::cout << "P(" << GET_CAMERA_LABEL(camno) << ")=" << paramP[camno] << "\n";
	}

	ros::init(argc, argv,"test_pmat_node");
	ros::NodeHandle n;
	nh = &n;

	ros::Subscriber s1=n.subscribe("test/pmat", 1, reload);
	std_msgs::Bool msg;
	reload(msg);

	for( int camno=0; camno < CAMERA_NUM; ++camno ){
    	cv::Mat pmat(pvec[camno]);
	    cv::Mat P=pmat.reshape(1,3);
        std::cout << "test_node::matrix P(" << GET_CAMERA_LABEL(camno) << ")=" << P << std::endl;
    }

	ros::spin();

	return 0;
}
