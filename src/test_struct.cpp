#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <stdlib.h>


struct marker{
    int seq;
    int x;
    int y;
    void set_data(int seq, int x = -1, int y = -1){
        this->seq = seq;
        this->x = x;
        this->y = y;
        this->print_data();
    }
    void print_data()
    {
        ROS_INFO("test_node::print_data seq=%d x=%d y=%d", this->seq, this->x, this->y);
    }
    bool check_data()
    {
      	bool ret=false;
        if ((this->x < 0) || (this->y < 0)) {
	    	ROS_ERROR("test_node::check NG. (%d, %d)", this->x, this->y);
    	}else{
	    	ROS_INFO("test_node::check OK. (%d, %d)", this->x, this->y);
            ret=true;
	    }
        return ret;    
    }
};

ros::NodeHandle *nh;
static marker mak;

void test(std_msgs::Bool e)
{
    mak.set_data(1,2,3);
    mak.check_data();

    mak.set_data(2);
    mak.check_data();

    mak.set_data(3,4,5);
    mak.check_data();

    mak.set_data(4);
    mak.check_data();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv,"test_struct_node");
	ros::NodeHandle n;
	nh = &n;

	ros::Subscriber s1=n.subscribe("test/struct", 1, test);

	std_msgs::Bool msg;
	test(msg);

	ros::spin();

	return 0;
}
