#include <ros/ros.h>
#include <pid_wrapper.h>


int main(int argc, char **argv)
{
    ros::Time::init();
    ros::init(argc, argv,"pid_controller_node");
    ros::NodeHandle nh("~airsim");
    ros::NodeHandle nh_private("~");

    pid_wrapper(nh, nh_private);

    // ros::spin();

    return 0;
}