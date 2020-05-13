#include <twist2stamp.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist2stamp");
    ros::NodeHandle nh("");
    converter::converter conversion(nh);

    // ros::spin();

    return 0;
}
