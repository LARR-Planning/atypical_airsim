#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

namespace converter{
    class converter
    {
        private:
        ros::NodeHandle nh_;
        ros::Publisher pub_stamped;
        ros::Subscriber sub_non_stamped;

        geometry_msgs::Twist sub_twist;
        geometry_msgs::TwistStamped pub_twist;
        bool has_twist = false;
        // ros::Rate loop_rate(50);

        public:
        void callback_non_stamped(const geometry_msgs::Twist& non_stamped)
        {
            sub_twist.angular = non_stamped.angular;
            sub_twist.linear = non_stamped.linear;
            // std::cout << "Called" << std::endl;
            has_twist = true;
        }

        converter(ros::NodeHandle& nh) : nh_(nh)
        {
            sub_non_stamped = nh.subscribe("/acc_cmd", 100, &converter::converter::callback_non_stamped, this); 
            pub_stamped = nh.advertise<geometry_msgs::TwistStamped>("/acc_cmd_body_frame", 100);

            while(ros::ok())
            {   ros::spinOnce();
                ros::Rate loop_rate(100);
                // ROS_INFO("%d", has_twist);
                if(has_twist)
                {
                    conversion(sub_twist);
                }
	    // ROS_INFO("%f", pub_twist.twist.linear.x);
                
                // ros::spinOnce();
                loop_rate.sleep();
            }
        }

        void conversion(geometry_msgs::Twist non_stamped)
        {
            ros::Time curr_ros_time = ros::Time::now();

            pub_twist.header.frame_id = "world_ent";
            pub_twist.header.stamp = curr_ros_time;
            pub_twist.twist.angular.x = non_stamped.angular.x;
            pub_twist.twist.angular.y = non_stamped.angular.y;
            pub_twist.twist.angular.z = non_stamped.angular.z;

            pub_twist.twist.linear.x = non_stamped.linear.x;
            pub_twist.twist.linear.y = non_stamped.linear.y;            
            pub_twist.twist.linear.z = non_stamped.linear.z;
            // ROS_INFO("Hello");
            pub_stamped.publish(pub_twist);
        }


    };
}
