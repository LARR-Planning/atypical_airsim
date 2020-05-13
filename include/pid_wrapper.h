#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <car_pid.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <driving_msgs/VehicleCmd.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>


struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q);
Quaternion ToQuaternion(double yaw, double pitch, double roll); // yaw (Z), pitch (Y), roll (X)


class pid_wrapper
{
    public:
    pid_wrapper(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~pid_wrapper(){};
    
    void acc_cmd_cb(const geometry_msgs::TwistStamped& msg);
    void odom_cb(const nav_msgs::Odometry &msg);
    void twist_cb(const geometry_msgs::TwistStamped &msg);

    void loop();

    void des_path_gen(double time_step, bool update);
    geometry_msgs::Pose dynamics(geometry_msgs::Pose prev_pose, geometry_msgs::TwistStamped input, double time_step);
    driving_msgs::VehicleCmd convert_twst_to_vehicle_cmd(geometry_msgs::Twist twist)
    {
        driving_msgs::VehicleCmd cmd;
        cmd.accel_decel_cmd = twist.linear.x;
        cmd.steer_angle_cmd = twist.angular.z;
        return cmd;
    }

    private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;    

    ros::Subscriber sub_des_acc; //Subscriber for desirced accerelation(high level control)
    ros::Subscriber sub_odom; //Current odometry (pose)
    ros::Subscriber sub_vel; //Current velocity (twist)
    ros::Publisher  pub_acc_cmd; //Publish Vehicle Cmd
    ros::Publisher pub_vis_path;
    
    geometry_msgs::TwistStamped prev_acc_cmd;
    nav_msgs::Odometry cur_odom;
    geometry_msgs::TwistStamped cur_twist;
    geometry_msgs::Pose prev_pose;

    nav_msgs::Path des_path;
    nav_msgs::Path cur_path;

    Controller pid_controller;

    tf::Transform transform;    

    //Parameter
    double kp; double ki; double kd; 
    double steer_ratio;
    double update_period;

    double v = 0.0; //Body frame velocity
    double w = 0.0; //Body frame angular velocity


    double last_time;
    double check_point; // Check point for Receding-Horizon update of des_path
    
    double L = 2.32; //Length of car

    tf::TransformListener listener;
    // tf::StampedTransform transform;
    geometry_msgs::Vector3Stamped vec, vec2;

};