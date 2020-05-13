#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include <math_common.h>
#include <algorithm>
using namespace std;

class YawController
{
    private:
        double wheel_base_ = 2.32; //38cm 
        double steer_ratio_;
        double min_speed_ = 0.1;
        double max_lat_accel_ = 3.0;

        double min_angle_ = -8;
        double max_angle_ = 8;

    public:
        // YawController(double wheel_base, double steer_ratio, double min_speed, double max_lat_accel, double min_angle, double max_angle )
        //  : wheel_base_(wheel_base), steer_ratio_(steer_ratio), min_speed_(min_speed), max_lat_accel_(max_lat_accel), min_angle_(min_angle), max_angle_(max_angle)
        // {

        // }
        double get_angle(double radius)
        {
            double angle = atan(wheel_base_/radius ) * steer_ratio_;
            // cout << "Inside get_angle "; cout <<angle << endl;
            double return_ang = std::max(min_angle_, std::min(max_angle_, angle));
            // cout << "Return_angle "; cout <<return_ang << endl;

            return return_ang;
        }

        double get_steering(double linear_velocity, double angular_velocity, double current_velocity);

        void set_yawcontroller(double steer_ratio)
        {
            steer_ratio_ = steer_ratio;
        }
};

class VelController
{
    private:
        double kp; double kd; double ki; double max = 10000.0; double min = -10000.0;
        double int_val = 0.0; 
        double last_int_val = 0.0;
        double last_error = 0.0; 

    public:
        
        double step(double error, double sample_time);
        void reset();
        void set_velcontroller(double kp_data, double ki_data, double kd_data)
        {
            kp = kp_data; ki = ki_data; kd = kd_data;
        }
};
class Controller
{
    private:
        double acceleration_limit = 5.0; 
        double deceleration_limit = -5.0;
        double wheel_base = 2.7; 
        double wheel_radius = 0.38; 
        // double steer_ratio = 14.8; 
        double max_lat_accel = 3.0; 
        double max_steer_angle = 8.0; 
        double min_speed = 0.05;
        YawController yaw_controller;
        VelController vel_controller;

        double acceleration;
        double steer; 

    public:
        void control(geometry_msgs::TwistStamped twist_cmd, geometry_msgs::TwistStamped current_velocity, double time_span );
        void reset();

        void set_controller(double kp, double ki, double kd, double steer_ratio)
        {
            vel_controller.set_velcontroller(kp, ki, kd);
            yaw_controller.set_yawcontroller(steer_ratio);
        }

        double get_acceleration(){
            return acceleration;
        }
        double get_steer(){ return steer;}
    
};