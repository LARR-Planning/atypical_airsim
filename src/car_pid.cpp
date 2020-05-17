#include <car_pid.h>
#define MIN_VEL 0.1

using namespace std;

double YawController::get_steering(double linear_velocity, double angular_velocity, double cur_twist)
{
    if(std::abs(linear_velocity ) > 0)
        angular_velocity = cur_twist * angular_velocity / linear_velocity;
    else
    {
        angular_velocity = 0;
    }

    if(std::abs(cur_twist) > 0.1)
    {
        double max_yaw_rate = std::abs(max_lat_accel_ / cur_twist);
        angular_velocity = std::max(-max_yaw_rate, std::min(max_yaw_rate, angular_velocity));
    }


    if(abs(angular_velocity) >0.01)
    {
        // ROS_DEBUG("Idon;tkndw  %d", std::max(cur_twist, min_speed_) / angular_velocity);
    
        double ang = get_angle(std::max(cur_twist, min_speed_) / angular_velocity);
        return ang;
    }
    else
    {   
            return 0.0;
    } 
}



void Controller::control(geometry_msgs::TwistStamped twist_des, geometry_msgs::TwistStamped cur_twist, double time_span )
{
    // double cur_speed;
    // if(twist_des.twist.linear.x <0)
    // {
    //     cur_speed = std::sqrt((cur_twist.twist.linear.x * cur_twist.twist.linear.x) + cur_twist.twist.linear.y * cur_twist.twist.linear.y);
    // }
    // else
    // {
    //     cur_speed = -std::sqrt((cur_twist.twist.linear.x * cur_twist.twist.linear.x) + cur_twist.twist.linear.y * cur_twist.twist.linear.y);        
    // }
    // cout << cur_speed << endl;

    double tmp_velocity_error = twist_des.twist.linear.x - cur_twist.twist.linear.x;
    // double tmp_velocity_error = twist_des.twist.linear.x - cur_speed;

    double tmp_acceleration = vel_controller.step(tmp_velocity_error, time_span); // Get velocity correction from PID controller
    cout << "des_vel ";
    cout << twist_des.twist.linear.x << endl;
    cout << "vel_error";
    cout << tmp_velocity_error << endl;
    double tmp_steer = yaw_controller.get_steering(twist_des.twist.linear.x, twist_des.twist.angular.z, cur_twist.twist.linear.x); //Calculate steering angle 

    // if( cur_twist.twist.linear.x < MIN_VEL)
    // {
    //     acceleration = 0.0;
    //     // torque = calc_torque(deceleration_limit);
    //     steer = tmp_steer;
    // }
    // else{
        // if(abs(tmp_acceleration) > 0.0)
        {   acceleration = tmp_acceleration;
            steer = tmp_steer;
        // }
        // else{
        //     // torque = calc_torque(-acceleration);
        //     acceleration = 0.0;
        //     steer = tmp_steer;
        //     cout << "Stuck in here1" << endl;
        }
    // }
}


void Controller::reset()
{
    vel_controller.reset();
}


void VelController::reset()
{
    int_val = 0.0;
    last_int_val = 0.0;
}

double VelController::step(double error, double sample_time)
{
    last_int_val = int_val;
    double integral = int_val + error * sample_time;
    double derivative = (error - last_error ) / sample_time;

    double y = kp*error + ki*integral + kd*derivative;
    double val = std::max(min, std::min(y, max));

    //cout << "integral" << endl;
    //cout << integral << endl;
    //cout << "der" << endl;
    //cout << derivative << endl;
   // cout << "pro" << endl;
    //cout << error << endl;

    if(val > max)
        val = max;
    else if(val < min)
        val = min;
    else
    {
        int_val = integral;
    }
    last_error = error;

    return val;
    
}
