#include <pid_wrapper.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3Stamped.h>

using namespace std;

pid_wrapper::pid_wrapper(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private)
{
    Controller pid_control;
    pid_control;
    
    //load parameter
    bool b1 =  nh_.getParam("/pid_node/kp", kp);
    b1 = nh_.getParam("/pid_node/ki", ki);
    b1 = nh_.getParam("/pid_node/kd", kd);
    b1 = nh_.getParam("/pid_node/steer_ratio", steer_ratio);
    b1 = nh_.getParam("/pid_node/update_period", update_period);
    

    sub_des_acc = nh_.subscribe("/acc_cmd_body_frame", 40, &pid_wrapper::acc_cmd_cb, this);
    sub_odom = nh_.subscribe("/airsim_car_node/PhysXCar/odom_local_ned", 40, &pid_wrapper::odom_cb, this);
    // sub_vel = nh_.subscribe("/airsim_car_node/PhysXCar/twist_local_ned", 100, &pid_wrapper::twist_cb, this);
    pid_controller.set_controller(kp, ki, kd, steer_ratio);


    pub_acc_cmd = nh_.advertise<driving_msgs::VehicleCmd>("/airsim_car_node/PhysXCar/vehicle_cmd", 40);
    pub_vis_path = nh_.advertise<nav_msgs::Path>("desired_path", 40);

    last_time = ros::Time::now().toSec();

    pid_wrapper::loop();
}

void pid_wrapper::loop()
{   check_point = ros::Time::now().toSec();

    ros::Rate loop_rate(40);
    pid_controller.reset();

    while(ros::ok())
    {   
        // cout << cur_twist.twist.linear.x << endl;


        double cur_time = ros::Time::now().toSec();
        double time_step = (double)(cur_time - last_time);
        last_time = cur_time;

        //Transform from "PhysXCar" frame(current twist frame) to "PhysXCar/odom_local_ned" frame
        vec.header = cur_twist.header;
        vec.vector.x = cur_twist.twist.linear.x;
        vec.vector.y = cur_twist.twist.linear.y;
        vec.vector.z = cur_twist.twist.linear.z;
        if (isCallbackReceived)
        listener.transformVector(cur_odom.child_frame_id, vec, vec2);
        //cout << cur_odom.child_frame_id << endl;
        //cout << cur_odom.header.frame_id << endl;
        cur_twist.twist.linear.x = vec2.vector.x;
        cur_twist.twist.linear.y = vec2.vector.y;
        cur_twist.twist.linear.z = vec2.vector.z;


        bool can_control = true;
        if(can_control)
        {
            //Update position to current odom in given period
            bool update = cur_time - check_point > update_period;
            //Generate reference trajectory 
            des_path_gen(time_step, update);

            geometry_msgs::TwistStamped twist_des;
            twist_des.twist.linear.x = v;
            twist_des.twist.angular.z = w;

            //cout << "twist_des.twist.linear.x "; cout << twist_des.twist.linear.x <<endl;
                pid_controller.control(twist_des, cur_twist, time_step);

            ROS_INFO( "[PID loop] cur_twist [x,y]=[%f,%f]",cur_twist.twist.linear.x,cur_twist.twist.linear.y);
            //cout << "cur_twist y ";
            //cout << cur_twist.twist.linear.y << endl;

            double acceleration = pid_controller.get_acceleration();
            // double steer = pid_controller.get_steer();

            
            //Should Translate steering angle to angular velocity (Airsim is little bit strange...)
            //Airsim Steering input == Airsim Car's angular velocity....

            double real_s = -prev_acc_cmd.twist.angular.z;
            double linear_vel = cur_twist.twist.linear.x;
            double ang_vel = linear_vel * tan(real_s) / L;
            double steer = ang_vel;

            //cout << "Acceleration";
            //cout << acceleration << endl;
            // cout << "Steeer";
            // cout << steer << endl;

            geometry_msgs::TwistStamped acc_publish;
            acc_publish.twist.angular.z = -steer;
            acc_publish.twist.linear.x = acceleration;
            acc_publish.header.frame_id = "/PhysXCar/odom_local_ned";
            acc_publish.header.stamp = ros::Time::now();
            
            driving_msgs::VehicleCmd cmd = convert_twst_to_vehicle_cmd(acc_publish.twist);
            cmd.steer_angle_cmd*=180/M_PI;
            pub_acc_cmd.publish(cmd);

        }
        isCallbackReceived = false;
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void pid_wrapper::des_path_gen(double time_step, bool update)
{   

    geometry_msgs::PoseStamped des_pose;
    
    // des_pose.header.frame_id = "/PhysXCar/odom_local_ned";
    des_pose.header.frame_id = "/PhysXCar/odom_local_ned";
    des_pose.header.stamp = ros::Time::now();
        
    des_pose.pose = pid_wrapper::dynamics(prev_pose, prev_acc_cmd, time_step);

    des_path.header.frame_id = "/PhysXCar/odom_local_ned";
    des_path.header.stamp = ros::Time::now();
    des_path.poses.push_back(des_pose);
    
    //Receding Horizon Update (Prevent the far-away drift of desired path)
    if(update)
    {   
        des_path.poses.clear();
        check_point = ros::Time::now().toSec();

        //Update to current body frame odom value
        prev_pose.orientation.w = 0.0;
        prev_pose.orientation.x = 0.0;
        prev_pose.orientation.y = 0.0;
        prev_pose.orientation.z = 0.0;
        
        prev_pose.position.x = 0.0;
        prev_pose.position.y = 0.0;
        prev_pose.position.z = 0.0;
        
        v = cur_twist.twist.linear.x; //Body frame velocity update
        w = -cur_twist.twist.angular.z;
        pid_controller.reset();
    }   
    else
    {
        prev_pose = des_pose.pose;
    }
    pub_vis_path.publish(des_path);
}

geometry_msgs::Pose pid_wrapper::dynamics(geometry_msgs::Pose prev_pose, geometry_msgs::TwistStamped input, double time_step)
{   
    //Input
    double accel = input.twist.linear.x;
    double steer = -input.twist.angular.z;
    
    Quaternion quat;  
    quat.w = prev_pose.orientation.w;
    quat.x = prev_pose.orientation.x;
    quat.y = prev_pose.orientation.y;
    quat.z = prev_pose.orientation.z;
    
    EulerAngles euler = ToEulerAngles(quat);
    double yaw = euler.yaw;

    // double v = 0.0; double w = 0.0;

    //cout << "accel " << endl; cout << accel << endl;
    v = v + accel * time_step;
    w = v * tan(steer) / L;
    //cout << "v in dynamics " << endl; cout << v << endl;

    geometry_msgs::Pose cur_pose;
    cur_pose.position.x = prev_pose.position.x + cos(yaw) * v * time_step;
    cur_pose.position.y = prev_pose.position.y + sin(yaw) * v * time_step;
    cur_pose.position.z = 0.0;

    Quaternion quat2 = ToQuaternion(yaw + w * time_step, 0.0, 0.0);
    cur_pose.orientation.w = quat2.w;
    cur_pose.orientation.x = quat2.x;
    cur_pose.orientation.y = quat2.y;
    cur_pose.orientation.z = quat2.z;
    
    return cur_pose;
}


void pid_wrapper::acc_cmd_cb(const geometry_msgs::TwistStamped& msg)
{
    prev_acc_cmd.header.frame_id = msg.header.frame_id;
    prev_acc_cmd.header.stamp = msg.header.stamp;
    prev_acc_cmd.header.seq = msg.header.seq;
    
    prev_acc_cmd.twist.linear.x = msg.twist.linear.x;
    prev_acc_cmd.twist.linear.y = msg.twist.linear.y;
    prev_acc_cmd.twist.linear.z = msg.twist.linear.z;
    
    prev_acc_cmd.twist.angular.x = msg.twist.angular.x;
    prev_acc_cmd.twist.angular.y = msg.twist.angular.y;
    prev_acc_cmd.twist.angular.z = msg.twist.angular.z;
        
}

void pid_wrapper::odom_cb(const nav_msgs::Odometry &msg)
{
    cur_odom.header = msg.header;
    cur_odom.child_frame_id = msg.child_frame_id;
    cur_odom.pose.pose.orientation.w = msg.pose.pose.orientation.w;
    cur_odom.pose.pose.orientation.x = msg.pose.pose.orientation.x;
    cur_odom.pose.pose.orientation.y = msg.pose.pose.orientation.y;
    cur_odom.pose.pose.orientation.z = msg.pose.pose.orientation.z;

    cur_odom.pose.pose.position.x = msg.pose.pose.position.x;
    cur_odom.pose.pose.position.y = msg.pose.pose.position.y;
    cur_odom.pose.pose.position.z = msg.pose.pose.position.z;

    // cur_twist.twist = msg.twist.twist;
    cur_twist.twist.angular.x = msg.twist.twist.angular.x;
    cur_twist.twist.angular.y = msg.twist.twist.angular.y;
    cur_twist.twist.angular.z = msg.twist.twist.angular.z;        

    cur_twist.twist.linear.x = msg.twist.twist.linear.x;
    cur_twist.twist.linear.y = msg.twist.twist.linear.y;
    cur_twist.twist.linear.z = msg.twist.twist.linear.z;
    
    cur_twist.header.frame_id = msg.header.frame_id;
    cur_twist.header.stamp = msg.header.stamp;
    cur_twist.header.seq = msg.header.seq;
    
    ROS_INFO( "[PID callback] cur_twist [x,y] = [%f,%f]",cur_twist.twist.linear.x,cur_twist.twist.linear.y);

    // cout << "!!!!msg twlist ";
    // cout << msg.twist.twist.linear.y << endl;
    isCallbackReceived = true;
   
}

void pid_wrapper::twist_cb(const geometry_msgs::TwistStamped &msg)
{
    cur_twist.header = msg.header;
    cur_twist.twist.angular.x = msg.twist.angular.x;
    cur_twist.twist.angular.y = msg.twist.angular.y;
    cur_twist.twist.angular.z = msg.twist.angular.z;
    
    cur_twist.twist.linear.x = msg.twist.linear.x;
    cur_twist.twist.linear.y = msg.twist.linear.y;
    cur_twist.twist.linear.z = msg.twist.linear.z;
    
}



EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

// void Transformer::transformVector(const std::string& target_frame,
//                                    const Stamped<tf::Vector3>& stamped_in,
//                                    Stamped<tf::Vector3>& stamped_out) const
// {
//    StampedTransform transform;
//    lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);
 
//    tf::Vector3 end = stamped_in;
//    tf::Vector3 origin = tf::Vector3(0,0,0);
//    tf::Vector3 output = (transform * end) - (transform * origin);
//    stamped_out.setData( output);
 
//    stamped_out.stamp_ = transform.stamp_;
//    stamped_out.frame_id_ = target_frame;
//  };
