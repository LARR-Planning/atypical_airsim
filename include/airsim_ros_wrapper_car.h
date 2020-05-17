#include "common/AirSimSettings.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "ros/ros.h"
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "sensors/lidar/LidarBase.hpp"
#include <chrono>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include "airsim_settings_parser.h"
#include <atypical_ros/Reset.h>
#include <atypical_ros/VelCmd.h>
#include <math_common.h>
#include <mavros_msgs/State.h>
#include <std_srvs/Empty.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <atypical_ros/GPSYaw.h>
#include <driving_msgs/VehicleCmd.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <octomap/OcTree.h>

typedef msr::airlib::AirSimSettings::VehicleSetting VehicleSetting;
typedef msr::airlib::AirSimSettings::LidarSetting LidarSetting;

struct VelCmd
{
    double x;
    double y;
    double z;
    msr::airlib::DrivetrainType drivetrain;
    msr::airlib::YawMode yaw_mode;
    std::string vehicle_name;

    // VelCmd() : 
    //     x(0), y(0), z(0), 
    //     vehicle_name("") {drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    //             yaw_mode = msr::airlib::YawMode();};

    // VelCmd(const double& x, const double& y, const double& z, 
    //         msr::airlib::DrivetrainType drivetrain, 
    //         const msr::airlib::YawMode& yaw_mode,
    //         const std::string& vehicle_name) : 
    //     x(x), y(y), z(z), 
    //     drivetrain(drivetrain), 
    //     yaw_mode(yaw_mode), 
    //     vehicle_name(vehicle_name) {};
};

class AirsimCar_ROSWrapper
{
public:
    AirsimCar_ROSWrapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    ~AirsimCar_ROSWrapper() {}; 

    ros::AsyncSpinner lidar_async_spinner_;
    ros::AsyncSpinner object_async_spinner_; // JBS

    bool is_used_lidar_timer_cb_queue_;
    bool is_used_object_timer_cb_queue_; // JBS

    void initialize_airsim();
    void initialize_ros();

private:

    msr::airlib::CarRpcLibClient airsim_client_;
    msr::airlib::CarRpcLibClient airsim_client_lidar_;
    msr::airlib::CarRpcLibClient airsim_client_object_; // JBS

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    string object_name; // JBS
    geometry_msgs::PoseStamped object_pose_enu;
    geometry_msgs::PoseStamped ned_pose_to_enu_pose(const geometry_msgs::PoseStamped & pose_ned);
    ros::Publisher pose_object_enu_pub;


    //Ros Publisher
    ros::Publisher pub_tf_odom;
    std::vector<ros::Publisher> pub_PointCloud2_lidar; //goes to octomap_server
    ros::Publisher pub_carstate_state; //publish state values of car
    ros::Publisher origin_geo_point_pub_;
    //TODO: How to publish dynamic obstacle????? 
    //std::vector<ros::Publisher> pub_posewithcovariance_dyn_obst;




    /// ROS subscriber callbacks
    void acc_cmd_body_frame_cb(const geometry_msgs::TwistStamped::ConstPtr& msg, const std::string& vehicle_name);
    void acc_cmd_world_frame_cb(const geometry_msgs::TwistStamped::ConstPtr& msg, const std::string& vehicle_name);

    void vehicle_cmd_cb(const driving_msgs::VehicleCmd::ConstPtr& msg, const std::string& vehicle_name);

    // void acc_cmd_world_frame_cb(const airsim_ros_pkgs::VelCmd::ConstPtr& msg, const std::string& vehicle_name);
    // void acc_cmd_body_frame_cb(const airsim_ros_pkgs::VelCmd::ConstPtr& msg, const std::string& vehicle_name);

    void control_car(CarRpcLibClient& client, geometry_msgs::TwistStamped cmd_vel);

    //visualization
    // ros::Publisher pub_vis_tf_car;
    std::vector<ros::Publisher> pub_tf_object; //tf of dynamic obstacles

    ros::CallbackQueue lidar_timer_cb_queue_;
    ros::CallbackQueue object_timer_cb_queue_;

    struct Car_ROS
    {   
        Car_ROS()
        {

        }
        std::string vehicle_name_;
        std::string odom_frame_id_;

        //State
        msr::airlib::CarApiBase::CarState curr_car_state;

        // All things ROS
        ros::Publisher pub_odom_local_ned;
        ros::Publisher pub_global_gps;
        ros::Publisher pub_twist_local_ned;
        
        ros::Subscriber sub_acc_cmd_body_frame;
        ros::Subscriber sub_acc_cmd_world_frame;
        ros::Subscriber sub_vehicle_cmd;

        //Estimation
        nav_msgs::Odometry cur_odom_ned;
        geometry_msgs::TwistStamped cur_twist;
        
        sensor_msgs::NavSatFix gps_sensor_msg;
                
        bool has_acc_cmd;
        //Change velocity command type as TwistStamped
        geometry_msgs::TwistStamped acc_cmd;
        // VelCmd acc_cmd;

    };   

    std::unordered_map<std::string, int> vehicle_name_idx_map_;

    // ROS timer
    ros::Timer airsim_control_update_timer_;
    ros::Timer airsim_lidar_update_timer_;
    ros::Timer airsim_object_update_timer_; // JBS

    void publish_odom_tf(const nav_msgs::Odometry& odom_ned_msg);

    /// ROS timer callbacks
    void car_state_timer_cb(const ros::TimerEvent& event); // update car state from airsim_client_ every nth sec
    void lidar_timer_cb(const ros::TimerEvent& event);
    void objects_timer_cb(const ros::TimerEvent& event); // JBS update the pose of object

    std::recursive_mutex car_control_mutex_;

    ros::ServiceServer reset_srvr_;
    bool reset_srv_cb(atypical_ros::Reset::Request& request, atypical_ros::Reset::Response& response);


    /// ROS subscriber callbacks
    void acc_cmd_world_cb(const geometry_msgs::Twist::ConstPtr& msg, const std::string& vehicle_name);
    void acc_cmd_body_cb(const geometry_msgs::Twist::ConstPtr& msg, const std::string& vehicle_name);

    // void steer_quat_cmd_cb(const airsim_ros_pkgs::GimbalAngleQuatCmd& gimbal_angle_quat_cmd_msg);
    // void steer_euler_cmd_cb(const airsim_ros_pkgs::GimbalAngleEulerCmd& gimbal_angle_euler_cmd_msg);

    std::vector<Car_ROS> car_ros_vec_;

    /// ROS tf broadcasters
    void pub_tf_odometry(const nav_msgs::Odometry& odom_ned_msg);

    std::vector<geometry_msgs::TransformStamped> static_tf_msg_vec_;
    std::vector<ros::Publisher> lidar_pub_vec_;
    std::vector<ros::Publisher> imu_pub_vec_;

    std::map<std::string, std::string> vehicle_imu_map_;
    std::map<std::string, std::string> vehicle_lidar_map_;

    ///ROS param
    bool is_vulkan_; 
    std::vector<string> vehicle_names_;
    AirSimSettingsParser airsim_settings_parser_;
    static const std::unordered_map<int, std::string> image_type_int_to_string_map_;

    // parse setting json ang generate ros pubsubsrv
    void create_ros_pubs_from_settings_json();
    void append_static_lidar_tf(const std::string& vehicle_name, const std::string& lidar_name, const LidarSetting& lidar_setting);
    void append_static_vehicle_tf(const std::string& vehicle_name, const VehicleSetting& vehicle_setting);
    void set_nans_to_zeros_in_pose(VehicleSetting& vehicle_setting) const;
    // void set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, CameraSetting& camera_setting) const;
    void set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, LidarSetting& lidar_setting) const;

    /// utils
    tf2::Quaternion get_tf2_quat(const msr::airlib::Quaternionr& airlib_quat) const;
    msr::airlib::Quaternionr get_airlib_quat(const geometry_msgs::Quaternion& geometry_msgs_quat) const;
    msr::airlib::Quaternionr get_airlib_quat(const tf2::Quaternion& tf2_quat) const;
    nav_msgs::Odometry get_odom_msg_from_airsim_state(const msr::airlib::CarApiBase::CarState& car_state) const;
    sensor_msgs::PointCloud2 get_lidar_msg_from_airsim(const msr::airlib::LidarData& lidar_data) const;
    sensor_msgs::Imu get_imu_msg_from_airsim(const msr::airlib::ImuBase::Output& imu_data) const;
    atypical_ros::GPSYaw get_gps_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const;
    sensor_msgs::NavSatFix get_gps_sensor_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const;
    geometry_msgs::Twist get_twist_msg_from_airsim_state(const msr::airlib::CarApiBase::CarState& car_state) const;

    msr::airlib::GeoPoint origin_geo_point_;// gps coord of unreal origin 
    atypical_ros::GPSYaw origin_geo_point_msg_; // duplicate

    /// ROS tf
    std::string world_frame_id_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_pub_;
    tf2_ros::Buffer tf_buffer_;

    /// ROS params
    double acc_cmd_duration_;


    void exec(); //terminate processes

};
