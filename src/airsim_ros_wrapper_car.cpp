#include <airsim_ros_wrapper_car.h>
#include <boost/make_shared.hpp>
// #include <pluginlib/class_list_macros.h>
// PLUGINLIB_EXPORT_CLASS(AirsimCar_ROSWrapper, nodelet::Nodelet)
#include "common/AirSimSettings.hpp"
#include "ros/console.h"

#define PI 3.141592
// using namespace std;
// constexpr char AirsimCar_ROSWrapper::CAM_YML_NAME[];
// constexpr char AirsimCar_ROSWrapper::WIDTH_YML_NAME[];
// constexpr char AirsimCar_ROSWrapper::HEIGHT_YML_NAME[];
// constexpr char AirsimCar_ROSWrapper::K_YML_NAME[];
// constexpr char AirsimCar_ROSWrapper::D_YML_NAME[];
// constexpr char AirsimCar_ROSWrapper::R_YML_NAME[];
// constexpr char AirsimCar_ROSWrapper::P_YML_NAME[];
// constexpr char AirsimCar_ROSWrapper::DMODEL_YML_NAME[];

const std::unordered_map<int, std::string> AirsimCar_ROSWrapper::image_type_int_to_string_map_ = {
    { 0, "Scene" },
    { 1, "DepthPlanner" },
    { 2, "DepthPerspective" },
    { 3, "DepthVis" },
    { 4, "DisparityNormalized" },
    { 5, "Segmentation" },
    { 6, "SurfaceNormals" },
    { 7, "Infrared" }
};

AirsimCar_ROSWrapper::AirsimCar_ROSWrapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) : 
    nh_(nh), 
    nh_private_(nh_private),
    // object_async_spinner_(1, &object_timer_cb_queue_), // a thread for image callbacks to be 'spun' by img_async_spinner_ 
    lidar_async_spinner_(1, &lidar_timer_cb_queue_) // same as above, but for lidar
{
    is_used_lidar_timer_cb_queue_ = false;
    // is_used_img_timer_cb_queue_ = false;

    world_frame_id_ = "world_ned"; // todo rosparam?

    initialize_ros();


    std::cout << "AirsimCar_ROSWrapper Initialized!\n";
    // intitialize placeholder control commands
    // acc_cmd_ = VelCmd();
    // gimbal_cmd_ = GimbalCmd();

    initialize_airsim();



}

void AirsimCar_ROSWrapper::initialize_airsim()
{
    // todo do not reset if already in air?
    try
    {
        airsim_client_.confirmConnection();
        // airsim_client_images_.confirmConnection();
        airsim_client_lidar_.confirmConnection();
        // airsim_client_object_.confirmConnection(); //JBS 
        for (const auto& vehicle_name : vehicle_names_)
        {
            airsim_client_.enableApiControl(true, vehicle_name); // todo expose as rosservice?
            airsim_client_.armDisarm(true, vehicle_name); // todo exposes as rosservice?
        }

        origin_geo_point_ = airsim_client_.getHomeGeoPoint("");
        // todo there's only one global origin geopoint for environment. but airsim API accept a parameter vehicle_name? inside carsimpawnapi.cpp, there's a geopoint being assigned in the constructor. by? 
        origin_geo_point_msg_ = get_gps_msg_from_airsim_geo_point(origin_geo_point_);
        // std::cout << origin_geo_point_msg_.altitude << std::endl;
    }
    catch (rpc::rpc_error&  e)
    {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
    }
}

void AirsimCar_ROSWrapper::initialize_ros()
{

    // ros params
    double update_airsim_control_every_n_sec;
    nh_private_.getParam("is_vulkan", is_vulkan_);
    nh_private_.getParam("update_airsim_control_every_n_sec", update_airsim_control_every_n_sec);
    nh_private_.getParam("object_name",object_name);
    acc_cmd_duration_ = 0.05; // todo rosparam
    // todo enforce dynamics constraints in this node as well?
    // nh_.getParam("max_vert_vel_", max_vert_vel_);
    // nh_.getParam("max_horz_vel", max_horz_vel_)

    create_ros_pubs_from_settings_json();
    airsim_control_update_timer_ = nh_private_.createTimer(ros::Duration(update_airsim_control_every_n_sec), &AirsimCar_ROSWrapper::car_state_timer_cb, this);

}


// XmlRpc::XmlRpcValue can't be const in this case
void AirsimCar_ROSWrapper::create_ros_pubs_from_settings_json()
{
    // subscribe to control commands on global nodehandle
    // gimbal_angle_quat_cmd_sub_ = nh_private_.subscribe("gimbal_angle_quat_cmd", 50, &AirsimCar_ROSWrapper::gimbal_angle_quat_cmd_cb, this);
    // gimbal_angle_euler_cmd_sub_ = nh_private_.subscribe("gimbal_angle_euler_cmd", 50, &AirsimCar_ROSWrapper::gimbal_angle_euler_cmd_cb, this);
    origin_geo_point_pub_ = nh_private_.advertise<atypical_ros::GPSYaw>("origin_geo_point", 10);       

    // airsim_img_request_vehicle_name_pair_vec_.clear();
    // image_pub_vec_.clear();
    // cam_info_pub_vec_.clear();s
    // camera_info_msg_vec_.clear();
    static_tf_msg_vec_.clear();
    imu_pub_vec_.clear();
    lidar_pub_vec_.clear();
    vehicle_names_.clear(); // todo should eventually support different types of vehicles in a single instance
    // vehicle_setting_vec_.clear();
    // vehicle_imu_map_;
    car_ros_vec_.clear();
    // callback_queues_.clear();

    // image_transport::ImageTransport image_transporter(nh_private_);

    int idx = 0;
    // iterate over std::map<std::string, std::unique_ptr<VehicleSetting>> vehicles;
    for (const auto& curr_vehicle_elem : AirSimSettings::singleton().vehicles)
    {
        auto& vehicle_setting = curr_vehicle_elem.second;
        auto curr_vehicle_name = curr_vehicle_elem.first;
        vehicle_names_.push_back(curr_vehicle_name);
        set_nans_to_zeros_in_pose(*vehicle_setting);
        // auto vehicle_setting_local = vehicle_setting.get();

        // JBS
        pose_object_enu_pub = nh_private_.advertise<geometry_msgs::PoseStamped>("object_pose",10);



        append_static_vehicle_tf(curr_vehicle_name, *vehicle_setting);
        // vehicle_name_idx_map_[curr_vehicle_name] = idx; // allows fast lookup in command callbacks in case of a lot of drones  

        Car_ROS car_ros;
        car_ros.odom_frame_id_ = curr_vehicle_name + "/odom_local_ned";
        car_ros.vehicle_name_ = curr_vehicle_name;
        car_ros.pub_odom_local_ned = nh_private_.advertise<nav_msgs::Odometry>(curr_vehicle_name + "/odom_local_ned", 50);
        // car_ros.pub_global_gps = nh_private_.advertise<sensor_msgs::NavSatFix>(curr_vehicle_name + "/global_gps", 10);
        car_ros.pub_twist_local_ned = nh_private_.advertise<geometry_msgs::TwistStamped>(curr_vehicle_name + "/twist_local_ned", 50);

        // bind to a single callback. todo optimal subs queue length
        // bind multiple topics to a single callback, but keep track of which vehicle name it was by passing curr_vehicle_name as the 2nd argument 
        car_ros.sub_acc_cmd_body_frame = nh_private_.subscribe<geometry_msgs::TwistStamped>(curr_vehicle_name + "/acc_cmd_body_frame", 1, 
            boost::bind(&AirsimCar_ROSWrapper::acc_cmd_body_frame_cb, this, _1, car_ros.vehicle_name_)); // todo ros::TransportHints().tcpNoDelay();
        car_ros.sub_acc_cmd_world_frame = nh_private_.subscribe<geometry_msgs::TwistStamped>(curr_vehicle_name + "/acc_cmd_world_frame", 1, 
            boost::bind(&AirsimCar_ROSWrapper::acc_cmd_world_frame_cb, this, _1, car_ros.vehicle_name_));

        car_ros.sub_vehicle_cmd = nh_private_.subscribe<driving_msgs::VehicleCmd>(curr_vehicle_name + "/vehicle_cmd", 1,
            boost::bind(&AirsimCar_ROSWrapper::vehicle_cmd_cb, this, _1, car_ros.vehicle_name_));


        car_ros_vec_.push_back(car_ros);
        idx++;

        // iterate over sensors std::map<std::string, std::unique_ptr<SensorSetting>> sensors;
        for (auto& curr_sensor_map : vehicle_setting->sensors)
        {
            auto& sensor_name = curr_sensor_map.first;
            auto& sensor_setting = curr_sensor_map.second;

            switch (sensor_setting->sensor_type)
            {
                case SensorBase::SensorType::Barometer:
                {
                    std::cout << "Barometer" << std::endl; 
                    break;
                }
                case SensorBase::SensorType::Imu:
                {
                    vehicle_imu_map_[curr_vehicle_name] = sensor_name; 
                    // todo this is pretty non scalable, refactor airsim and ros api and maintain a vehicle <-> sensor (setting) map
                    std::cout << "Imu" << std::endl;
                    imu_pub_vec_.push_back(nh_private_.advertise<sensor_msgs::Imu> (curr_vehicle_name + "/imu/" + sensor_name, 10));
                    break;
                }
                case SensorBase::SensorType::Gps:
                {
                    std::cout << "Gps" << std::endl; 
                    break;
                }
                case SensorBase::SensorType::Magnetometer:
                {
                    std::cout << "Magnetometer" << std::endl; 
                    break;
                }
                case SensorBase::SensorType::Distance:
                {
                    std::cout << "Distance" << std::endl; 
                    break;
                }
                case SensorBase::SensorType::Lidar:
                {
                    std::cout << "Lidar" << std::endl;
                    auto lidar_setting = *static_cast<LidarSetting*>(sensor_setting.get());
                    set_nans_to_zeros_in_pose(*vehicle_setting, lidar_setting);
                    append_static_lidar_tf(curr_vehicle_name, sensor_name, lidar_setting); // todo is there a more readable way to down-cast?
                    vehicle_lidar_map_[curr_vehicle_name] = sensor_name; // non scalable 
                    lidar_pub_vec_.push_back(nh_private_.advertise<sensor_msgs::PointCloud2> (curr_vehicle_name + "/lidar/" + sensor_name, 10));
                    break;
                }
                default:
                {
                    throw std::invalid_argument("Unexpected sensor type");
                }
            }
        }
    }


    // todo add per vehicle reset in AirLib API
    reset_srvr_ = nh_private_.advertiseService("reset",&AirsimCar_ROSWrapper::reset_srv_cb, this);

    // todo mimic gazebo's /use_sim_time feature which publishes airsim's clock time..via an rpc call?!
    // clock_pub_ = nh_private_.advertise<rosgraph_msgs::Clock>("clock", 10); 

    if (lidar_pub_vec_.size() > 0)
    {    
        double update_lidar_every_n_sec;
        nh_private_.getParam("update_lidar_every_n_sec", update_lidar_every_n_sec);
        nh_private_.setCallbackQueue(&lidar_timer_cb_queue_);
        // nh_private_.setCallbackQueue(&object_timer_cb_queue_); //JBS 
        bool separate_spinner = true; // todo debugging race condition
        if(separate_spinner)
        {
            ros::TimerOptions timer_options(ros::Duration(update_lidar_every_n_sec), boost::bind(&AirsimCar_ROSWrapper::lidar_timer_cb, this, _1), &lidar_timer_cb_queue_);
            airsim_lidar_update_timer_ = nh_private_.createTimer(timer_options);            
            is_used_lidar_timer_cb_queue_ = true;
            
            // JBS
            //ros::TimerOptions timer_options1(ros::Duration(update_lidar_every_n_sec), boost::bind(&AirsimCar_ROSWrapper::objects_timer_cb, this, _1), &object_timer_cb_queue_);
            //airsim_object_update_timer_ = nh_private_.createTimer(timer_options1);



        }
        else
        {
            airsim_lidar_update_timer_ = nh_private_.createTimer(ros::Duration(update_lidar_every_n_sec), &AirsimCar_ROSWrapper::lidar_timer_cb, this);
        }
    }












    // initialize_airsim();
}

/**
 * @brief control car with given action cmd (twist)
 * 
 * @param client 
 * @param cmd_vel 
 */
void AirsimCar_ROSWrapper::control_car(CarRpcLibClient& client, geometry_msgs::TwistStamped cmd_vel){
    
    CarApiBase::CarControls controls;
    controls.steering = -cmd_vel.twist.angular.z;
    controls.throttle = cmd_vel.twist.linear.x;
    // if(cmd_vel.twist.linear.x < 0)
    // {
    //     controls.brake = -1.0*controls.throttle;
    //     controls.throttle = 0.0;
    // }
    // else{
    //     controls.brake = 0.0;
    // }

    // ROS_INFO("Throttle: %f", cmd_vel.twist.linear.x);
    // ROS_INFO("%f", controls.steering);
    // ROS_INFO("%f", controls.throttle);
    if(cmd_vel.twist.linear.x > 0)
    {
        controls.is_manual_gear = true;
        controls.manual_gear = 3;
    }
    else
    {
        controls.is_manual_gear = true;
        controls.manual_gear = -1;
    }
    client.setCarControls(controls);

}

geometry_msgs::PoseStamped AirsimCar_ROSWrapper::ned_pose_to_enu_pose(const geometry_msgs::PoseStamped & pose_ned){

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = pose_ned.header.frame_id;
    // odom_ned_msg.header.frame_id = world_frame_id_;
    // odom_ned_msg.child_frame_id = "/airsim/odom_local_ned"; // todo make param
    Eigen::Quaternionf quat;
    Eigen::Vector3f transl;
    transl(0) = pose_ned.pose.position.x;
    transl(1) = pose_ned.pose.position.y;
    transl(2) = pose_ned.pose.position.z;
    quat.x() =  pose_ned.pose.orientation.x;
    quat.y() =  pose_ned.pose.orientation.y;
    quat.z() =  pose_ned.pose.orientation.z;
    quat.w() =  pose_ned.pose.orientation.w;

    Eigen::Matrix3f rotm_en;
    Eigen::Affine3f transform;
    rotm_en << 1,0,0,
            0,-1,0,
            0,0,-1;

    quat.normalize();
    transform.setIdentity();
    transform.translate(transl);
    transform.rotate(quat);

    transform.prerotate(rotm_en);
    transform.rotate(rotm_en.transpose()); // to enu

    // std::cout << transform.matrix() << std::endl;

    quat = Eigen::Quaternionf(transform.rotation());
    pose_stamped.pose.position.x = transform.translation()(0);
    pose_stamped.pose.position.y = transform.translation()(1);
    pose_stamped.pose.position.z = transform.translation()(2);

    pose_stamped.pose.orientation.w = quat.w();
    pose_stamped.pose.orientation.x = quat.x();
    pose_stamped.pose.orientation.y = quat.y();
    pose_stamped.pose.orientation.z = quat.z();

    return pose_stamped;
}

void AirsimCar_ROSWrapper::objects_timer_cb(const ros::TimerEvent &event) {
try{
   ROS_DEBUG_STREAM("In timer cb");
   std::unique_lock<std::recursive_mutex> lck(car_control_mutex_);

   auto pose_true = airsim_client_object_.simGetObjectPose(object_name); // ned
   lck.unlock();
   // JBS

   ROS_DEBUG_STREAM("Object "<< object_name << " : " << pose_true.position.x() << " , " << pose_true.position.y()  << " , " << pose_true.position.z());
   geometry_msgs::PoseStamped object_pose_ned;
   object_pose_ned.header.frame_id = "map"; // ?
   object_pose_ned.pose.position.x = pose_true.position.x();
   object_pose_ned.pose.position.y = pose_true.position.y();
   object_pose_ned.pose.position.z = pose_true.position.z();
   object_pose_ned.pose.orientation.x = pose_true.orientation.x();
   object_pose_ned.pose.orientation.y = pose_true.orientation.y();
   object_pose_ned.pose.orientation.z = pose_true.orientation.z();
   object_pose_ned.pose.orientation.w = pose_true.orientation.w();

   if (not std::isnan(pose_true.position.x())){
       pose_object_enu_pub.publish(ned_pose_to_enu_pose(object_pose_ned));
   }
    }
        catch (rpc::rpc_error& e)
    {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, didn't get object information." << std::endl << msg << std::endl;
    }

}


void AirsimCar_ROSWrapper::car_state_timer_cb(const ros::TimerEvent& event)
{
    try
    {
        std::lock_guard<std::recursive_mutex> guard(car_control_mutex_);



        // todo this is global origin
        origin_geo_point_pub_.publish(origin_geo_point_msg_);
        // iterate over drones
        for (auto& car_ros: car_ros_vec_)
        {
            // get drone state from airsim
            std::unique_lock<std::recursive_mutex> lck(car_control_mutex_);
            car_ros.curr_car_state = airsim_client_.getCarState(car_ros.vehicle_name_);
            lck.unlock();
            ros::Time curr_ros_time = ros::Time::now();
            // convert airsim drone state to ROS msgs
            car_ros.cur_odom_ned = get_odom_msg_from_airsim_state(car_ros.curr_car_state);
            car_ros.cur_odom_ned.header.frame_id = car_ros.vehicle_name_;
            car_ros.cur_odom_ned.child_frame_id = car_ros.odom_frame_id_;
            car_ros.cur_odom_ned.header.stamp = curr_ros_time;

            // car_ros.gps_sensor_msg = get_gps_sensor_msg_from_airsim_geo_point(car_ros.curr_car_state.gps_location);
            // car_ros.gps_sensor_msg.header.stamp = curr_ros_time;
            car_ros.cur_twist.twist = get_twist_msg_from_airsim_state(car_ros.curr_car_state);
            car_ros.cur_twist.header.frame_id = car_ros.vehicle_name_;
            car_ros.cur_twist.header.stamp = curr_ros_time;

            car_ros.cur_twist.twist.linear.z = car_ros.curr_car_state.speed;

            // publish to ROS!  
            car_ros.pub_odom_local_ned.publish(car_ros.cur_odom_ned);

            publish_odom_tf(car_ros.cur_odom_ned);

            // car_ros.pub_twist_local_ned.publish(car_ros.cur_twist);


            // send control commands from the last callback to airsim
            if (car_ros.has_acc_cmd)
            {
                std::unique_lock<std::recursive_mutex> lck(car_control_mutex_);
                control_car(airsim_client_, car_ros.acc_cmd);
                // airsim_client_.moveByVelocityAsync(car_ros.acc_cmd.x, car_ros.acc_cmd.y, car_ros.acc_cmd.z, acc_cmd_duration_, 
                    // msr::airlib::DrivetrainType::MaxDegreeOfFreedom, car_ros.acc_cmd.yaw_mode, car_ros.vehicle_name_);
                lck.unlock();
            }

            // "clear" control cmds
            car_ros.has_acc_cmd = false;
        }

        // IMUS
        if (imu_pub_vec_.size() > 0)
        {
            int ctr = 0;
            for (const auto& vehicle_imu_pair: vehicle_imu_map_)
            {
                std::unique_lock<std::recursive_mutex> lck(car_control_mutex_);
                auto imu_data = airsim_client_.getImuData(vehicle_imu_pair.second, vehicle_imu_pair.first);
                lck.unlock();
                sensor_msgs::Imu imu_msg = get_imu_msg_from_airsim(imu_data);
                imu_msg.header.frame_id = vehicle_imu_pair.first;
                imu_msg.header.stamp = ros::Time::now();
                imu_pub_vec_[ctr].publish(imu_msg);
                ctr++;
            } 
        }

        if (static_tf_msg_vec_.size() > 0)
        {
            for (auto& static_tf_msg : static_tf_msg_vec_)
            {
                static_tf_msg.header.stamp = ros::Time::now();
                static_tf_pub_.sendTransform(static_tf_msg);
            }
        }

        // // todo add and expose a gimbal angular velocity to airlib
        // if (has_gimbal_cmd_)
        // {
        //     std::unique_lock<std::recursive_mutex> lck(car_control_mutex_);
        //     airsim_client_.simSetCameraOrientation(gimbal_cmd_.camera_name, gimbal_cmd_.target_quat, gimbal_cmd_.vehicle_name);
        //     lck.unlock();
        // }

        // has_gimbal_cmd_ = false;
    }

    catch (rpc::rpc_error& e)
    {
        std::cout << "error" << std::endl;
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API:" << std::endl << msg << std::endl;
    }
}



// sensor_msgs::NavSatFix AirsimCar_ROSWrapper::get_gps_sensor_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const
// {
//     sensor_msgs::NavSatFix gps_msg;
//     gps_msg.latitude = geo_point.latitude;
//     gps_msg.longitude = geo_point.longitude; 
//     gps_msg.altitude = geo_point.altitude;
//     return gps_msg;
// }


//FIXME About CarApiBase class
nav_msgs::Odometry AirsimCar_ROSWrapper::get_odom_msg_from_airsim_state(const msr::airlib::CarApiBase::CarState& car_state) const
{
    nav_msgs::Odometry odom_ned_msg;
    // odom_ned_msg.header.frame_id = world_frame_id_;
    // odom_ned_msg.child_frame_id = "/airsim/odom_local_ned"; // todo make param

    odom_ned_msg.pose.pose.position.x = car_state.kinematics_estimated.pose.position.x();
    odom_ned_msg.pose.pose.position.y = car_state.kinematics_estimated.pose.position.y();
    odom_ned_msg.pose.pose.position.z = car_state.kinematics_estimated.pose.position.z();
    odom_ned_msg.pose.pose.orientation.x = car_state.kinematics_estimated.pose.orientation.x();
    odom_ned_msg.pose.pose.orientation.y = car_state.kinematics_estimated.pose.orientation.y();
    odom_ned_msg.pose.pose.orientation.z = car_state.kinematics_estimated.pose.orientation.z();
    odom_ned_msg.pose.pose.orientation.w = car_state.kinematics_estimated.pose.orientation.w();

    odom_ned_msg.twist.twist.linear.x = car_state.kinematics_estimated.twist.linear.x();
    odom_ned_msg.twist.twist.linear.y = car_state.kinematics_estimated.twist.linear.y();
    odom_ned_msg.twist.twist.linear.z = car_state.kinematics_estimated.twist.linear.z();
    odom_ned_msg.twist.twist.angular.x = car_state.kinematics_estimated.twist.angular.x();
    odom_ned_msg.twist.twist.angular.y = car_state.kinematics_estimated.twist.angular.y();
    odom_ned_msg.twist.twist.angular.z = car_state.kinematics_estimated.twist.angular.z();

    return odom_ned_msg;
}

// https://docs.ros.org/jade/api/sensor_msgs/html/point__cloud__conversion_8h_source.html#l00066
// look at UnrealLidarSensor.cpp UnrealLidarSensor::getPointCloud() for math
// read this carefully https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/PointCloud2.html
sensor_msgs::PointCloud2 AirsimCar_ROSWrapper::get_lidar_msg_from_airsim(const msr::airlib::LidarData& lidar_data) const
{
    sensor_msgs::PointCloud2 lidar_msg;
    lidar_msg.header.frame_id = world_frame_id_; // todo

    if (lidar_data.point_cloud.size() > 3)
    {
        lidar_msg.height = 1;
        lidar_msg.width = lidar_data.point_cloud.size() / 3;

        lidar_msg.fields.resize(3);
        lidar_msg.fields[0].name = "x"; 
        lidar_msg.fields[1].name = "y"; 
        lidar_msg.fields[2].name = "z";
        int offset = 0;

        for (size_t d = 0; d < lidar_msg.fields.size(); ++d, offset += 4)
        {
            lidar_msg.fields[d].offset = offset;
            lidar_msg.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
            lidar_msg.fields[d].count  = 1;
        }

        lidar_msg.is_bigendian = false;
        lidar_msg.point_step = offset; // 4 * num fields
        lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width;

        lidar_msg.is_dense = true; // todo
        std::vector<float> data_std = lidar_data.point_cloud;

        const unsigned char* bytes = reinterpret_cast<const unsigned char*>(&data_std[0]);
        vector<unsigned char> lidar_msg_data(bytes, bytes + sizeof(float) * data_std.size());
        lidar_msg.data = std::move(lidar_msg_data);
    }
    else
    {
        // msg = []
    }
    return lidar_msg;
}

// todo covariances
sensor_msgs::Imu AirsimCar_ROSWrapper::get_imu_msg_from_airsim(const msr::airlib::ImuBase::Output& imu_data) const
{
    sensor_msgs::Imu imu_msg;
    // imu_msg.header.frame_id = "/airsim/odom_local_ned";// todo multiple drones
    imu_msg.orientation.x = imu_data.orientation.x();
    imu_msg.orientation.y = imu_data.orientation.y();
    imu_msg.orientation.z = imu_data.orientation.z();
    imu_msg.orientation.w = imu_data.orientation.w();

    // todo radians per second
    imu_msg.angular_velocity.x = math_common::deg2rad(imu_data.angular_velocity.x());
    imu_msg.angular_velocity.y = math_common::deg2rad(imu_data.angular_velocity.y());
    imu_msg.angular_velocity.z = math_common::deg2rad(imu_data.angular_velocity.z());

    // meters/s2^m 
    imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x();
    imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y();
    imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z();

    // imu_msg.orientation_covariance = ;
    // imu_msg.angular_velocity_covariance = ;
    // imu_msg.linear_acceleration_covariance = ;

    return imu_msg;
}

// AirsimCar_ROSWrapper::

    // geometry_msgs::Twist AirsimCar_ROSWrapper::get_twist_msg_from_airsim_state(const msr::airlib::CarApiBase::CarState& car_state) const


geometry_msgs::Twist AirsimCar_ROSWrapper::get_twist_msg_from_airsim_state(const msr::airlib::CarApiBase::CarState& car_state) const
{
    geometry_msgs::Twist twist;

    twist.angular.x = car_state.kinematics_estimated.twist.angular.x();
    twist.angular.y = car_state.kinematics_estimated.twist.angular.y();
    twist.angular.z = car_state.kinematics_estimated.twist.angular.z();
    twist.linear.x = car_state.kinematics_estimated.twist.linear.x();
    twist.linear.y = car_state.kinematics_estimated.twist.linear.y();
    twist.linear.z = car_state.kinematics_estimated.twist.linear.z();

    return twist;

}


void AirsimCar_ROSWrapper::publish_odom_tf(const nav_msgs::Odometry& odom_ned_msg)
{
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header = odom_ned_msg.header;
    odom_tf.child_frame_id = odom_ned_msg.child_frame_id; 
    odom_tf.transform.translation.x = odom_ned_msg.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_ned_msg.pose.pose.position.y;
    odom_tf.transform.translation.z = odom_ned_msg.pose.pose.position.z;
    odom_tf.transform.rotation.x = odom_ned_msg.pose.pose.orientation.x;
    odom_tf.transform.rotation.y = odom_ned_msg.pose.pose.orientation.y;
    odom_tf.transform.rotation.z = odom_ned_msg.pose.pose.orientation.z;
    odom_tf.transform.rotation.w = odom_ned_msg.pose.pose.orientation.w;
    tf_broadcaster_.sendTransform(odom_tf);
}

// todo add reset by vehicle_name API to airlib
// todo not async remove waitonlasttask
bool AirsimCar_ROSWrapper::reset_srv_cb(atypical_ros::Reset::Request& request, atypical_ros::Reset::Response& response)
{
    std::lock_guard<std::recursive_mutex> guard(car_control_mutex_);

    airsim_client_.reset();
    return true; //todo
}

// void AirsimCar_ROSWrapper::acc_cmd_body_frame_cb(const airsim_ros_pkgs::VelCmd& msg, const std::string& vehicle_name)
void AirsimCar_ROSWrapper::acc_cmd_body_frame_cb(const geometry_msgs::TwistStamped::ConstPtr& msg, const std::string& vehicle_name)
{
    std::lock_guard<std::recursive_mutex> guard(car_control_mutex_);

    int vehicle_idx = vehicle_name_idx_map_[vehicle_name];

    double roll, pitch, yaw;
    tf2::Matrix3x3(get_tf2_quat(car_ros_vec_[vehicle_idx].curr_car_state.kinematics_estimated.pose.orientation)).getRPY(roll, pitch, yaw); // ros uses xyzw
    // car_ros_vec_[vehicle_idx].acc_cmd.header = msg->header;
    // car_ros_vec_[vehicle_idx].acc_cmd.twist.linear = msg->twist.linear;    
    
    // car_ros_vec_[vehicle_idx].acc_cmd.twist.angular = msg->twist.angular;

    car_ros_vec_[vehicle_idx].has_acc_cmd = true;
}


void AirsimCar_ROSWrapper::acc_cmd_world_frame_cb(const geometry_msgs::TwistStamped::ConstPtr& msg, const std::string& vehicle_name)
{
    std::lock_guard<std::recursive_mutex> guard(car_control_mutex_);

    int vehicle_idx = vehicle_name_idx_map_[vehicle_name];

    car_ros_vec_[vehicle_idx].acc_cmd.header = msg->header;
    car_ros_vec_[vehicle_idx].acc_cmd.twist = msg->twist;

    // car_ros_vec_[vehicle_idx].acc_cmd.x = msg->twist.linear.x;
    // car_ros_vec_[vehicle_idx].acc_cmd.y = msg->twist.linear.y;
    // car_ros_vec_[vehicle_idx].acc_cmd.z = msg->twist.linear.z;
    // car_ros_vec_[vehicle_idx].acc_cmd.drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    // car_ros_vec_[vehicle_idx].acc_cmd.yaw_mode.is_rate = true;
    // car_ros_vec_[vehicle_idx].acc_cmd.yaw_mode.yaw_or_rate = math_common::rad2deg(msg->twist.angular.z);
    // car_ros_vec_[vehicle_idx].has_acc_cmd = true;
}
// airsim uses nans for zeros in settings.json. we set them to zeros here for handling tfs in ROS 
void AirsimCar_ROSWrapper::set_nans_to_zeros_in_pose(VehicleSetting& vehicle_setting) const
{
    if (std::isnan(vehicle_setting.position.x()))
        vehicle_setting.position.x() = 0.0;

    if (std::isnan(vehicle_setting.position.y()))
        vehicle_setting.position.y() = 0.0;

    if (std::isnan(vehicle_setting.position.z()))
        vehicle_setting.position.z() = 0.0;

    if (std::isnan(vehicle_setting.rotation.yaw))
        vehicle_setting.rotation.yaw = 0.0;

    if (std::isnan(vehicle_setting.rotation.pitch))
        vehicle_setting.rotation.pitch = 0.0;

    if (std::isnan(vehicle_setting.rotation.roll))
        vehicle_setting.rotation.roll = 0.0;
}


void AirsimCar_ROSWrapper::set_nans_to_zeros_in_pose(const VehicleSetting& vehicle_setting, LidarSetting& lidar_setting) const
{
    if (std::isnan(lidar_setting.position.x()))
        lidar_setting.position.x() = vehicle_setting.position.x();

    if (std::isnan(lidar_setting.position.y()))
        lidar_setting.position.y() = vehicle_setting.position.y();

    if (std::isnan(lidar_setting.position.z()))
        lidar_setting.position.z() = vehicle_setting.position.z();

    if (std::isnan(lidar_setting.rotation.yaw))
        lidar_setting.rotation.yaw = vehicle_setting.rotation.yaw;

    if (std::isnan(lidar_setting.rotation.pitch))
        lidar_setting.rotation.pitch = vehicle_setting.rotation.pitch;

    if (std::isnan(lidar_setting.rotation.roll))
        lidar_setting.rotation.roll = vehicle_setting.rotation.roll;
}

tf2::Quaternion AirsimCar_ROSWrapper::get_tf2_quat(const msr::airlib::Quaternionr& airlib_quat) const
{
    return tf2::Quaternion(airlib_quat.x(), airlib_quat.y(), airlib_quat.z(), airlib_quat.w());
}

msr::airlib::Quaternionr AirsimCar_ROSWrapper::get_airlib_quat(const geometry_msgs::Quaternion& geometry_msgs_quat) const
{
    return msr::airlib::Quaternionr(geometry_msgs_quat.w, geometry_msgs_quat.x, geometry_msgs_quat.y, geometry_msgs_quat.z); 
}

msr::airlib::Quaternionr AirsimCar_ROSWrapper::get_airlib_quat(const tf2::Quaternion& tf2_quat) const
{
    return msr::airlib::Quaternionr(tf2_quat.w(), tf2_quat.x(), tf2_quat.y(), tf2_quat.z()); 
}


void AirsimCar_ROSWrapper::lidar_timer_cb(const ros::TimerEvent& event)
{    
    try
    {
        // std::lock_guard<std::recursive_mutex> guard(drone_control_mutex_);
        if (lidar_pub_vec_.size() > 0)
        {
            // std::lock_guard<std::recursive_mutex> guard(lidar_mutex_);
            int ctr = 0;
            for (const auto& vehicle_lidar_pair: vehicle_lidar_map_)
            {
                std::unique_lock<std::recursive_mutex> lck(car_control_mutex_);

                auto lidar_data = airsim_client_lidar_.getLidarData(vehicle_lidar_pair.second, vehicle_lidar_pair.first); // airsim api is imu_name, vehicle_name



                ROS_DEBUG_STREAM("In timer cb");


                lck.unlock();
                sensor_msgs::PointCloud2 lidar_msg = get_lidar_msg_from_airsim(lidar_data); // todo make const ptr msg to avoid copy
                lidar_msg.header.frame_id = vehicle_lidar_pair.second; // sensor frame name. todo add to doc
                lidar_msg.header.stamp = ros::Time::now();
                lidar_pub_vec_[ctr].publish(lidar_msg);

                // JBS
            
                // auto pose_true = airsim_client_object_.simGetObjectPose(object_name); // ned
                // ROS_DEBUG_STREAM("Object "<< object_name << " : " << pose_true.position.x() << " , " << pose_true.position.y()  << " , " << pose_true.position.z());
                // geometry_msgs::PoseStamped object_pose_ned;
                // object_pose_ned.header.frame_id = "map"; // ?
                // object_pose_ned.pose.position.x = pose_true.position.x();
                // object_pose_ned.pose.position.y = pose_true.position.y();
                // object_pose_ned.pose.position.z = pose_true.position.z();
                // object_pose_ned.pose.orientation.x = pose_true.orientation.x();
                // object_pose_ned.pose.orientation.y = pose_true.orientation.y();
                // object_pose_ned.pose.orientation.z = pose_true.orientation.z();
                // object_pose_ned.pose.orientation.w = pose_true.orientation.w();

                // if (not std::isnan(pose_true.position.x())){
                //     pose_object_enu_pub.publish(ned_pose_to_enu_pose(object_pose_ned));
                // }
                
                ctr++;
            } 
        }





    }

    catch (rpc::rpc_error& e)
    {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, didn't get image response." << std::endl << msg << std::endl;
    }

}


void AirsimCar_ROSWrapper::append_static_lidar_tf(const std::string& vehicle_name, const std::string& lidar_name, const LidarSetting& lidar_setting)
{

    geometry_msgs::TransformStamped lidar_tf_msg;
    lidar_tf_msg.header.frame_id = vehicle_name + "/odom_local_ned"; // todo multiple drones
    lidar_tf_msg.child_frame_id = lidar_name;
    lidar_tf_msg.transform.translation.x = lidar_setting.position.x();
    lidar_tf_msg.transform.translation.y = lidar_setting.position.y();
    lidar_tf_msg.transform.translation.z = lidar_setting.position.z();
    tf2::Quaternion quat;
    quat.setRPY(lidar_setting.rotation.roll, lidar_setting.rotation.pitch, lidar_setting.rotation.yaw);
    lidar_tf_msg.transform.rotation.x = quat.x();
    lidar_tf_msg.transform.rotation.y = quat.y();
    lidar_tf_msg.transform.rotation.z = quat.z();
    lidar_tf_msg.transform.rotation.w = quat.w();

    static_tf_msg_vec_.push_back(lidar_tf_msg);
}

void AirsimCar_ROSWrapper::append_static_vehicle_tf(const std::string& vehicle_name, const VehicleSetting& vehicle_setting)
{
    geometry_msgs::TransformStamped vehicle_tf_msg;
    vehicle_tf_msg.header.frame_id = world_frame_id_;
    vehicle_tf_msg.header.stamp = ros::Time::now();
    vehicle_tf_msg.child_frame_id = vehicle_name;
    vehicle_tf_msg.transform.translation.x = vehicle_setting.position.x();
    vehicle_tf_msg.transform.translation.y = vehicle_setting.position.y();
    vehicle_tf_msg.transform.translation.z = vehicle_setting.position.z();
    tf2::Quaternion quat;
    quat.setRPY(vehicle_setting.rotation.roll, vehicle_setting.rotation.pitch, vehicle_setting.rotation.yaw);
    vehicle_tf_msg.transform.rotation.x = quat.x();
    vehicle_tf_msg.transform.rotation.y = quat.y();
    vehicle_tf_msg.transform.rotation.z = quat.z();
    vehicle_tf_msg.transform.rotation.w = quat.w();

    static_tf_msg_vec_.push_back(vehicle_tf_msg);
}

atypical_ros::GPSYaw AirsimCar_ROSWrapper::get_gps_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const
{
    atypical_ros::GPSYaw gps_msg;
    gps_msg.latitude = geo_point.latitude;
    gps_msg.longitude = geo_point.longitude; 
    gps_msg.altitude = geo_point.altitude;
    return gps_msg;
}


void AirsimCar_ROSWrapper::vehicle_cmd_cb(const driving_msgs::VehicleCmd::ConstPtr& msg, const std::string& vehicle_name)
{
    std::lock_guard<std::recursive_mutex> guard(car_control_mutex_);

    int vehicle_idx = vehicle_name_idx_map_[vehicle_name];

    double roll, pitch, yaw;
    tf2::Matrix3x3(get_tf2_quat(car_ros_vec_[vehicle_idx].curr_car_state.kinematics_estimated.pose.orientation)).getRPY(roll, pitch, yaw); // ros uses xyzw

    // yaw = yaw + PI/2;
    // car_ros_vec_[vehicle_idx].acc_cmd.header = msg->header;
    // car_ros_vec_[vehicle_idx].acc_cmd.twist.linear.x = msg->twist.linear.x * cos(yaw) + (msg->twist.linear.y * sin(yaw)); //body frame assuming zero pitch roll
    //car_ros_vec_[vehicle_idx].acc_cmd.twist.linear.y = (msg->twist.linear.x * sin(yaw)) + (msg->twist.linear.y * cos(yaw)); //body frame
    car_ros_vec_[vehicle_idx].acc_cmd.twist.linear.x = msg->accel_decel_cmd;
    car_ros_vec_[vehicle_idx].acc_cmd.twist.linear.y= 0.0;
    car_ros_vec_[vehicle_idx].acc_cmd.twist.linear.z = 0.0;
        
    // std::cout << car_ros_vec_[vehicle_idx].acc_cmd.twist.linear.x  << std::endl;
    // std:: cout << "yaw "; std::cout << yaw << std::endl;
    car_ros_vec_[vehicle_idx].acc_cmd.twist.angular.x = 0.0;
    car_ros_vec_[vehicle_idx].acc_cmd.twist.angular.y = 0.0;
    car_ros_vec_[vehicle_idx].acc_cmd.twist.angular.z = msg->steer_angle_cmd;
    

    // todo do actual body frame?
    // car_ros_vec_[vehicle_idx].acc_cmd.x = (msg->twist.linear.x * cos(yaw)) - (msg->twist.linear.y * sin(yaw)); //body frame assuming zero pitch roll
    // car_ros_vec_[vehicle_idx].acc_cmd.y = (msg->twist.linear.x * sin(yaw)) + (msg->twist.linear.y * cos(yaw)); //body frame
    // car_ros_vec_[vehicle_idx].acc_cmd.z = msg->twist.linear.z;
    // car_ros_vec_[vehicle_idx].acc_cmd.drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    // car_ros_vec_[vehicle_idx].acc_cmd.yaw_mode.is_rate = true;
    // airsim uses degrees
    // car_ros_vec_[vehicle_idx].acc_cmd.yaw_mode.yaw_or_rate = math_common::rad2deg(msg->twist.angular.z);
    car_ros_vec_[vehicle_idx].has_acc_cmd = true;
}
