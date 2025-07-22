#pragma once

#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "preprocess.h"
#include "ikd-Tree/ikd_Tree.h"
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

struct StampedMessage {
    double timestamp;
    sensor_msgs::msg::Imu::ConstSharedPtr imu_msg;
    PointCloudXYZI::Ptr lidar_msg;
    tf2_msgs::msg::TFMessage::ConstSharedPtr tf_msg;
    string topic_name; // To distinguish message types

    bool operator<(const StampedMessage& other) const {
        return timestamp < other.timestamp;
    }
};

class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~LaserMappingNode();

    void process_bag_file(const std::string& bag_path);
    FrameResult process_frame(const MeasureGroup& meas);
    bool sync_packages(MeasureGroup &meas);

private:

    void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg);
    void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg);
    void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in);
    void initialization_thread_func();
    void processing_thread_func();
    void accumulate_map_points();
    void map_publish_callback();
    void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);

    std::thread init_thread_;
    std::thread processing_thread_;
    std::mutex mtx_buffer_;
    std::condition_variable sig_buffer_;
    deque<double> time_buffer_;
    deque<PointCloudXYZI::Ptr> lidar_buffer_;
    deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer_;
    bool timediff_set_flg = false;
    double timediff_lidar_wrt_imu = 0.0;
    bool flg_is_system_initialized_ = false;
    bool flg_exit_ = false;


    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

    bool effect_pub_en = false, map_pub_en = false;
    double epsi[23] = {0.001};

    FILE *fp;
    ofstream fout_pre, fout_out, fout_dbg;

    std::string bag_file_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pubClock_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    std::deque<StampedMessage> message_buffer_;
    double bag_buffer_time_sec_ = 2.0;
    bool offline_buffer_enabled = true;
};
