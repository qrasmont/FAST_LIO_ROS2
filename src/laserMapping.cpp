// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "fast_lio/laserMapping.h"
#include "fast_lio/common_lib.h"
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

std::string tf_topic = "/tf";
std::string tf_static_topic = "/tf_static";

LaserMappingNode::LaserMappingNode(const rclcpp::NodeOptions& options) : Node("laser_mapping", options)
{
    FastLioConfig config;
    this->declare_parameter<bool>("publish.path_en", true);
    this->declare_parameter<bool>("publish.effect_map_en", false);
    this->declare_parameter<bool>("publish.map_en", false);
    this->declare_parameter<bool>("publish.scan_publish_en", true);
    this->declare_parameter<bool>("publish.dense_publish_en", false);
    this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
    this->declare_parameter<int>("max_iteration", 4);
    this->declare_parameter<std::string>("map_file_path", "");
    this->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
    this->declare_parameter<std::string>("common.imu_topic", "/livox/imu");
    this->declare_parameter<bool>("common.time_sync_en", false);
    this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
    this->declare_parameter<double>("filter_size_surf", 0.5);
    this->declare_parameter<double>("filter_size_map", 0.5);
    this->declare_parameter<double>("cube_side_length", 200.);
    this->declare_parameter<double>("mapping.det_range", 100.0);
    this->declare_parameter<double>("mapping.fov_degree", 180.);
    this->declare_parameter<double>("mapping.gyr_cov", 0.1);
    this->declare_parameter<double>("mapping.acc_cov", 0.1);
    this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
    this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
    this->declare_parameter<double>("preprocess.blind", 0.01);
    this->declare_parameter<int>("preprocess.lidar_type", 1); // AVIA
    this->declare_parameter<int>("preprocess.scan_line", 16);
    this->declare_parameter<int>("preprocess.timestamp_unit", 2); // US
    this->declare_parameter<int>("preprocess.scan_rate", 10);
    this->declare_parameter<int>("point_filter_num", 2);
    this->declare_parameter<bool>("feature_extract_enable", false);
    this->declare_parameter<bool>("runtime_pos_log_enable", false);
    this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
    this->declare_parameter<bool>("pcd_save.pcd_save_en", true);
    this->declare_parameter<std::string>("pcd_save.save_path", "PCD/");
    this->declare_parameter<int>("pcd_save.interval", -1);
    this->declare_parameter<std::vector<double>>("mapping.extrinsic_T", std::vector<double>({0.0, 0.0, 0.0}));
    this->declare_parameter<std::vector<double>>("mapping.extrinsic_R", std::vector<double>({1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}));
    this->declare_parameter<std::string>("log_path", "Log/");
    this->declare_parameter<std::string>("bag_file", "");
    this->declare_parameter<bool>("offline_buffer_enabled", true);
    this->declare_parameter<double>("bag_buffer_time_sec", 2.0);
    this->declare_parameter<std::string>("qos_profile", "default");


    this->get_parameter("publish.path_en", path_en_);
    this->get_parameter("publish.effect_map_en", effect_pub_en_);
    this->get_parameter("publish.map_en", map_pub_en_);
    this->get_parameter("publish.scan_publish_en", scan_pub_en_);
    this->get_parameter("publish.scan_bodyframe_pub_en", scan_body_pub_en_);
    config.max_iteration = this->get_parameter("max_iteration").as_int();
    config.map_file_path = this->get_parameter("map_file_path").as_string();
    lid_topic_ = this->get_parameter("common.lid_topic").as_string();
    imu_topic_ = this->get_parameter("common.imu_topic").as_string();
    config.time_sync_en = this->get_parameter("common.time_sync_en").as_bool();
    config.time_offset_lidar_to_imu = this->get_parameter("common.time_offset_lidar_to_imu").as_double();
    config.filter_size_surf = this->get_parameter("filter_size_surf").as_double();
    config.filter_size_map = this->get_parameter("filter_size_map").as_double();
    config.cube_side_length = this->get_parameter("cube_side_length").as_double();
    config.fov_degree = this->get_parameter("mapping.fov_degree").as_double();
    config.det_range = this->get_parameter("mapping.det_range").as_double();
    config.gyr_cov = this->get_parameter("mapping.gyr_cov").as_double();
    config.acc_cov = this->get_parameter("mapping.acc_cov").as_double();
    config.b_gyr_cov = this->get_parameter("mapping.b_gyr_cov").as_double();
    config.b_acc_cov = this->get_parameter("mapping.b_acc_cov").as_double();
    config.blind = this->get_parameter("preprocess.blind").as_double();
    config.lidar_type = this->get_parameter("preprocess.lidar_type").as_int();
    config.scan_line = this->get_parameter("preprocess.scan_line").as_int();
    config.timestamp_unit = this->get_parameter("preprocess.timestamp_unit").as_int();
    config.scan_rate = this->get_parameter("preprocess.scan_rate").as_int();
    config.point_filter_num = this->get_parameter("point_filter_num").as_int();
    config.feature_extract_enable = this->get_parameter("feature_extract_enable").as_bool();
    config.runtime_pos_log_enable = this->get_parameter("runtime_pos_log_enable").as_bool();
    config.extrinsic_est_en = this->get_parameter("mapping.extrinsic_est_en").as_bool();
    config.pcd_save_en = this->get_parameter("pcd_save.pcd_save_en").as_bool();
    config.pcd_save_path = this->get_parameter("pcd_save.save_path").as_string();
    config.pcd_save_interval = this->get_parameter("pcd_save.interval").as_int();
    config.extrinsic_T = this->get_parameter("mapping.extrinsic_T").as_double_array();
    config.extrinsic_R = this->get_parameter("mapping.extrinsic_R").as_double_array();
    config.log_path = this->get_parameter("log_path").as_string();
    this->get_parameter("offline_buffer_enabled", offline_buffer_enabled_);
    config.dense_publish_en = this->get_parameter("publish.dense_publish_en").as_bool();
    config.map_pub_en = this->get_parameter("publish.map_en").as_bool();
    this->get_parameter("qos_profile", qos_profile_);

    fast_lio_core_ = std::make_unique<FastLioCore>(config);

    rclcpp::QoS qos(20);

    if (qos_profile_ == "sensor_data") {
        RCLCPP_INFO(this->get_logger(), "Using 'sensor_data' QoS profile for subscribers.");
        qos = rclcpp::SensorDataQoS();
    } else {
        RCLCPP_INFO(this->get_logger(), "Using 'default' (Reliable, depth 20) QoS profile for subscribers.");
        qos = rclcpp::QoS(20);
    }

    if (config.lidar_type == 1) // AVIA
    {
        sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic_, qos, std::bind(&LaserMappingNode::livox_pcl_cbk, this, std::placeholders::_1));
    }
    else
    {
        sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic_, qos, std::bind(&LaserMappingNode::standard_pcl_cbk, this, std::placeholders::_1));
    }
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_, qos, std::bind(&LaserMappingNode::imu_cbk, this, std::placeholders::_1));

    pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 20);
    pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);
    pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 20);
    pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 20);
    pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
    pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 20);
    pubClock_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);


    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);


    auto map_period_ms = std::chrono::milliseconds(1000);
    map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_period_ms, std::bind(&LaserMappingNode::map_publish_callback, this));
    map_save_srv_ = this->create_service<std_srvs::srv::Trigger>("map_save", std::bind(&LaserMappingNode::map_save_callback, this, std::placeholders::_1, std::placeholders::_2));

    path_.header.stamp = this->get_clock()->now();
    path_.header.frame_id ="camera_init";

    processing_thread_ = std::thread(&LaserMappingNode::processing_thread_func, this);

    RCLCPP_INFO(this->get_logger(), "Node init finished.");
}

LaserMappingNode::~LaserMappingNode()
{
    fast_lio_core_->flg_exit_ = true;
    fast_lio_core_->sig_buffer_.notify_all();
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
}

void LaserMappingNode::standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
    std::lock_guard<std::mutex> lock(fast_lio_core_->mtx_buffer_);
    double cur_time = get_time_sec(msg->header.stamp);

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    fast_lio_core_->p_pre_->process(msg, ptr);

    fast_lio_core_->lidar_buffer_.push_back(ptr);
    fast_lio_core_->time_buffer_.push_back(cur_time);
    fast_lio_core_->sig_buffer_.notify_one();
}

void LaserMappingNode::livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg)
{
    std::lock_guard<std::mutex> lock(fast_lio_core_->mtx_buffer_);
    double cur_time = get_time_sec(msg->header.stamp);

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    fast_lio_core_->p_pre_->process(msg, ptr);

    fast_lio_core_->lidar_buffer_.push_back(ptr);
    fast_lio_core_->time_buffer_.push_back(cur_time);
    fast_lio_core_->sig_buffer_.notify_one();
}

void LaserMappingNode::imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    std::lock_guard<std::mutex> lock(fast_lio_core_->mtx_buffer_);
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
    fast_lio_core_->imu_buffer_.push_back(msg);
    fast_lio_core_->sig_buffer_.notify_one();
}

void LaserMappingNode::processing_thread_func()
{
    RCLCPP_INFO(this->get_logger(), "Processing thread started.");

    fast_lio_core_->initial_setup();

    RCLCPP_INFO(this->get_logger(), "Initialization complete, starting main processing loop.");

    while (rclcpp::ok())
    {
        std::unique_lock<std::mutex> lock(fast_lio_core_->mtx_buffer_);

        // Wait until there is a complete data package
        fast_lio_core_->sig_buffer_.wait(lock, [&] {
            if (fast_lio_core_->flg_exit_)
            {
                return true;
            }

            // There aren't enough LIDAR scans or IMU messages to form a package.
            if (fast_lio_core_->lidar_buffer_.size() < 2 || fast_lio_core_->imu_buffer_.empty()) {
                return false;
            }

            // We must have IMU data that covers the entire first scan.
            return get_time_sec(fast_lio_core_->imu_buffer_.back()->header.stamp) >= fast_lio_core_->time_buffer_.at(1);
        });

        if (fast_lio_core_->flg_exit_)
        {
            break;
        }

        lock.unlock();

        // Process package
        fast_lio_core_->process();

        fast_lio_core_->accumulate_map_points();

        // Publish odom & clouds
        nav_msgs::msg::Odometry odom;
        geometry_msgs::msg::Quaternion quat;
        if (fast_lio_core_->get_publish_odometry(odom, quat))
        {
            odom.header.stamp = get_ros_time(fast_lio_core_->get_lidar_end_time());
            pubOdomAftMapped_->publish(odom);

            geometry_msgs::msg::TransformStamped trans;
            trans.header.frame_id = "camera_init";
            trans.header.stamp = odom.header.stamp;
            trans.child_frame_id = "body";
            trans.transform.translation.x = odom.pose.pose.position.x;
            trans.transform.translation.y = odom.pose.pose.position.y;
            trans.transform.translation.z = odom.pose.pose.position.z;
            trans.transform.rotation = quat;
            tf_broadcaster_->sendTransform(trans);

            if (path_en_ && fast_lio_core_->get_publish_path(path_, odom))
            {
                pubPath_->publish(path_);
            }
        }

        PointCloudXYZI::Ptr cloud_world;
        if (scan_pub_en_ && fast_lio_core_->get_publish_frame_world(cloud_world))
        {
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*cloud_world, msg);
            msg.header.stamp = get_ros_time(fast_lio_core_->get_lidar_end_time());
            msg.header.frame_id = "camera_init";
            pubLaserCloudFull_->publish(msg);
        }

        PointCloudXYZI::Ptr cloud_body;
        if (scan_body_pub_en_ && fast_lio_core_->get_publish_frame_body(cloud_body))
        {
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*cloud_body, msg);
            msg.header.stamp = get_ros_time(fast_lio_core_->get_lidar_end_time());
            msg.header.frame_id = "body";
            pubLaserCloudFull_body_->publish(msg);
        }

        PointCloudXYZI::Ptr cloud_effect;
        if (effect_pub_en_ && fast_lio_core_->get_publish_effect_world(cloud_effect))
        {
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*cloud_effect, msg);
            msg.header.stamp = get_ros_time(fast_lio_core_->get_lidar_end_time());
            msg.header.frame_id = "camera_init";
            pubLaserCloudEffect_->publish(msg);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Processing thread stopped.");
}

void LaserMappingNode::process_bag_file(const std::string& bag_path)
{
    std::string storage_id = "";
    double bag_buffer_time_sec = this->get_parameter("bag_buffer_time_sec").as_double();

    rclcpp::Rate rate(200.0);

    if (offline_buffer_enabled_)
    {
        RCLCPP_INFO(this->get_logger(), "Offline mode with BUFFERED reading.");
        rosbag2_storage::StorageOptions storage_options({bag_path, storage_id});
        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";
        rosbag2_cpp::readers::SequentialReader reader;

        try {
            reader.open(storage_options, converter_options);
            rosbag2_storage::StorageFilter filter;
            filter.topics = {lid_topic_, imu_topic_, tf_topic, tf_static_topic};
            reader.set_filter(filter);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Fatal error opening bag file: %s", e.what());
            return;
        }

        rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
        rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> livox_serialization;
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_serialization;
        rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_serialization;

        RCLCPP_INFO(this->get_logger(), "Reading initial messages for IMU initialization...");
        deque<sensor_msgs::msg::Imu::ConstSharedPtr> init_imu_data;
        while (reader.has_next() && init_imu_data.size() < INIT_IMU_COUNT) {
            auto serialized_msg = reader.read_next();
            if (serialized_msg->topic_name == imu_topic_) {
                auto msg = std::make_shared<sensor_msgs::msg::Imu>();
                rclcpp::SerializedMessage extracted_serialized_msg(*serialized_msg->serialized_data);
                imu_serialization.deserialize_message(&extracted_serialized_msg, msg.get());
                init_imu_data.push_back(msg);
            }
        }

        if (init_imu_data.size() < INIT_IMU_COUNT) {
            RCLCPP_ERROR(this->get_logger(), "Not enough IMU messages in bag to initialize. Found %zu, need %d.", init_imu_data.size(), INIT_IMU_COUNT);
            return;
        }

        std::stable_sort(init_imu_data.begin(), init_imu_data.end(),
            [](const sensor_msgs::msg::Imu::ConstSharedPtr& a, const sensor_msgs::msg::Imu::ConstSharedPtr& b) {
            return get_time_sec(a->header.stamp) < get_time_sec(b->header.stamp);
        });

        fast_lio_core_->p_imu_->IMU_init(init_imu_data, fast_lio_core_->kf_);
        fast_lio_core_->flg_is_system_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "System initialized. Starting processing.");

        reader.seek(0);
        std::deque<StampedMessage> message_buffer;

        while (reader.has_next() && rclcpp::ok()) {
            while (reader.has_next()) {
                if (!message_buffer.empty() && 
                    (message_buffer.back().timestamp - message_buffer.front().timestamp > bag_buffer_time_sec)) {
                    break; 
                }
                auto serialized_msg = reader.read_next();
                rclcpp::SerializedMessage extracted_serialized_msg(*serialized_msg->serialized_data);

                if (serialized_msg->topic_name == imu_topic_) {
                    auto msg = std::make_shared<sensor_msgs::msg::Imu>();
                    imu_serialization.deserialize_message(&extracted_serialized_msg, msg.get());
                    message_buffer.push_back({get_time_sec(msg->header.stamp), msg, nullptr, nullptr, imu_topic_});
                } else if (serialized_msg->topic_name == lid_topic_) {
                    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
                    double header_stamp = 0.0;
                    if (fast_lio_core_->p_pre_->lidar_type == AVIA) {
                        auto livox_msg = std::make_unique<livox_ros_driver2::msg::CustomMsg>();
                        livox_serialization.deserialize_message(&extracted_serialized_msg, livox_msg.get());
                        header_stamp = get_time_sec(livox_msg->header.stamp);
                        fast_lio_core_->p_pre_->process(std::move(livox_msg), cloud);
                    } else {
                        auto pc2_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
                        pc2_serialization.deserialize_message(&extracted_serialized_msg, pc2_msg.get());
                        header_stamp = get_time_sec(pc2_msg->header.stamp);
                        fast_lio_core_->p_pre_->process(std::move(pc2_msg), cloud);
                    }
                    message_buffer.push_back({header_stamp, nullptr, cloud, nullptr, lid_topic_});
                } else if (serialized_msg->topic_name == tf_topic || serialized_msg->topic_name == tf_static_topic) {
                    auto msg = std::make_shared<tf2_msgs::msg::TFMessage>();
                    tf_serialization.deserialize_message(&extracted_serialized_msg, msg.get());
                    if (!msg->transforms.empty()) {
                        message_buffer.push_back({get_time_sec(msg->transforms[0].header.stamp), nullptr, nullptr, msg, serialized_msg->topic_name});
                    }
                }
            }

            std::stable_sort(message_buffer.begin(), message_buffer.end());

            double process_until_time = message_buffer.back().timestamp - (bag_buffer_time_sec / 2.0);
            if (!reader.has_next()) {
                process_until_time = std::numeric_limits<double>::max();
            }

            while (!message_buffer.empty() && message_buffer.front().timestamp < process_until_time) {
                StampedMessage stamped_msg = message_buffer.front();
                message_buffer.pop_front();

                rosgraph_msgs::msg::Clock clock_msg;
                clock_msg.clock = get_ros_time(stamped_msg.timestamp);
                pubClock_->publish(clock_msg);

                if (stamped_msg.topic_name == imu_topic_) {
                    fast_lio_core_->imu_buffer_.push_back(stamped_msg.imu_msg);
                } else if (stamped_msg.topic_name == lid_topic_) {
                    fast_lio_core_->lidar_buffer_.push_back(stamped_msg.lidar_msg);
                    fast_lio_core_->time_buffer_.push_back(stamped_msg.timestamp);
                } else if (stamped_msg.topic_name == tf_topic) {
                    tf_broadcaster_->sendTransform(stamped_msg.tf_msg->transforms);
                } else if (stamped_msg.topic_name == tf_static_topic) {
                    static_tf_broadcaster_->sendTransform(stamped_msg.tf_msg->transforms);
                }

                MeasureGroup meas;
                if (fast_lio_core_->sync_packages(meas)) {
                    fast_lio_core_->process_frame(meas);
                    fast_lio_core_->accumulate_map_points();

                    // Publish results
                    nav_msgs::msg::Odometry odom;
                    geometry_msgs::msg::Quaternion quat;
                    if (fast_lio_core_->get_publish_odometry(odom, quat))
                    {
                        odom.header.stamp = get_ros_time(fast_lio_core_->get_lidar_end_time());
                        pubOdomAftMapped_->publish(odom);

                        geometry_msgs::msg::TransformStamped trans;
                        trans.header.frame_id = "camera_init";
                        trans.header.stamp = odom.header.stamp;
                        trans.child_frame_id = "body";
                        trans.transform.translation.x = odom.pose.pose.position.x;
                        trans.transform.translation.y = odom.pose.pose.position.y;
                        trans.transform.translation.z = odom.pose.pose.position.z;
                        trans.transform.rotation = quat;
                        tf_broadcaster_->sendTransform(trans);

                        if (path_en_ && fast_lio_core_->get_publish_path(path_, odom))
                        {
                            pubPath_->publish(path_);
                        }
                    }

                    PointCloudXYZI::Ptr cloud_world;
                    if (scan_pub_en_ && fast_lio_core_->get_publish_frame_world(cloud_world))
                    {
                        sensor_msgs::msg::PointCloud2 msg;
                        pcl::toROSMsg(*cloud_world, msg);
                        msg.header.stamp = get_ros_time(fast_lio_core_->get_lidar_end_time());
                        msg.header.frame_id = "camera_init";
                        pubLaserCloudFull_->publish(msg);
                    }

                    PointCloudXYZI::Ptr cloud_body;
                    if (scan_body_pub_en_ && fast_lio_core_->get_publish_frame_body(cloud_body))
                    {
                        sensor_msgs::msg::PointCloud2 msg;
                        pcl::toROSMsg(*cloud_body, msg);
                        msg.header.stamp = get_ros_time(fast_lio_core_->get_lidar_end_time());
                        msg.header.frame_id = "body";
                        pubLaserCloudFull_body_->publish(msg);
                    }

                    PointCloudXYZI::Ptr cloud_effect;
                    if (effect_pub_en_ && fast_lio_core_->get_publish_effect_world(cloud_effect))
                    {
                        sensor_msgs::msg::PointCloud2 msg;
                        pcl::toROSMsg(*cloud_effect, msg);
                        msg.header.stamp = get_ros_time(fast_lio_core_->get_lidar_end_time());
                        msg.header.frame_id = "camera_init";
                        pubLaserCloudEffect_->publish(msg);
                    }

                    rclcpp::spin_some(this->get_node_base_interface());
                    rate.sleep();
                }
            }
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Offline mode with FULL BAG reading.");
        std::vector<StampedMessage> all_messages;
        rosbag2_storage::StorageOptions storage_options({bag_path, storage_id});
        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";
        rosbag2_cpp::readers::SequentialReader reader;

        try {
            reader.open(storage_options, converter_options);
            rosbag2_storage::StorageFilter filter;
            filter.topics = {lid_topic_, imu_topic_, tf_topic, tf_static_topic};
            reader.set_filter(filter);

            rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
            rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> livox_serialization;
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_serialization;
            rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_serialization;

            while (reader.has_next())
            {
                auto serialized_msg = reader.read_next();
                rclcpp::SerializedMessage extracted_serialized_msg(*serialized_msg->serialized_data);

                if (serialized_msg->topic_name == imu_topic_) {
                    auto msg = std::make_shared<sensor_msgs::msg::Imu>();
                    imu_serialization.deserialize_message(&extracted_serialized_msg, msg.get());
                    all_messages.push_back({get_time_sec(msg->header.stamp), msg, nullptr, nullptr, imu_topic_});
                } else if (serialized_msg->topic_name == lid_topic_) {
                    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
                    double header_stamp = 0.0;
                    if (fast_lio_core_->p_pre_->lidar_type == AVIA) {
                        auto livox_msg = std::make_unique<livox_ros_driver2::msg::CustomMsg>();
                        livox_serialization.deserialize_message(&extracted_serialized_msg, livox_msg.get());
                        header_stamp = get_time_sec(livox_msg->header.stamp);
                        fast_lio_core_->p_pre_->process(std::move(livox_msg), cloud);
                    } else {
                        auto pc2_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
                        pc2_serialization.deserialize_message(&extracted_serialized_msg, pc2_msg.get());
                        header_stamp = get_time_sec(pc2_msg->header.stamp);
                        fast_lio_core_->p_pre_->process(std::move(pc2_msg), cloud);
                    }
                    all_messages.push_back({header_stamp, nullptr, cloud, nullptr, lid_topic_});
                } else if (serialized_msg->topic_name == tf_topic || serialized_msg->topic_name == tf_static_topic) {
                    auto msg = std::make_shared<tf2_msgs::msg::TFMessage>();
                    tf_serialization.deserialize_message(&extracted_serialized_msg, msg.get());
                    if (!msg->transforms.empty()) {
                        all_messages.push_back({get_time_sec(msg->transforms[0].header.stamp), nullptr, nullptr, msg, serialized_msg->topic_name});
                    }
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Fatal error reading bag file: %s", e.what());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Read %zu messages. Sorting...", all_messages.size());
        std::stable_sort(all_messages.begin(), all_messages.end());
        RCLCPP_INFO(this->get_logger(), "Processing sorted messages...");

        deque<sensor_msgs::msg::Imu::ConstSharedPtr> init_imu_data;
        for (const auto& msg : all_messages) {
            if (msg.topic_name == imu_topic_) {
                init_imu_data.push_back(msg.imu_msg);
                if (init_imu_data.size() >= INIT_IMU_COUNT) break;
            }
        }

        fast_lio_core_->p_imu_->IMU_init(init_imu_data, fast_lio_core_->kf_);
        fast_lio_core_->flg_is_system_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "System initialized. Starting processing.");

        for (const auto& msg : all_messages)
        {
            rosgraph_msgs::msg::Clock clock_msg;
            clock_msg.clock = get_ros_time(msg.timestamp);
            pubClock_->publish(clock_msg);

            if (msg.topic_name == imu_topic_) {
                fast_lio_core_->imu_buffer_.push_back(msg.imu_msg);
            } else if (msg.topic_name == lid_topic_) {
                fast_lio_core_->lidar_buffer_.push_back(msg.lidar_msg);
                fast_lio_core_->time_buffer_.push_back(msg.timestamp);
            } else if (msg.topic_name == tf_topic) {
                tf_broadcaster_->sendTransform(msg.tf_msg->transforms);
            } else if (msg.topic_name == tf_static_topic) {
                static_tf_broadcaster_->sendTransform(msg.tf_msg->transforms);
            }

            MeasureGroup meas;
            if (fast_lio_core_->sync_packages(meas)) {
                fast_lio_core_->process_frame(meas);
                fast_lio_core_->accumulate_map_points();

                nav_msgs::msg::Odometry odom;
                geometry_msgs::msg::Quaternion quat;
                if (fast_lio_core_->get_publish_odometry(odom, quat))
                {
                    odom.header.stamp = get_ros_time(fast_lio_core_->get_lidar_end_time());
                    pubOdomAftMapped_->publish(odom);

                    geometry_msgs::msg::TransformStamped trans;
                    trans.header.frame_id = "camera_init";
                    trans.header.stamp = odom.header.stamp;
                    trans.child_frame_id = "body";
                    trans.transform.translation.x = odom.pose.pose.position.x;
                    trans.transform.translation.y = odom.pose.pose.position.y;
                    trans.transform.translation.z = odom.pose.pose.position.z;
                    trans.transform.rotation = quat;
                    tf_broadcaster_->sendTransform(trans);

                    if (path_en_ && fast_lio_core_->get_publish_path(path_, odom))
                    {
                        pubPath_->publish(path_);
                    }
                }

                PointCloudXYZI::Ptr cloud_world;
                if (scan_pub_en_ && fast_lio_core_->get_publish_frame_world(cloud_world))
                {
                    sensor_msgs::msg::PointCloud2 msg;
                    pcl::toROSMsg(*cloud_world, msg);
                    msg.header.stamp = get_ros_time(fast_lio_core_->get_lidar_end_time());
                    msg.header.frame_id = "camera_init";
                    pubLaserCloudFull_->publish(msg);
                }

                PointCloudXYZI::Ptr cloud_body;
                if (scan_body_pub_en_ && fast_lio_core_->get_publish_frame_body(cloud_body))
                {
                    sensor_msgs::msg::PointCloud2 msg;
                    pcl::toROSMsg(*cloud_body, msg);
                    msg.header.stamp = get_ros_time(fast_lio_core_->get_lidar_end_time());
                    msg.header.frame_id = "body";
                    pubLaserCloudFull_body_->publish(msg);
                }

                PointCloudXYZI::Ptr cloud_effect;
                if (effect_pub_en_ && fast_lio_core_->get_publish_effect_world(cloud_effect))
                {
                    sensor_msgs::msg::PointCloud2 msg;
                    pcl::toROSMsg(*cloud_effect, msg);
                    msg.header.stamp = get_ros_time(fast_lio_core_->get_lidar_end_time());
                    msg.header.frame_id = "camera_init";
                    pubLaserCloudEffect_->publish(msg);
                }

                rclcpp::spin_some(this->get_node_base_interface());
                rate.sleep();
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "Offline processing finished.");
}

void LaserMappingNode::map_publish_callback()
{
    if (map_pub_en_)
    {
        PointCloudXYZI::Ptr map_cloud;
        if (fast_lio_core_->get_publish_map(map_cloud))
        {
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*map_cloud, msg);
            msg.header.stamp = this->get_clock()->now();
            msg.header.frame_id = "camera_init";
            pubLaserCloudMap_->publish(msg);
        }
    }
}

void LaserMappingNode::map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    (void)req;
    RCLCPP_INFO(this->get_logger(), "Saving map...");
    fast_lio_core_->save_pcd();
    res->success = true;
    res->message = "Map saved.";
}
