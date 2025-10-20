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
#include "preprocess.h"
#include "ikd-Tree/ikd_Tree.h"
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <condition_variable>
#include <deque>
#include <atomic>


#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)
#define INIT_IMU_COUNT      (200)
#define MOV_THRESHOLD       (1.5f)
#define NUM_MATCH_POINTS    (5)
#define LIDAR_SP_LEN        (2)

// Configuration structure for FastLioCore
struct FastLioConfig {
    bool feature_extract_enable;
    int point_filter_num;
    int max_iteration;
    double filter_size_surf;
    double filter_size_map;
    double cube_side_length;
    bool runtime_pos_log_enable;
    std::string map_file_path;
    bool time_sync_en;
    double time_offset_lidar_to_imu;
    int lidar_type;
    int scan_line;
    double blind;
    int timestamp_unit;
    int scan_rate;
    double acc_cov;
    double gyr_cov;
    double b_acc_cov;
    double b_gyr_cov;
    double fov_degree;
    double det_range;
    bool extrinsic_est_en;
    std::vector<double> extrinsic_T;
    std::vector<double> extrinsic_R;
    bool pcd_save_en;
    std::string pcd_save_path;
    int pcd_save_interval;
    std::string log_path;
    bool dense_publish_en;
    bool map_pub_en;
};

struct StampedMessage {
    double timestamp;
    sensor_msgs::msg::Imu::ConstSharedPtr imu_msg;
    PointCloudXYZI::Ptr lidar_msg;
    tf2_msgs::msg::TFMessage::ConstSharedPtr tf_msg;
    std::string topic_name;

    bool operator<(const StampedMessage& other) const {
        return timestamp < other.timestamp;
    }
};

class FastLioCore {
public:
    FastLioCore(const FastLioConfig& config);
    ~FastLioCore();

    void process();
    void initial_setup();
    bool sync_packages(MeasureGroup &meas);
    FrameResult process_frame(const MeasureGroup& meas);
    void map_incremental();
    void lasermap_fov_segment();
    void save_pcd();
    void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);
    void accumulate_map_points();


    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer_;
    std::deque<PointCloudXYZI::Ptr> lidar_buffer_;
    std::deque<double> time_buffer_;
    std::mutex mtx_buffer_;
    std::condition_variable sig_buffer_;
    std::atomic<bool> flg_exit_ = false;

    bool get_publish_odometry(nav_msgs::msg::Odometry& odom, geometry_msgs::msg::Quaternion& quat);
    bool get_publish_path(nav_msgs::msg::Path& path, nav_msgs::msg::Odometry const & odom);
    bool get_publish_frame_world(PointCloudXYZI::Ptr& cloud);
    bool get_publish_frame_body(PointCloudXYZI::Ptr& cloud);
    bool get_publish_effect_world(PointCloudXYZI::Ptr& cloud);
    bool get_publish_map(PointCloudXYZI::Ptr& cloud);
    double get_lidar_end_time() const;

    PointCloudXYZI::Ptr laserCloudOri_;
    PointCloudXYZI::Ptr corr_normvect_;
    int feats_down_size_ = 0;
    PointCloudXYZI::Ptr feats_down_body_;
    PointCloudXYZI::Ptr feats_down_world_;
    std::vector<PointVector> Nearest_Points_;
    KD_TREE<PointTypeNorm> ikdtree_;
    std::vector<bool> point_selected_surf_;
    PointCloudXYZI::Ptr normvec_;
    std::vector<float> res_last_;
    int effct_feat_num_ = 0;
    bool extrinsic_est_en_;
    std::shared_ptr<Preprocess> p_pre_;
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;
    std::shared_ptr<ImuProcess> p_imu_;

    bool flg_is_system_initialized_ = false;

private:
    FastLioConfig config_;

    std::shared_mutex mtx_ikdtree_;
    std::mutex map_mtx_;
    PointCloudXYZI::Ptr global_map_cloud_;

    bool flg_first_scan_ = true;

    double lidar_end_time_ = 0;
    double first_lidar_time_ = 0;
    double last_timestamp_lidar_ = 0;
    double last_timestamp_imu_ = -1.0;

    state_ikfom state_point_;
    vect3 pos_lid_;

    PointCloudXYZI::Ptr feats_undistort_;
    PointCloudXYZI::Ptr pcl_wait_save_;

    pcl::VoxelGrid<PointTypeNorm> downSizeFilterSurf_;
    pcl::VoxelGrid<PointTypeNorm> downSizeFilterMap_;

    std::vector<std::vector<int>> pointSearchInd_surf_;
    std::vector<BoxPointType> cub_needrm;

    int pcd_index_ = 0;
    int scan_count_ = 0;

    FILE *fp_log_ = nullptr;
    std::ofstream fout_pre_, fout_out_, fout_dbg_;

    double cube_len_;
    double FOV_DEG_;
    double HALF_FOV_COS_;
    int NUM_MAX_ITERATIONS_;
    int pcd_save_interval_;
    int kdtree_delete_counter_ = 0;
    double kdtree_delete_time_ = 0.0;

    void pruning_thread_main();

    std::atomic<bool> is_map_pruning_ = false;
    std::atomic<bool> map_pruning_finished_ = false;
    BoxPointType pending_new_local_map_;

    std::thread pruning_thread_;
    std::mutex mtx_map_pruning_;
    std::condition_variable sig_map_pruning_;
    std::deque<BoxPointType> cub_to_rm_;
    static constexpr size_t kMaxPruneQueue_ = 8;

    void coalesce_boxes(std::vector<BoxPointType>& boxes) const;

};
