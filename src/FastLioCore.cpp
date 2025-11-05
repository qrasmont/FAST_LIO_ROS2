#include "fast_lio/FastLioCore.h"
#include "fast_lio/common_lib.h"
#include <algorithm>

// Global variables for local map management
BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;

// Helper function to convert point from body to world frame
void pointBodyToWorld(PointTypeNorm const * const pi, PointTypeNorm * const po, const state_ikfom& s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

// Helper function to convert point from Lidar body to IMU body frame
void RGBpointBodyLidarToIMU(PointTypeNorm const * const pi, PointTypeNorm * const po, const state_ikfom& s)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(s.offset_R_L_I*p_body_lidar + s.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

// Function to dump LIO state to a log file for debugging
void dump_lio_state_to_log(FILE *fp, const state_ikfom& s, double lidar_beg_time, double first_lidar_time)
{
    V3D rot_ang(Log(s.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2)); // Angle
    fprintf(fp, "%lf %lf %lf ", s.pos(0), s.pos(1), s.pos(2));       // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                      // omega (placeholder)
    fprintf(fp, "%lf %lf %lf ", s.vel(0), s.vel(1), s.vel(2));       // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                      // Acc (placeholder)
    fprintf(fp, "%lf %lf %lf ", s.bg(0), s.bg(1), s.bg(2));          // Bias_g
    fprintf(fp, "%lf %lf %lf ", s.ba(0), s.ba(1), s.ba(2));          // Bias_a
    fprintf(fp, "%lf %lf %lf ", s.grav[0], s.grav[1], s.grav[2]);    // Gravity
    fprintf(fp, "\r\n");
    fflush(fp);
}

FastLioCore::FastLioCore(const FastLioConfig& config)
    : config_(config),
      p_pre_(new Preprocess()),
      feats_undistort_(new PointCloudXYZI()),
      feats_down_body_(new PointCloudXYZI()),
      feats_down_world_(new PointCloudXYZI()),
      normvec_(new PointCloudXYZI(100000, 1)),
      laserCloudOri_(new PointCloudXYZI(100000, 1)),
      corr_normvect_(new PointCloudXYZI(100000, 1)),
      pcl_wait_save_(new PointCloudXYZI()),
      point_selected_surf_(100000, false),
      res_last_(100000, -1000.0f),
      global_map_cloud_(new PointCloudXYZI())
{
    p_imu_ = std::make_shared<ImuProcess>(config_.log_path);
    extrinsic_est_en_ = config_.extrinsic_est_en;

    // Set up preprocess object
    p_pre_->set(config_.feature_extract_enable, config_.lidar_type, config_.blind, config_.point_filter_num);
    p_pre_->N_SCANS = config_.scan_line;
    p_pre_->time_unit = config_.timestamp_unit;
    p_pre_->SCAN_RATE = config_.scan_rate;

    // Set up IMU processor
    V3D Lidar_T_wrt_IMU(config.extrinsic_T.data());
    M3D Lidar_R_wrt_IMU = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(config.extrinsic_R.data());
    p_imu_->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu_->set_gyr_cov(V3D(config_.gyr_cov, config_.gyr_cov, config_.gyr_cov));
    p_imu_->set_acc_cov(V3D(config_.acc_cov, config_.acc_cov, config_.acc_cov));
    p_imu_->set_gyr_bias_cov(V3D(config_.b_gyr_cov, config_.b_gyr_cov, config_.b_gyr_cov));
    p_imu_->set_acc_bias_cov(V3D(config_.b_acc_cov, config_.b_acc_cov, config_.b_acc_cov));

    // Set up EKF
    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    auto h_model = std::bind(&FastLioCore::h_share_model, this, std::placeholders::_1, std::placeholders::_2);
    kf_.init_dyn_share(get_f, df_dx, df_dw, h_model, config_.max_iteration, epsi);

    // Set up voxel filters
    downSizeFilterSurf_.setLeafSize(config_.filter_size_surf, config_.filter_size_surf, config_.filter_size_surf);
    downSizeFilterMap_.setLeafSize(config_.filter_size_map, config_.filter_size_map, config_.filter_size_map);

    // Other initializations
    cube_len_ = config_.cube_side_length;
    FOV_DEG_ = (config_.fov_degree + 10.0) > 179.9 ? 179.9 : (config_.fov_degree + 10.0);
    HALF_FOV_COS_ = cos((FOV_DEG_) * 0.5 * PI_M / 180.0);
    NUM_MAX_ITERATIONS_ = config_.max_iteration;
    pcd_save_interval_ = config_.pcd_save_interval;

    // Debug log
    if (config_.runtime_pos_log_enable) {
        string pos_log_dir = DEBUG_FILE_DIR(config_.log_path, "pos_log.txt");
        fp_log_ = fopen(pos_log_dir.c_str(),"w");
    }

    pruning_thread_ = std::thread(&FastLioCore::pruning_thread_main, this);
}

FastLioCore::~FastLioCore()
{
    {
        std::lock_guard<std::mutex> lock(mtx_map_pruning_);
        flg_exit_.store(true, std::memory_order_release);
        sig_map_pruning_.notify_one();
    }

    if (pruning_thread_.joinable())
    {
        pruning_thread_.join();
    }

    if (config_.pcd_save_en && pcl_wait_save_->size() > 0) {
        save_pcd();
    }
    if (fp_log_) {
        fclose(fp_log_);
    }
}

void FastLioCore::initial_setup()
{
    std::unique_lock<std::mutex> lock(mtx_buffer_);
    sig_buffer_.wait(lock, [&] {
        return flg_exit_ || imu_buffer_.size() >= INIT_IMU_COUNT;
    });

    if (flg_exit_) return;

    p_imu_->IMU_init(imu_buffer_, kf_);
    flg_is_system_initialized_ = true;
    std::cout << "System initialized." << std::endl;
}

void FastLioCore::process()
{
    if (!flg_is_system_initialized_) {
        return;
    }

    MeasureGroup meas;
    if (sync_packages(meas)) {
        process_frame(meas);
    }
}

bool FastLioCore::sync_packages(MeasureGroup &meas)
{
    std::lock_guard<std::mutex> lock(mtx_buffer_);
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    if (lidar_buffer_.size() < 2) {
        return false;
    }

    const double period = 1.0 / std::max(1, config_.scan_rate);
    const double MAX_LAG = std::clamp(3.0 * period, 0.1, 0.7);
    int drops = 0;
    const int MAX_DROPS_PER_SYNC = 4;

    auto backlog_seconds = [&]() -> double {
        double t_front  = time_buffer_.front();
        double t_latest = !imu_buffer_.empty() ? get_time_sec(imu_buffer_.back()->header.stamp)
                                               : time_buffer_.back();
        return t_latest - t_front;
    };

    while (lidar_buffer_.size() >= 2 && backlog_seconds() > MAX_LAG && drops < MAX_DROPS_PER_SYNC)
    {
        // Drop the oldest LiDAR scan and IMU samples
	RCLCPP_INFO(rclcpp::get_logger("fast_lio_core"), "Dropping old scans");

        const double old_first = time_buffer_.front();
        const double old_second = time_buffer_.at(1);

        while (!imu_buffer_.empty() &&
               get_time_sec(imu_buffer_.front()->header.stamp) < old_second)
        {
            imu_buffer_.pop_front();
        }
        lidar_buffer_.pop_front();
        time_buffer_.pop_front();
        ++drops;
    }

    // If we dropped too much wait for more data
    if (lidar_buffer_.size() < 2) {
        return false;
    }

    double first_scan_start_time  = time_buffer_.front();
    double second_scan_start_time = time_buffer_.at(1);

    auto it_imu = imu_buffer_.begin();
    while (it_imu != imu_buffer_.end()) {
        if (get_time_sec((*it_imu)->header.stamp) >= second_scan_start_time) {
            break;
        }
        it_imu++;
    }

    if (it_imu == imu_buffer_.end()) {
        return false;
    }

    meas.lidar = lidar_buffer_.front();
    meas.lidar_beg_time = first_scan_start_time;

    lidar_end_time_ = second_scan_start_time;
    meas.lidar_end_time = lidar_end_time_;

    meas.imu.clear();
    while (!imu_buffer_.empty() && get_time_sec(imu_buffer_.front()->header.stamp) < second_scan_start_time)
    {
        meas.imu.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();

    return true;
}

FrameResult FastLioCore::process_frame(const MeasureGroup& meas)
{
    if (map_pruning_finished_.load(std::memory_order_acquire))
    {
        PointVector points_history;
        {
            std::shared_lock<std::shared_mutex> rlk(mtx_ikdtree_);
            ikdtree_.acquire_removed_points(points_history);
        }
        map_pruning_finished_.store(false, std::memory_order_release);
        is_map_pruning_.store(false, std::memory_order_release);
    }

    if (flg_first_scan_)
    {
        first_lidar_time_ = meas.lidar_beg_time;
        p_imu_->first_lidar_time = first_lidar_time_;
        flg_first_scan_ = false;
    }

    p_imu_->Process(meas, kf_, feats_undistort_);
    state_point_ = kf_.get_x();
    pos_lid_ = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;

    if (feats_undistort_->empty())
    {
        std::cerr << "No point, skip this scan!\n" << std::endl;
        return FrameResult();
    }

    lasermap_fov_segment();

    downSizeFilterSurf_.setInputCloud(feats_undistort_);
    downSizeFilterSurf_.filter(*feats_down_body_);
    feats_down_size_ = feats_down_body_->points.size();

    if(ikdtree_.Root_Node == nullptr)
    {
        if(feats_down_size_ > 5)
        {
            ikdtree_.set_downsample_param(config_.filter_size_map);
            feats_down_world_->resize(feats_down_size_);
            for(int i = 0; i < feats_down_size_; i++)
            {
                pointBodyToWorld(&(feats_down_body_->points[i]), &(feats_down_world_->points[i]), state_point_);
            }
            std::lock_guard<std::shared_mutex> lock(mtx_ikdtree_);
            ikdtree_.Build(feats_down_world_->points);
        }
        return FrameResult();
    }

    if (feats_down_size_ < 5)
    {
        std::cerr << "No point, skip this scan!\n" << std::endl;
        return FrameResult();
    }

    normvec_->resize(feats_down_size_);
    feats_down_world_->resize(feats_down_size_);

    pointSearchInd_surf_.resize(feats_down_size_);
    Nearest_Points_.resize(feats_down_size_);

    double solve_H_time = 0;
    kf_.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
    state_point_ = kf_.get_x();
    pos_lid_ = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;

    map_incremental();

    FrameResult result;
    result.pose = Eigen::Matrix4d::Identity();
    result.pose.block<3, 3>(0, 0) = state_point_.rot.toRotationMatrix();
    result.pose.block<3, 1>(0, 3) = pos_lid_;
    pcl::copyPointCloud(*feats_down_body_, result.cloud);

    if (config_.runtime_pos_log_enable) {
        dump_lio_state_to_log(fp_log_, state_point_, meas.lidar_beg_time, first_lidar_time_);
    }

    return result;
}

void FastLioCore::lasermap_fov_segment()
{
    if (is_map_pruning_.load(std::memory_order_acquire))
    {
        return;
    }

    cub_needrm.clear();
    kdtree_delete_counter_ = 0;
    kdtree_delete_time_ = 0.0;

    PointTypeNorm p_body;
    p_body.x = LIDAR_SP_LEN; p_body.y = 0.0; p_body.z = 0.0;
    PointTypeNorm p_world;
    pointBodyToWorld(&p_body, &p_world, state_point_);

    V3D pos_LiD = pos_lid_;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len_ / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len_ / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * config_.det_range || dist_to_map_edge[i][1] <= MOV_THRESHOLD * config_.det_range) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len_ - 2.0 * MOV_THRESHOLD * config_.det_range) * 0.5 * 0.9, double(config_.det_range * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * config_.det_range){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * config_.det_range){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }

    // Defer the state update
    pending_new_local_map_ = New_LocalMap_Points;
    is_map_pruning_.store(true, std::memory_order_release);

    // Send the deletion task to the worker thread
    if (!cub_needrm.empty())
    {
        std::lock_guard<std::mutex> lock(mtx_map_pruning_);
        if (cub_to_rm_.size() > kMaxPruneQueue_)
        {
            cub_to_rm_.erase(cub_to_rm_.begin(),
                             cub_to_rm_.begin() + (cub_to_rm_.size() - kMaxPruneQueue_));
        }

        for (const auto& box : cub_needrm)
        {
            cub_to_rm_.push_back(box);
        }

        sig_map_pruning_.notify_one();
    }
}

void FastLioCore::pruning_thread_main()
{
    RCLCPP_INFO(rclcpp::get_logger("fast_lio_core"), "Pruning thread started.");

    while (!flg_exit_.load(std::memory_order_acquire))
    {
        std::deque<BoxPointType> current_cub_to_rm;
        {
            std::unique_lock<std::mutex> lock(mtx_map_pruning_);
            sig_map_pruning_.wait(lock, [this]{
                return flg_exit_.load(std::memory_order_acquire) || !cub_to_rm_.empty();
            });

            if (flg_exit_.load(std::memory_order_acquire))
            {
                break;
            }

            current_cub_to_rm.swap(cub_to_rm_);
        }

        if (!current_cub_to_rm.empty())
        {
            std::vector<BoxPointType> boxes_to_delete(current_cub_to_rm.begin(), current_cub_to_rm.end());
            coalesce_boxes(boxes_to_delete);
            // try to take the writer lock; if busy, requeue and retry later
            bool deleted = false;
            {
                std::unique_lock<std::shared_mutex> wlk(mtx_ikdtree_, std::try_to_lock);
                if (wlk.owns_lock())
                {
                    const double t0 = omp_get_wtime();
                    kdtree_delete_counter_ = ikdtree_.Delete_Point_Boxes(boxes_to_delete);
                    kdtree_delete_time_ = omp_get_wtime() - t0;
                    {
                        std::lock_guard<std::mutex> lck(map_mtx_);
                        LocalMap_Points = pending_new_local_map_;
                    }
                    deleted = true;
                }
            }

            if (deleted)
            {
                map_pruning_finished_.store(true, std::memory_order_release);
            }
            else
            {
                // couldn't get the writer lock; push back and try later
                std::lock_guard<std::mutex> lock(mtx_map_pruning_);
                for (auto it = boxes_to_delete.rbegin(); it != boxes_to_delete.rend(); ++it)
                {
                    cub_to_rm_.push_front(*it);
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("fast_lio_core"), "Pruning thread stopped.");
}

void FastLioCore::map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size_);
    PointNoNeedDownsample.reserve(feats_down_size_);
    for (int i = 0; i < feats_down_size_; i++)
    {
        pointBodyToWorld(&(feats_down_body_->points[i]), &(feats_down_world_->points[i]), state_point_);
        if (!Nearest_Points_[i].empty() && flg_is_system_initialized_)
        {
            const PointVector &points_near = Nearest_Points_[i];
            bool need_add = true;
            PointTypeNorm mid_point; 
            mid_point.x = floor(feats_down_world_->points[i].x/config_.filter_size_map)*config_.filter_size_map + 0.5 * config_.filter_size_map;
            mid_point.y = floor(feats_down_world_->points[i].y/config_.filter_size_map)*config_.filter_size_map + 0.5 * config_.filter_size_map;
            mid_point.z = floor(feats_down_world_->points[i].z/config_.filter_size_map)*config_.filter_size_map + 0.5 * config_.filter_size_map;
            float dist  = calc_dist(feats_down_world_->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * config_.filter_size_map && fabs(points_near[0].y - mid_point.y) > 0.5 * config_.filter_size_map && fabs(points_near[0].z - mid_point.z) > 0.5 * config_.filter_size_map){
                PointNoNeedDownsample.push_back(feats_down_world_->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world_->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world_->points[i]);
        }
    }

    {
        std::lock_guard<std::shared_mutex> lock(mtx_ikdtree_);
        ikdtree_.Add_Points(PointToAdd, true);
        ikdtree_.Add_Points(PointNoNeedDownsample, false);
    }

    if (config_.map_pub_en) {
        std::lock_guard<std::mutex> lock(map_mtx_);
        for(const auto& pt : PointToAdd) {
            global_map_cloud_->push_back(pt);
        }
        for(const auto& pt : PointNoNeedDownsample) {
            global_map_cloud_->push_back(pt);
        }
    }
}

void FastLioCore::save_pcd()
{
    if (pcl_wait_save_->size() > 0)
    {
        pcl::PCDWriter pcd_writer;
        std::string file_name = config_.pcd_save_path + "map.pcd";
        pcd_writer.writeBinary(file_name, *pcl_wait_save_);
        std::cout << "Map saved to " << file_name << std::endl;
    }
}

double FastLioCore::get_lidar_end_time() const {
    return lidar_end_time_;
}

bool FastLioCore::get_publish_odometry(nav_msgs::msg::Odometry& odom, geometry_msgs::msg::Quaternion& quat)
{
    odom.header.frame_id = "camera_init";
    odom.child_frame_id = "body";

    quat.x = state_point_.rot.coeffs()[0];
    quat.y = state_point_.rot.coeffs()[1];
    quat.z = state_point_.rot.coeffs()[2];
    quat.w = state_point_.rot.coeffs()[3];

    odom.pose.pose.position.x = state_point_.pos(0);
    odom.pose.pose.position.y = state_point_.pos(1);
    odom.pose.pose.position.z = state_point_.pos(2);
    odom.pose.pose.orientation = quat;

    auto P = kf_.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odom.pose.covariance[i*6 + 0] = P(k, 3);
        odom.pose.covariance[i*6 + 1] = P(k, 4);
        odom.pose.covariance[i*6 + 2] = P(k, 5);
        odom.pose.covariance[i*6 + 3] = P(k, 0);
        odom.pose.covariance[i*6 + 4] = P(k, 1);
        odom.pose.covariance[i*6 + 5] = P(k, 2);
    }
    return true;
}

bool FastLioCore::get_publish_path(nav_msgs::msg::Path& path, nav_msgs::msg::Odometry const & odom)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    path.poses.push_back(pose);
    return true;
}

bool FastLioCore::get_publish_frame_world(PointCloudXYZI::Ptr& cloud)
{
    PointCloudXYZI::Ptr cloud_to_publish = config_.dense_publish_en ? feats_undistort_ : feats_down_body_;

    int size = cloud_to_publish->points.size();
    if (size == 0) return false;

    cloud.reset(new PointCloudXYZI(size, 1));
    for (int i = 0; i < size; i++)
    {
        pointBodyToWorld(&cloud_to_publish->points[i], &cloud->points[i], state_point_);
    }
    return true;
}

bool FastLioCore::get_publish_frame_body(PointCloudXYZI::Ptr& cloud)
{
    int size = feats_undistort_->points.size();
    if (size == 0) return false;

    cloud.reset(new PointCloudXYZI(size, 1));
    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort_->points[i], &cloud->points[i], state_point_);
    }
    return true;
}

bool FastLioCore::get_publish_effect_world(PointCloudXYZI::Ptr& cloud)
{
    if (effct_feat_num_ == 0) return false;

    cloud.reset(new PointCloudXYZI(effct_feat_num_, 1));
    for (int i = 0; i < effct_feat_num_; i++)
    {
        pointBodyToWorld(&laserCloudOri_->points[i], &cloud->points[i], state_point_);
    }
    return true;
}

bool FastLioCore::get_publish_map(PointCloudXYZI::Ptr& cloud)
{
    if (!config_.map_pub_en || global_map_cloud_->empty()) {
        return false;
    }

    PointCloudXYZI::Ptr map_to_publish(new PointCloudXYZI());

    {
        std::lock_guard<std::mutex> lock(map_mtx_);
        *map_to_publish = *global_map_cloud_;
    }

    cloud.reset(new PointCloudXYZI());

    downSizeFilterMap_.setInputCloud(map_to_publish);
    downSizeFilterMap_.filter(*cloud);

    return true;
}

void FastLioCore::h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri_->clear();
    corr_normvect_->clear();

    {
        std::shared_lock<std::shared_mutex> rlk(mtx_ikdtree_);

        #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
        #endif
        for (int i = 0; i < feats_down_size_; i++)
        {
            PointTypeNorm &point_body  = feats_down_body_->points[i]; 
            PointTypeNorm &point_world = feats_down_world_->points[i]; 

            V3D p_body(point_body.x, point_body.y, point_body.z);
            V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
            point_world.x = p_global(0);
            point_world.y = p_global(1);
            point_world.z = p_global(2);
            point_world.intensity = point_body.intensity;

            vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

            auto &points_near = Nearest_Points_[i];

            if (ekfom_data.converge)
            {
                ikdtree_.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
                point_selected_surf_[i] =
                    (points_near.size() < NUM_MATCH_POINTS) ? false
                    : (pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5) ? false : true;
            }

            if (!point_selected_surf_[i]) continue;

            VF(4) pabcd;
            point_selected_surf_[i] = false;
            if (esti_plane(pabcd, points_near, 0.1f))
            {
                float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
                float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

                if (s > 0.9)
                {
                    point_selected_surf_[i] = true;
                    normvec_->points[i].x = pabcd(0);
                    normvec_->points[i].y = pabcd(1);
                    normvec_->points[i].z = pabcd(2);
                    normvec_->points[i].intensity = pd2;
                    res_last_[i] = abs(pd2);
                }
            }
        }
    }

    effct_feat_num_ = 0;

    for (int i = 0; i < feats_down_size_; i++)
    {
        if (point_selected_surf_[i])
        {
            laserCloudOri_->points[effct_feat_num_] = feats_down_body_->points[i];
            corr_normvect_->points[effct_feat_num_] = normvec_->points[i];
            effct_feat_num_++;
        }
    }

    if (effct_feat_num_ < 1)
    {
        ekfom_data.valid = false;
        std::cerr << "No Effective Points!" << std::endl;
        return;
    }

    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num_, 12);
    ekfom_data.h.resize(effct_feat_num_);

    for (int i = 0; i < effct_feat_num_; i++)
    {
        const PointTypeNorm &laser_p  = laserCloudOri_->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        const PointTypeNorm &norm_p = corr_normvect_->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en_)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        ekfom_data.h(i) = -norm_p.intensity;
    }
}


static inline void expand_box(BoxPointType& acc, const BoxPointType& b)
{
    for (int d = 0; d < 3; ++d)
    {
        acc.vertex_min[d] = std::min(acc.vertex_min[d], b.vertex_min[d]);
        acc.vertex_max[d] = std::max(acc.vertex_max[d], b.vertex_max[d]);
    }
}

void FastLioCore::coalesce_boxes(std::vector<BoxPointType>& boxes) const
{
    if (boxes.size() <= 8)
    {
        // Leave small batches alone
        return;
    }

    const size_t n   = boxes.size();
    const size_t mid = n / 2;

    BoxPointType A = boxes[0];
    for (size_t i = 1; i < mid; ++i) {
        expand_box(A, boxes[i]);
    }

    BoxPointType B = boxes[mid];
    for (size_t i = mid + 1; i < n; ++i) {
        expand_box(B, boxes[i]);
    }

    // Replace input with up to two merged boxes
    boxes.clear();
    boxes.push_back(A);
    boxes.push_back(B);
}

void FastLioCore::accumulate_map_points()
{
    if (config_.pcd_save_en || config_.map_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(config_.dense_publish_en ? feats_undistort_ : feats_down_body_);
        int size = laserCloudFullRes->points.size();
        if (size == 0) return;

        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            pointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i], state_point_);
        }
        *pcl_wait_save_ += *laserCloudWorld;
    }
}
