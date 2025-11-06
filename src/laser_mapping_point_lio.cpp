#include "d_point_lio/laser_mapping.h"
#include "d_point_lio/preprocess.h"
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Eigen>
#include <cmath>
#include <signal.h>
#include <algorithm>

LaserMapping::LaserMapping() {
    // Initialize ESEKF
    InitializeState();

    // Initialize point clouds
    feats_undistort_.reset(new PointCloudXYZI());
    feats_down_body_.reset(new PointCloudXYZI());
    feats_down_world_.reset(new PointCloudXYZI());
    init_feats_world_.reset(new PointCloudXYZI());

    // Set up transformation matrices
    Lidar_R_wrt_IMU_ = Matrix3d::Identity();
    Lidar_T_wrt_IMU_ = Vector3d::Zero();

    LOG(INFO) << "LaserMapping constructor completed";
}

LaserMapping::~LaserMapping() {
    Finish();
    LOG(INFO) << "LaserMapping destructor completed";
}

bool LaserMapping::InitROS(ros::NodeHandle &nh) {
    LOG(INFO) << "Initializing LaserMapping with Point-LIO logic...";

    if (!LoadParams(nh)) {
        LOG(ERROR) << "Failed to load parameters";
        return false;
    }

    SubAndPubToROS(nh);

    running_ = true;
    process_thread_ = std::thread(&LaserMapping::ProcessLoop, this);

    LOG(INFO) << "LaserMapping initialized successfully with use_imu_as_input=" << config_.use_imu_as_input;
    return true;
}

void LaserMapping::Run() {
    // Main processing loop is handled by ProcessLoop thread
    LOG(INFO) << "LaserMapping running with Point-LIO algorithm";
}

void LaserMapping::Finish() {
    if (running_) {
        running_ = false;
        flg_exit_ = true;
        sig_buffer_.notify_all();
        if (process_thread_.joinable()) {
            process_thread_.join();
        }
    }
}

bool LaserMapping::LoadParams(ros::NodeHandle &nh) {
    // Load Point-LIO specific parameters
    nh.getParam("use_imu_as_input", config_.use_imu_as_input);
    nh.getParam("prop_at_freq_of_imu", config_.prop_at_freq_of_imu);
    nh.getParam("check_satu", config_.check_satu);
    nh.getParam("space_down_sample", config_.space_down_sample);
    nh.getParam("publish_odometry_without_downsample", config_.publish_odometry_without_downsample);
    nh.getParam("imu_en", config_.imu_en);
    nh.getParam("runtime_pos_log_enable", config_.runtime_pos_log);
    nh.getParam("mapping/extrinsic_est_en", config_.extrinsic_est_en);
    nh.getParam("con_frame_num", config_.con_frame_num);
    nh.getParam("cut_frame_time_interval", config_.cut_frame_time_interval);
    nh.getParam("common/time_diff_lidar_to_imu", config_.time_diff_lidar_to_imu);

    // Load noise parameters
    nh.getParam("mapping/acc_cov_input", config_.acc_cov_input);
    nh.getParam("mapping/gyr_cov_input", config_.gyr_cov_input);
    nh.getParam("mapping/acc_cov_output", config_.acc_cov_output);
    nh.getParam("mapping/gyr_cov_output", config_.gyr_cov_output);
    nh.getParam("mapping/vel_cov", config_.vel_cov);
    nh.getParam("mapping/b_gyr_cov", config_.b_gyr_cov);
    nh.getParam("mapping/b_acc_cov", config_.b_acc_cov);
    nh.getParam("mapping/imu_meas_acc_cov", config_.imu_meas_acc_cov);
    nh.getParam("mapping/imu_meas_omg_cov", config_.imu_meas_omg_cov);
    nh.getParam("mapping/lidar_meas_cov", config_.laser_point_cov);
    nh.getParam("mapping/imu_time_inte", config_.imu_time_inte);
    nh.getParam("mapping/lidar_time_inte", config_.lidar_time_inte);
    nh.getParam("mapping/satu_acc", config_.satu_acc);
    nh.getParam("mapping/satu_gyro", config_.satu_gyro);
    nh.getParam("mapping/acc_norm", config_.acc_norm);

    // Load mapping parameters
    nh.getParam("mapping/plane_thr", config_.plane_thr);
    nh.getParam("mapping/match_s", config_.match_s);
    nh.getParam("init_map_size", config_.init_map_size);

    // Load gravity settings
    nh.getParam("mapping/gravity", config_.gravity);
    nh.getParam("mapping/gravity_init", config_.gravity_init);

    // Load common parameters
    nh.getParam("common/lid_topic", config_.lidar_topic);
    nh.getParam("common/imu_topic", config_.imu_topic);
    nh.getParam("common/time_sync_en", config_.time_sync_en);
    nh.getParam("common/con_frame", config_.con_frame);
    nh.getParam("common/cut_frame", config_.cut_frame);

    // Load preprocess parameters
    nh.getParam("preprocess/lidar_type", config_.lidar_type);
    nh.getParam("preprocess/scan_line", config_.scan_line);
    nh.getParam("preprocess/blind", config_.blind);
    nh.getParam("preprocess/time_scale", config_.time_scale);

    // Load filter parameters
    nh.getParam("point_filter_num", config_.point_filter_num);
    nh.getParam("filter_size_surf", config_.filter_size_surf);
    nh.getParam("filter_size_map", config_.filter_size_map);
    nh.getParam("max_iteration", config_.max_iteration);
    nh.getParam("cube_side_length", config_.cube_side_length);

    // Load publish parameters
    nh.getParam("publish/path_publish_en", config_.path_publish_en);
    nh.getParam("publish/scan_publish_en", config_.scan_publish_en);
    nh.getParam("publish/scan_effect_pub_en", config_.scan_effect_pub_en);
    nh.getParam("publish/dense_publish_en", config_.dense_publish_en);
    nh.getParam("publish/scan_bodyframe_pub_en", config_.scan_bodyframe_pub_en);
    nh.getParam("path_save_en", config_.pcd_save_en);
    nh.getParam("pcd_save/interval", config_.pcd_save_interval);

    // Setup extrinsic transformation
    if (config_.extrinsic_T.size() >= 3 && config_.extrinsic_R.size() >= 9) {
        Lidar_T_wrt_IMU_ = Vector3d(config_.extrinsic_T[0], config_.extrinsic_T[1], config_.extrinsic_T[2]);
        Lidar_R_wrt_IMU_ = Matrix3d(config_.extrinsic_R.data());
    }

    LOG(INFO) << "Point-LIO parameters loaded successfully";
    LOG(INFO) << "use_imu_as_input: " << config_.use_imu_as_input;
    LOG(INFO) << "LiDAR topic: " << config_.lidar_topic;
    LOG(INFO) << "IMU topic: " << config_.imu_topic;
    LOG(INFO) << "LiDAR type: " << config_.lidar_type;

    return true;
}

void LaserMapping::SubAndPubToROS(ros::NodeHandle &nh) {
    // Subscribers based on LiDAR type
    if (config_.lidar_type == 1) {  // Livox
        sub_pcl_ = nh.subscribe(config_.lidar_topic, 200000,
                                &LaserMapping::LivoxPCLCallBack, this);
    } else {  // Standard PCL
        sub_pcl_std_ = nh.subscribe(config_.lidar_topic, 200000,
                                   &LaserMapping::StandardPCLCallBack, this);
    }

    // IMU subscriber
    sub_imu_ = nh.subscribe(config_.imu_topic, 200000,
                           &LaserMapping::IMUCallBack, this);

    // Publishers
    pub_path_ = nh.advertise<nav_msgs::Path>("path", 100);
    pub_global_map_ = nh.advertise<sensor_msgs::PointCloud2>("cloud_registered", 100);
    pub_laser_odometry_ = nh.advertise<nav_msgs::Odometry>("odom", 100);
    pub_laser_effect_ = nh.advertise<sensor_msgs::PointCloud2>("cloud_effected", 100);
    pub_laser_body_ = nh.advertise<sensor_msgs::PointCloud2>("cloud_body", 100);

    LOG(INFO) << "ROS subscribers and publishers initialized for Point-LIO";
}

void LaserMapping::StandardPCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(mtx_buffer_);

    // Point-LIO style callback
    scan_count_++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar_) {
        LOG(ERROR) << "LiDAR loop back, clear buffer";
        return;
    }

    last_timestamp_lidar_ = msg->header.stamp.toSec();

    // Process point cloud
    PointCloudXYZI::Ptr processed_cloud(new PointCloudXYZI());
    Preprocess preprocessor;
    preprocessor.point_filter_num = config_.point_filter_num;
    preprocessor.blind = config_.blind;
    preprocessor.process(msg, processed_cloud);

    if (processed_cloud->points.size() > 0) {
        lidar_buffer_.push_back(processed_cloud);
        time_buffer_.push_back(msg->header.stamp.toSec());
    }

    // Limit buffer size
    if (lidar_buffer_.size() > 1000) {
        lidar_buffer_.pop_front();
        time_buffer_.pop_front();
    }

    sig_buffer_.notify_one();
}

void LaserMapping::LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(mtx_buffer_);

    // Point-LIO style callback
    scan_count_++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar_) {
        LOG(ERROR) << "LiDAR loop back, clear buffer";
        return;
    }

    last_timestamp_lidar_ = msg->header.stamp.toSec();

    // Process Livox point cloud
    PointCloudXYZI::Ptr processed_cloud(new PointCloudXYZI());
    Preprocess preprocessor;
    preprocessor.point_filter_num = config_.point_filter_num;
    preprocessor.blind = config_.blind;
    preprocessor.process(msg, processed_cloud);

    if (processed_cloud->points.size() > 0) {
        lidar_buffer_.push_back(processed_cloud);
        time_buffer_.push_back(msg->header.stamp.toSec());
    }

    // Limit buffer size
    if (lidar_buffer_.size() > 1000) {
        lidar_buffer_.pop_front();
        time_buffer_.pop_front();
    }

    sig_buffer_.notify_one();
}

void LaserMapping::IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in) {
    std::lock_guard<std::mutex> lock(mtx_buffer_);

    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    // Apply time offset compensation
    msg->header.stamp = ros::Time().fromSec(msg->header.stamp.toSec() - config_.time_diff_lidar_to_imu);

    double timestamp = msg->header.stamp.toSec();

    if (timestamp < last_timestamp_imu_) {
        LOG(ERROR) << "IMU loop back, clear deque";
        return;
    }

    last_timestamp_imu_ = timestamp;

    // Check for saturation
    if (config_.check_satu) {
        double acc_norm = std::sqrt(msg->linear_acceleration.x * msg->linear_acceleration.x +
                                    msg->linear_acceleration.y * msg->linear_acceleration.y +
                                    msg->linear_acceleration.z * msg->linear_acceleration.z);
        if (acc_norm > config_.satu_acc) {
            LOG(WARNING) << "Accelerometer saturation detected: " << acc_norm;
        }

        double gyro_norm = std::sqrt(msg->angular_velocity.x * msg->angular_velocity.x +
                                     msg->angular_velocity.y * msg->angular_velocity.y +
                                     msg->angular_velocity.z * msg->angular_velocity.z);
        if (gyro_norm > config_.satu_gyro) {
            LOG(WARNING) << "Gyroscope saturation detected: " << gyro_norm;
        }
    }

    imu_buffer_.push_back(msg);

    // Initialize IMU if needed
    if (imu_need_init_ && imu_buffer_.size() > 100) {
        Vector3d mean_acc = Vector3d::Zero();
        for (const auto& imu : imu_buffer_) {
            mean_acc += Vector3d(imu->linear_acceleration.x,
                               imu->linear_acceleration.y,
                               imu->linear_acceleration.z);
        }
        mean_acc /= imu_buffer_.size();

        // Initialize gravity from mean acceleration
        kf_output_.x_.gravity = mean_acc;
        imu_need_init_ = false;
        after_imu_init_ = true;

        LOG(INFO) << "IMU initialized with gravity: " << kf_output_.x_.gravity.transpose();
    }

    // Limit buffer size
    if (imu_buffer_.size() > 2000) {
        imu_buffer_.pop_front();
    }
}

void LaserMapping::ProcessLoop() {
    LOG(INFO) << "Point-LIO processing thread started";

    while (running_ && !flg_exit_) {
        std::unique_lock<std::mutex> lock(mtx_buffer_);
        sig_buffer_.wait(lock, [this] {
            return (!lidar_buffer_.empty() || !imu_buffer_.empty()) || flg_exit_;
        });

        if (flg_exit_) break;

        if (!SyncPackages(meas_group_)) {
            continue;
        }

        lock.unlock();

        ProcessMeasure(meas_group_);
    }

    LOG(INFO) << "Point-LIO processing thread finished";
}

bool LaserMapping::SyncPackages(MeasureGroup &meas) {
    if (!config_.imu_en) {
        // IMU disabled mode
        if (!lidar_buffer_.empty()) {
            if (!lidar_pushed_) {
                meas.lidar = lidar_buffer_.front();
                meas.lidar_beg_time = time_buffer_.front();
                lose_lid_ = false;

                if (meas.lidar->points.size() < 1) {
                    LOG(WARNING) << "Empty LiDAR scan";
                    lose_lid_ = true;
                } else {
                    double end_time = meas.lidar->points.back().curvature;
                    for (const auto& pt : meas.lidar->points) {
                        if (pt.curvature > end_time) {
                            end_time = pt.curvature;
                        }
                    }
                    lidar_end_time_ = meas.lidar_beg_time + end_time;
                    meas.lidar_end_time = lidar_end_time_;
                    meas.lidar_last_time = lidar_end_time_;
                }
                lidar_pushed_ = true;
            }

            time_buffer_.pop_front();
            lidar_buffer_.pop_front();
            lidar_pushed_ = false;

            return !lose_lid_;
        }
        return false;
    }

    // IMU enabled mode - Point-LIO synchronization logic
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    // Push LiDAR scan
    if (!lidar_pushed_) {
        lose_lid_ = false;
        meas.lidar = lidar_buffer_.front();
        meas.lidar_beg_time = time_buffer_.front();

        if (meas.lidar->points.size() < 1) {
            LOG(WARNING) << "Empty LiDAR scan";
            lose_lid_ = true;
        } else {
            double end_time = meas.lidar->points.back().curvature;
            for (const auto& pt : meas.lidar->points) {
                if (pt.curvature > end_time) {
                    end_time = pt.curvature;
                }
            }
            lidar_end_time_ = meas.lidar_beg_time + end_time / 1000.0; // Convert ms to s
            meas.lidar_end_time = lidar_end_time_;
            meas.lidar_last_time = lidar_end_time_;
        }
        lidar_pushed_ = true;
    }

    if (!lose_lid_ && (last_timestamp_imu_ < meas.lidar_end_time)) {
        return false; // Wait for more IMU data
    }

    if (!lose_lid_ && !imu_pushed_) {
        // Push IMU data within LiDAR time window
        meas.imu_buffer.clear();

        if (imu_need_init_) {
            // Initialize with first IMU data
            if (!imu_buffer_.empty()) {
                imu_next_ = *(imu_buffer_.front());
            }
        }

        auto imu_it = imu_buffer_.begin();
        while (imu_it != imu_buffer_.end() && (*imu_it)->header.stamp.toSec() <= meas.lidar_end_time) {
            meas.imu_buffer.push_back(*imu_it);
            imu_it++;
        }

        imu_pushed_ = true;
    }

    // Clean up buffers
    if (!lose_lid_) {
        time_buffer_.pop_front();
        lidar_buffer_.pop_front();
    } else {
        // Remove empty LiDAR scan
        if (!lidar_buffer_.empty()) {
            time_buffer_.pop_front();
            lidar_buffer_.pop_front();
        }
    }

    // Remove used IMU data
    if (!meas.imu_buffer.empty()) {
        double last_imu_time = meas.imu_buffer.back()->header.stamp.toSec();
        while (!imu_buffer_.empty() && imu_buffer_.front()->header.stamp.toSec() <= last_imu_time) {
            imu_buffer_.pop_front();
        }
    }

    lidar_pushed_ = false;
    imu_pushed_ = false;

    return !lose_lid_ && !meas.lidar->empty() && !meas.imu_buffer.empty();
}

void LaserMapping::ProcessMeasure(MeasureGroup &meas) {
    LOG(INFO) << "Processing Point-LIO measure: " << meas.lidar->size()
              << " points, " << meas.imu_buffer.size() << " IMU measurements";

    // First frame handling
    if (flg_first_scan_) {
        first_lidar_time_ = meas.lidar_beg_time;
        flg_first_scan_ = false;

        if (first_imu_time_ < 1 && !meas.imu_buffer.empty()) {
            first_imu_time_ = meas.imu_buffer.front()->header.stamp.toSec();
            LOG(INFO) << "First IMU time: " << first_imu_time_;
        }

        time_current_ = 0.0;

        if (config_.imu_en) {
            // Initialize gravity from config
            kf_output_.x_.gravity = Vector3d(config_.gravity[0], config_.gravity[1], config_.gravity[2]);
        } else {
            kf_output_.x_.gravity = Vector3d(config_.gravity_init[0], config_.gravity_init[1], config_.gravity_init[2]);
            kf_output_.x_.acc = -kf_output_.x_.rot.transpose() * kf_output_.x_.gravity;
            imu_need_init_ = false;
            after_imu_init_ = true;
        }
    }

    // Point cloud preprocessing (use imu_buffer_ directly)
    *feats_undistort_ = *meas.lidar;

    // Downsample
    if (config_.space_down_sample) {
        downSizeFilterSurf_.setLeafSize(config_.filter_size_surf, config_.filter_size_surf, config_.filter_size_surf);
        downSizeFilterSurf_.setInputCloud(feats_undistort_);
        downSizeFilterSurf_.filter(*feats_down_body_);
    } else {
        *feats_down_body_ = *feats_undistort_;
    }

    // Sort points by time
    std::sort(feats_down_body_->points.begin(), feats_down_body_->points.end(),
              [](const PointType& a, const PointType& b) {
                  return a.curvature < b.curvature;
              });

    feats_down_size_ = feats_down_body_->points.size();

    // IMU initialization check
    if (!after_imu_init_) {
        if (!imu_need_init_) {
            Vector3d tmp_gravity;
            if (config_.imu_en && !meas.imu_buffer.empty()) {
                Vector3d mean_acc = Vector3d::Zero();
                for (const auto& imu : meas.imu_buffer) {
                    mean_acc += Vector3d(imu->linear_acceleration.x,
                                       imu->linear_acceleration.y,
                                       imu->linear_acceleration.z);
                }
                mean_acc /= meas.imu_buffer.size();
                tmp_gravity = -mean_acc / mean_acc.norm() * 9.81;
            } else {
                tmp_gravity = Vector3d(config_.gravity_init[0], config_.gravity_init[1], config_.gravity_init[2]);
                after_imu_init_ = true;
            }

            Matrix3d rot_init;
            // Simple gravity alignment (simplified)
            Vector3d z_axis = tmp_gravity.normalized();
            Vector3d x_axis(1, 0, 0);
            if (std::abs(z_axis.dot(x_axis)) > 0.9) {
                x_axis = Vector3d(0, 1, 0);
            }
            Vector3d y_axis = z_axis.cross(x_axis).normalized();
            x_axis = y_axis.cross(z_axis).normalized();

            rot_init.col(0) = x_axis;
            rot_init.col(1) = y_axis;
            rot_init.col(2) = z_axis;

            kf_output_.x_.rot = rot_init;
            kf_output_.x_.gravity = tmp_gravity;
            kf_output_.x_.acc = -rot_init.transpose() * tmp_gravity;

            LOG(INFO) << "Gravity initialized: " << tmp_gravity.transpose();
        } else {
            return; // Need more IMU data for initialization
        }
    }

    // Initialize map if needed
    if (!init_map_) {
        feats_down_world_->resize(feats_undistort_->size());
        for (int i = 0; i < feats_undistort_->size(); i++) {
            PointBodyToWorld(&(feats_undistort_->points[i]), &(feats_down_world_->points[i]));
        }

        for (const auto& pt : *feats_down_world_) {
            init_feats_world_->points.push_back(pt);
        }

        if (init_feats_world_->points.size() < config_.init_map_size) {
            init_map_ = false;
        } else {
            // In a full implementation, this would add points to an IVox or similar data structure
            LOG(INFO) << "Map initialized with " << init_feats_world_->points.size() << " points";
            init_map_ = true;
            init_feats_world_->clear();
        }
        return;
    }

    // Core Point-LIO processing for use_imu_as_input=false
    if (!config_.use_imu_as_input) {
        LOG(INFO) << "Processing in use_imu_as_input=false mode (Point-LIO style)";

        // In this mode:
        // 1. IMU is used for state propagation
        // 2. LiDAR is used for measurement update (correction)
        // 3. High-frequency prediction with IMU, low-frequency correction with LiDAR

        // Simplified implementation - just propagate and publish
        if (!meas.imu_buffer.empty()) {
            // Use latest IMU for prediction
            const auto& latest_imu = meas.imu_buffer.back();
            Vector3d acc(latest_imu->linear_acceleration.x,
                         latest_imu->linear_acceleration.y,
                         latest_imu->linear_acceleration.z);
            Vector3d gyro(latest_imu->angular_velocity.x,
                          latest_imu->angular_velocity.y,
                          latest_imu->angular_velocity.z);

            input_ikfom input_in;
            input_in.acc = acc;
            input_in.gyro = gyro;

            // Simple prediction (would normally use ESEKF)
            double dt = 0.01; // Simplified time step
            time_current_ = meas.lidar_end_time;

            // Update state (simplified)
            kf_output_.x_.pos += kf_output_.x_.vel * dt;
            kf_output_.x_.vel += (kf_output_.x_.rot * acc + kf_output_.x_.gravity) * dt;

            LOG(INFO) << "State propagated with IMU";
        }

        // Point cloud to world transformation
        feats_down_world_->resize(feats_down_body_->size());
        for (int i = 0; i < feats_down_body_->size(); i++) {
            PointBodyToWorld(&(feats_down_body_->points[i]), &(feats_down_world_->points[i]));
        }

        // Update current pose for publishing
        current_pose_.header.stamp = ros::Time().fromSec(meas.lidar_end_time);
        current_pose_.header.frame_id = "map";
        current_pose_.pose.position.x = kf_output_.x_.pos(0);
        current_pose_.pose.position.y = kf_output_.x_.pos(1);
        current_pose_.pose.position.z = kf_output_.x_.pos(2);

        Quaterniond q(kf_output_.x_.rot);
        current_pose_.pose.orientation.w = q.w();
        current_pose_.pose.orientation.x = q.x();
        current_pose_.pose.orientation.y = q.y();
        current_pose_.pose.orientation.z = q.z();

        // Publish results
        if (config_.scan_publish_en) {
            PublishGlobalMap();
        }
        if (config_.path_publish_en) {
            PublishPath();
        }
        if (config_.scan_effect_pub_en) {
            PublishFrameEffect();
        }
        if (config_.scan_bodyframe_pub_en) {
            PublishFrameBody();
        }

        PublishOdometry(meas.lidar_end_time);

        LOG(INFO) << "Frame processed in use_imu_as_input=false mode";
    }
}

// ESEKF and utility functions (simplified implementations)
void LaserMapping::InitializeState() {
    // Initialize state vector
    kf_output_.x_.pos = Vector3d::Zero();
    kf_output_.x_.rot = Matrix3d::Identity();
    kf_output_.x_.offset_R_L_I = Matrix3d::Identity();
    kf_output_.x_.offset_T_L_I = Vector3d::Zero();
    kf_output_.x_.vel = Vector3d::Zero();
    kf_output_.x_.omg = Vector3d::Zero();
    kf_output_.x_.acc = Vector3d::Zero();
    kf_output_.x_.gravity = Vector3d(0, 0, 9.81);
    kf_output_.x_.bg = Vector3d::Zero();
    kf_output_.x_.ba = Vector3d::Zero();

    // Initialize covariance matrix
    P_init_output_ = Matrix<double, 30, 30>::Identity() * 0.1;
    kf_output_.change_P(P_init_output_);

    // Initialize process noise
    Q_output_ = Matrix<double, 30, 30>::Identity() * 0.01;
    Q_output_.block<3, 3>(0, 0) *= config_.vel_cov;
    Q_output_.block<3, 3>(3, 3) *= config_.gyr_cov_output;
    Q_output_.block<3, 3>(6, 6) *= config_.acc_cov_output;
    Q_output_.block<3, 3>(9, 9) *= config_.b_gyr_cov;
    Q_output_.block<3, 3>(12, 12) *= config_.b_acc_cov;

    LOG(INFO) << "ESEKF state initialized";
}

void LaserMapping::PointBodyToWorld(const PointType *pi, PointType *po) {
    Vector3d p_body(pi->x, pi->y, pi->z);

    // Transform from LiDAR to IMU frame
    Vector3d p_imu = Lidar_R_wrt_IMU_ * p_body + Lidar_T_wrt_IMU_;

    // Transform to world frame
    Vector3d p_world = kf_output_.x_.rot * (kf_output_.x_.offset_R_L_I * p_imu + kf_output_.x_.offset_T_L_I) + kf_output_.x_.pos;

    po->x = p_world(0);
    po->y = p_world(1);
    po->z = p_world(2);
    po->intensity = pi->intensity;
    po->curvature = pi->curvature;
}

void LaserMapping::PointBodyLidarToIMU(const PointType *pi, PointType *po) {
    Vector3d p_lidar(pi->x, pi->y, pi->z);
    Vector3d p_imu = Lidar_R_wrt_IMU_ * p_lidar + Lidar_T_wrt_IMU_;

    po->x = p_imu(0);
    po->y = p_imu(1);
    po->z = p_imu(2);
    po->intensity = pi->intensity;
    po->curvature = pi->curvature;
}

// ESEKF functions (placeholders for full implementation)
void LaserMapping::ESEKFPrediction(double dt, const input_ikfom &in) {
    // Simplified prediction - full implementation would use ESEKF toolkit
    LOG(INFO) << "ESEKF prediction with dt=" << dt;
}

void LaserMapping::ESEKFUpdateWithLidar() {
    // Simplified update - full implementation would do point-to-plane matching
    LOG(INFO) << "ESEKF update with LiDAR";
}

void LaserMapping::ESEKFUpdateWithIMU() {
    // Simplified update - full implementation would do IMU measurement update
    LOG(INFO) << "ESEKF update with IMU";
}

// Publishing functions
void LaserMapping::PublishPath() {
    global_path_.poses.push_back(current_pose_);
    if (global_path_.poses.size() > 10000) {
        global_path_.poses.erase(global_path_.poses.begin());
    }
    pub_path_.publish(global_path_);
}

void LaserMapping::PublishGlobalMap() {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*feats_down_world_, msg);
    msg.header.stamp = ros::Time().fromSec(time_current_);
    msg.header.frame_id = "map";
    pub_global_map_.publish(msg);
}

void LaserMapping::PublishOdometry(const double &time) {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time().fromSec(time);
    odom.header.frame_id = "map";
    odom.child_frame_id = "body";

    odom.pose.pose = current_pose_.pose;

    // Fill velocity (simplified)
    odom.twist.twist.linear.x = kf_output_.x_.vel(0);
    odom.twist.twist.linear.y = kf_output_.x_.vel(1);
    odom.twist.twist.linear.z = kf_output_.x_.vel(2);

    odom.twist.twist.angular.x = kf_output_.x_.omg(0);
    odom.twist.twist.angular.y = kf_output_.x_.omg(1);
    odom.twist.twist.angular.z = kf_output_.x_.omg(2);

    pub_laser_odometry_.publish(odom);
}

void LaserMapping::PublishFrameEffect() {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*feats_down_world_, msg);
    msg.header.stamp = ros::Time().fromSec(time_current_);
    msg.header.frame_id = "map";
    pub_laser_effect_.publish(msg);
}

void LaserMapping::PublishFrameBody() {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*feats_down_body_, msg);
    msg.header.stamp = ros::Time().fromSec(time_current_);
    msg.header.frame_id = "body";
    pub_laser_body_.publish(msg);
}

// ESEKF measurement and process models (placeholders)
void LaserMapping::h_model_output() {
    LOG(INFO) << "ESEKF output measurement model";
}

void LaserMapping::h_model_IMU_output() {
    LOG(INFO) << "ESEKF IMU output measurement model";
}

void LaserMapping::get_f_output() {
    LOG(INFO) << "ESEKF output process model";
}

void LaserMapping::df_dx_output() {
    LOG(INFO) << "ESEKF output process Jacobian";
}

void LaserMapping::PointCloudFilter(PointCloudXYZI::Ptr &cloud, double size) {
    if (cloud->empty() || size < 0.01) return;

    static pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setLeafSize(size, size, size);
    voxel_filter.setInputCloud(cloud);
    voxel_filter.filter(*cloud);
}

void LaserMapping::PointCloudVoxelFilter(PointCloudXYZI::Ptr &cloud, double size) {
    PointCloudFilter(cloud, size);
}

void LaserMapping::PreprocessPoints(const livox_ros_driver::CustomMsg::ConstPtr &msg,
                                   PointCloudXYZI::Ptr &pcl_out) {
    Preprocess preprocessor;
    preprocessor.point_filter_num = config_.point_filter_num;
    preprocessor.blind = config_.blind;
    preprocessor.process(msg, pcl_out);
}

void LaserMapping::PreprocessPoints(const sensor_msgs::PointCloud2::ConstPtr &msg,
                                   PointCloudXYZI::Ptr &pcl_out) {
    Preprocess preprocessor;
    preprocessor.point_filter_num = config_.point_filter_num;
    preprocessor.blind = config_.blind;
    preprocessor.process(msg, pcl_out);
}