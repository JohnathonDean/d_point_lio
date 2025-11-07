#include "d_point_lio/laser_mapping.h"
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <Eigen/Eigen>
#include <cmath>
#include <cstring>
#include <algorithm>

namespace d_point_lio {

LaserMapping::LaserMapping() {
    // Initialize pointers
    current_pose_.header.frame_id = "map";
    current_pose_.pose.orientation.w = 1.0;
    global_path_.header.frame_id = "map";

    // Initialize ESEKF
    InitializeState();

    // Initialize point clouds
    feats_undistort_.reset(new PointCloudXYZI());
    feats_down_body_.reset(new PointCloudXYZI());
    feats_down_world_.reset(new PointCloudXYZI());
    init_feats_world_.reset(new PointCloudXYZI());
    global_map_.reset(new PointCloudXYZI());

    // Set up transformation matrices
    Lidar_R_wrt_IMU_ = CommonConstants::IDENTITY_MATRIX;
    Lidar_T_wrt_IMU_ = Vec3::Zero();

    // Initialize Point-LIO status variables
    first_point_ = true;
    is_first_frame_ = true;
    init_map_ = false;
    flg_first_scan_ = true;
    last_timestamp_lidar_ = -1.0;
    last_timestamp_imu_ = -1.0;
    time_update_last_ = 0.0;
    time_predict_last_const_ = 0.0;
    time_current_ = 0.0;
    first_lidar_time_ = 0.0;
    first_imu_time_ = 0.0;

    // Initialize Point-LIO state variables
    lidar_pushed_ = false;
    imu_pushed_ = false;
    lose_lid_ = false;
    scan_count_ = 0;
    frame_ct_ = 0;
    time_con_ = 0.0;
    lidar_end_time_ = 0.0;

    imu_need_init_ = true;
    after_imu_init_ = false;
    memset(mean_acc_, 0, sizeof(mean_acc_));
    acc_count_ = 0;

    ROS_INFO("LaserMapping constructor completed with Point-LIO initialization");
}

LaserMapping::~LaserMapping() {
    Finish();
}

bool LaserMapping::InitROS(ros::NodeHandle &nh) {
    ROS_INFO("Initializing LaserMapping...");

    if (!LoadParams(nh)) {
        ROS_ERROR("Failed to load parameters");
        return false;
    }

    SubAndPubToROS(nh);

    running_ = true;

    ROS_INFO("LaserMapping initialized successfully");
    return true;
}

void LaserMapping::Run() {
    // Process one iteration in faster-lio style
    if (!running_ || flg_exit_) return;

    std::unique_lock<std::mutex> lock(mtx_buffer_);
    if (SyncPackages(meas_group_)) {
        lock.unlock();
        ProcessMeasure(meas_group_);
    } else {
        lock.unlock();
    }
}

void LaserMapping::Finish() {
    // Save final map PCD file (faster-lio style)
    if (pcd_save_en_ && !global_map_->empty()) {
        std::string file_name = "final_map.pcd";

        try {
            // Create directory if it doesn't exist
            boost::filesystem::path dir("./pcd");
            if (!boost::filesystem::exists(dir)) {
                boost::filesystem::create_directories(dir);
            }

            // Downsample global map before saving (optional, for smaller file size)
            PointCloudXYZI::Ptr map_to_save(new PointCloudXYZI());
            if (filter_size_map_ > 0.01) {
                downSizeFilterMap_.setInputCloud(global_map_);
                downSizeFilterMap_.setLeafSize(filter_size_map_, filter_size_map_, filter_size_map_);
                downSizeFilterMap_.filter(*map_to_save);
            } else {
                map_to_save = global_map_;
            }

            // Save point cloud to PCD file
            if (pcl::io::savePCDFileASCII("./pcd/" + file_name, *map_to_save) == 0) {
                ROS_INFO("Saved final map to: ./pcd/%s (%zu points, downsampled to %zu points)",
                         file_name.c_str(), global_map_->size(), map_to_save->size());
            } else {
                ROS_ERROR("Failed to save final map PCD file");
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Error saving PCD file: %s", e.what());
        }
    } else {
        ROS_INFO("PCD saving disabled or map is empty");
    }

    // Set running flag to false
    running_ = false;
}

bool LaserMapping::LoadParams(ros::NodeHandle &nh) {
    // Load Point-LIO specific parameters
    nh.getParam("use_imu_as_input", use_imu_as_input_);
    nh.getParam("prop_at_freq_of_imu", prop_at_freq_of_imu_);
    nh.getParam("check_satu", check_satu_);
    nh.getParam("space_down_sample", space_down_sample_);
    nh.getParam("publish_odometry_without_downsample", publish_odometry_without_downsample_);
    nh.getParam("imu_en", imu_en_);
    nh.getParam("runtime_pos_log_enable", runtime_pos_log_);
    nh.getParam("mapping/extrinsic_est_en", extrinsic_est_en_);
    nh.getParam("con_frame_num", con_frame_num_);
    nh.getParam("cut_frame_time_interval", cut_frame_time_interval_);
    nh.getParam("common/time_diff_lidar_to_imu", time_diff_lidar_to_imu_);

    // Load noise parameters
    nh.getParam("mapping/acc_cov_input", acc_cov_input_);
    nh.getParam("mapping/gyr_cov_input", gyr_cov_input_);
    nh.getParam("mapping/acc_cov_output", acc_cov_output_);
    nh.getParam("mapping/gyr_cov_output", gyr_cov_output_);
    nh.getParam("mapping/vel_cov", vel_cov_);
    nh.getParam("mapping/b_gyr_cov", b_gyr_cov_);
    nh.getParam("mapping/b_acc_cov", b_acc_cov_);
    nh.getParam("mapping/imu_meas_acc_cov", imu_meas_acc_cov_);
    nh.getParam("mapping/imu_meas_omg_cov", imu_meas_omg_cov_);
    nh.getParam("mapping/lidar_meas_cov", laser_point_cov_);
    nh.getParam("mapping/imu_time_inte", imu_time_inte_);
    nh.getParam("mapping/lidar_time_inte", lidar_time_inte_);
    nh.getParam("mapping/satu_acc", satu_acc_);
    nh.getParam("mapping/satu_gyro", satu_gyro_);
    nh.getParam("mapping/acc_norm", acc_norm_);

    // Load mapping parameters
    nh.getParam("mapping/plane_thr", plane_thr_);
    nh.getParam("mapping/match_s", match_s_);
    nh.getParam("init_map_size", init_map_size_);

    // Load gravity settings
    nh.getParam("mapping/gravity", gravity_);
    nh.getParam("mapping/gravity_init", gravity_init_);

    // Load common parameters
    nh.getParam("common/lid_topic", lidar_topic_);
    nh.getParam("common/imu_topic", imu_topic_);
    nh.getParam("common/time_sync_en", time_sync_en_);
    nh.getParam("common/con_frame", con_frame_);
    nh.getParam("common/cut_frame", cut_frame_);

    // Load preprocess parameters
        nh.getParam("preprocess/scan_line", scan_line_);
    nh.getParam("preprocess/blind", blind_);
    nh.getParam("preprocess/time_scale", time_scale_);

    // Load filter parameters
    nh.getParam("point_filter_num", point_filter_num_);
    nh.getParam("filter_size_surf", filter_size_surf_);
    nh.getParam("filter_size_map", filter_size_map_);
    nh.getParam("max_iteration", max_iteration_);
    nh.getParam("cube_side_length", cube_side_length_);

    // Load publish parameters
    nh.getParam("publish/path_publish_en", path_publish_en_);
    nh.getParam("publish/scan_publish_en", scan_publish_en_);
    nh.getParam("publish/scan_effect_pub_en", scan_effect_pub_en_);
    nh.getParam("publish/dense_publish_en", dense_publish_en_);
    nh.getParam("publish/scan_bodyframe_pub_en", scan_bodyframe_pub_en_);
    nh.getParam("path_save_en", pcd_save_en_);
    nh.getParam("pcd_save/interval", pcd_save_interval_);

    // Setup extrinsic transformation
    if (extrinsic_T_.size() >= 3 && extrinsic_R_.size() >= 9) {
        Lidar_T_wrt_IMU_ = Vec3(extrinsic_T_[0], extrinsic_T_[1], extrinsic_T_[2]);
        Lidar_R_wrt_IMU_ = Mat33(extrinsic_R_.data());
    }

    ROS_INFO("Point-LIO parameters loaded successfully");
    ROS_INFO("use_imu_as_input: %s", use_imu_as_input_ ? "true" : "false");
    ROS_INFO("LiDAR topic: %s", lidar_topic_.c_str());
    ROS_INFO("IMU topic: %s", imu_topic_.c_str());
    
    return true;
}

void LaserMapping::SubAndPubToROS(ros::NodeHandle &nh) {
    // Livox LiDAR subscriber (only Livox supported)
    sub_pcl_ = nh.subscribe(lidar_topic_, 100000,
                            &LaserMapping::LivoxPCLCallBack, this);
    // IMU subscriber
    sub_imu_ = nh.subscribe(imu_topic_, 200000,
                           &LaserMapping::IMUCallBack, this);

    // Publishers
    pub_path_ = nh.advertise<nav_msgs::Path>("path", 100);
    pub_global_map_ = nh.advertise<sensor_msgs::PointCloud2>("cloud_registered", 100);
    pub_laser_odometry_ = nh.advertise<nav_msgs::Odometry>("odom", 100);
    pub_laser_effect_ = nh.advertise<sensor_msgs::PointCloud2>("cloud_effected", 100);
    pub_laser_body_ = nh.advertise<sensor_msgs::PointCloud2>("cloud_body", 100);

    ROS_INFO("ROS subscribers and publishers initialized");
}

void LaserMapping::LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(mtx_buffer_);

    // Point-LIO style callback
    scan_count_++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar_) {
        ROS_ERROR("LiDAR loop back, clear buffer");
        return;
    }

    last_timestamp_lidar_ = msg->header.stamp.toSec();

    // Process Livox point cloud directly
    PointCloudXYZI::Ptr processed_cloud(new PointCloudXYZI());
    PreprocessPoints(msg, processed_cloud);

    if (processed_cloud->points.size() > 0) {
        lidar_buffer_.push_back(processed_cloud);
        time_buffer_.push_back(msg->header.stamp.toSec());
    }

    // Limit buffer size
    if (lidar_buffer_.size() > 1000) {
        lidar_buffer_.pop_front();
        time_buffer_.pop_front();
    }

    }


void LaserMapping::IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in) {
    std::lock_guard<std::mutex> lock(mtx_buffer_);

    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    // Apply time offset compensation
    msg->header.stamp = ros::Time().fromSec(msg->header.stamp.toSec() - time_diff_lidar_to_imu_);

    double timestamp = msg->header.stamp.toSec();

    if (timestamp < last_timestamp_imu_) {
        ROS_ERROR("IMU loop back, clear deque");
        return;
    }

    last_timestamp_imu_ = timestamp;

    // Check for saturation if enabled
    if (check_satu_) {
        double acc_norm = std::sqrt(msg->linear_acceleration.x * msg->linear_acceleration.x +
                                    msg->linear_acceleration.y * msg->linear_acceleration.y +
                                    msg->linear_acceleration.z * msg->linear_acceleration.z);
        if (acc_norm > satu_acc_) {
            ROS_WARN("Accelerometer saturation detected: %f", acc_norm);
        }

        double gyro_norm = std::sqrt(msg->angular_velocity.x * msg->angular_velocity.x +
                                     msg->angular_velocity.y * msg->angular_velocity.y +
                                     msg->angular_velocity.z * msg->angular_velocity.z);
        if (gyro_norm > satu_gyro_) {
            ROS_WARN("Gyroscope saturation detected: %f", gyro_norm);
        }
    }

    imu_buffer_.push_back(msg);

    // Initialize IMU if needed
    if (imu_need_init_ && imu_buffer_.size() > 100) {
        Vec3 mean_acc = Vec3::Zero();
        for (const auto& imu : imu_buffer_) {
            mean_acc += Vec3(imu->linear_acceleration.x,
                           imu->linear_acceleration.y,
                           imu->linear_acceleration.z);
        }
        mean_acc /= imu_buffer_.size();

        // Initialize gravity from mean acceleration
        kf_output_.x_.gravity = mean_acc;
        imu_need_init_ = false;
        after_imu_init_ = true;

        ROS_INFO("IMU initialized with gravity: [%f, %f, %f]",
             kf_output_.x_.gravity.x(), kf_output_.x_.gravity.y(), kf_output_.x_.gravity.z());
    }

    // Limit buffer size
    if (imu_buffer_.size() > 2000) {
        imu_buffer_.pop_front();
    }
}


bool LaserMapping::SyncPackages(MeasureGroup &meas) {
    if (!imu_en_) {
        // IMU disabled mode
        if (!lidar_buffer_.empty()) {
            if (!lidar_pushed_) {
                meas.lidar = lidar_buffer_.front();
                meas.lidar_beg_time = time_buffer_.front();
                lose_lid_ = false;

                if (meas.lidar->points.size() < 1) {
                    ROS_WARN("Empty LiDAR scan");
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
            ROS_WARN("Empty LiDAR scan");
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
                imu_next_data_.acc = Vec3(imu_buffer_.front()->linear_acceleration.x,
                                        imu_buffer_.front()->linear_acceleration.y,
                                        imu_buffer_.front()->linear_acceleration.z);
                imu_next_data_.gyro = Vec3(imu_buffer_.front()->angular_velocity.x,
                                          imu_buffer_.front()->angular_velocity.y,
                                          imu_buffer_.front()->angular_velocity.z);
                imu_next_data_.timestamp = imu_buffer_.front()->header.stamp.toSec();
                imu_next_data_.valid = true;
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
    ROS_INFO("Processing Point-LIO measure: %zu points, %zu IMU measurements",
              meas.lidar->size(), meas.imu_buffer.size());

    // First frame handling
    if (flg_first_scan_) {
        first_lidar_time_ = meas.lidar_beg_time;
        flg_first_scan_ = false;

        if (first_imu_time_ < 1 && !meas.imu_buffer.empty()) {
            first_imu_time_ = meas.imu_buffer.front()->header.stamp.toSec();
            ROS_INFO("First IMU time: %f", first_imu_time_);
        }

        time_current_ = 0.0;

        if (imu_en_) {
            // Initialize gravity from config
            kf_output_.x_.gravity = Vec3(gravity_[0], gravity_[1], gravity_[2]);
        } else {
            kf_output_.x_.gravity = Vec3(gravity_init_[0], gravity_init_[1], gravity_init_[2]);
            kf_output_.x_.acc = -kf_output_.x_.rot.transpose() * kf_output_.x_.gravity;
            imu_need_init_ = false;
            after_imu_init_ = true;
        }
    }

    // Point cloud preprocessing
    *feats_undistort_ = *meas.lidar;

    // Downsample
    if (space_down_sample_) {
        downSizeFilterSurf_.setLeafSize(filter_size_surf_, filter_size_surf_, filter_size_surf_);
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
            Vec3 tmp_gravity;
            if (imu_en_ && !meas.imu_buffer.empty()) {
                Vec3 mean_acc = Vec3::Zero();
                for (const auto& imu : meas.imu_buffer) {
                    mean_acc += Vec3(imu->linear_acceleration.x,
                                   imu->linear_acceleration.y,
                                   imu->linear_acceleration.z);
                }
                mean_acc /= meas.imu_buffer.size();
                tmp_gravity = -mean_acc / mean_acc.norm() * CommonConstants::GRAVITY_NORM;
            } else {
                tmp_gravity = Vec3(gravity_init_[0], gravity_init_[1], gravity_init_[2]);
                after_imu_init_ = true;
            }

            Mat33 rot_init;
            // Simple gravity alignment (simplified)
            Vec3 z_axis = tmp_gravity.normalized();
            Vec3 x_axis(1, 0, 0);
            if (std::abs(z_axis.dot(x_axis)) > 0.9) {
                x_axis = Vec3(0, 1, 0);
            }
            Vec3 y_axis = z_axis.cross(x_axis).normalized();
            x_axis = y_axis.cross(z_axis).normalized();

            rot_init.col(0) = x_axis;
            rot_init.col(1) = y_axis;
            rot_init.col(2) = z_axis;

            kf_output_.x_.rot = rot_init;
            kf_output_.x_.gravity = tmp_gravity;
            kf_output_.x_.acc = -rot_init.transpose() * tmp_gravity;

            ROS_INFO("Gravity initialized: [%f, %f, %f]", tmp_gravity.x(), tmp_gravity.y(), tmp_gravity.z());
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

        if (init_feats_world_->points.size() < init_map_size_) {
            init_map_ = false;
        } else {
            // In a full implementation, this would add points to an IVox or similar data structure
            ROS_INFO("Map initialized with %zu points", init_feats_world_->points.size());
            init_map_ = true;
            init_feats_world_->clear();
        }
        return;
    }

    // Core Point-LIO processing for use_imu_as_input=false
    if (!use_imu_as_input_) {
        ROS_INFO("Processing in use_imu_as_input=false mode (Point-LIO style)");

        // In this mode:
        // 1. IMU is used for state propagation
        // 2. LiDAR is used for measurement update (correction)
        // 3. High-frequency prediction with IMU, low-frequency correction with LiDAR

        // Simplified implementation - just propagate and publish
        if (!meas.imu_buffer.empty()) {
            // Use latest IMU for prediction
            const auto& latest_imu = meas.imu_buffer.back();
            Vec3 acc(latest_imu->linear_acceleration.x,
                     latest_imu->linear_acceleration.y,
                     latest_imu->linear_acceleration.z);
            Vec3 gyro(latest_imu->angular_velocity.x,
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

            ROS_INFO("State propagated with IMU");
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
        if (scan_publish_en_) {
            PublishGlobalMap();
        }
        if (path_publish_en_) {
            PublishPath();
        }
        if (scan_effect_pub_en_) {
            PublishFrameEffect();
        }
        if (scan_bodyframe_pub_en_) {
            PublishFrameBody();
        }

        PublishOdometry(meas.lidar_end_time);

        ROS_INFO("Frame processed in use_imu_as_input=false mode");
    }
}

void LaserMapping::PreprocessPoints(const livox_ros_driver::CustomMsg::ConstPtr &msg,
                                   pcl::PointCloud<pcl::PointXYZINormal>::Ptr &pcl_out) {
    pcl_out->clear();
    int plsize = msg->point_num;

    for (int i = 0; i < plsize; i++) {
        if ((i % point_filter_num_) != 0) continue;

        if (msg->points[i].x < blind_ &&
            msg->points[i].y < blind_ &&
            msg->points[i].z < blind_) continue;

        pcl::PointXYZINormal point_temp;
        point_temp.x = msg->points[i].x;
        point_temp.y = msg->points[i].y;
        point_temp.z = msg->points[i].z;
        point_temp.intensity = msg->points[i].reflectivity;
        point_temp.curvature = msg->points[i].offset_time * time_scale_;

        pcl_out->push_back(point_temp);
    }
}


void LaserMapping::PointCloudVoxelFilter(pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud, double size) {
    if (cloud->empty() || size < 0.01) return;

    static pcl::VoxelGrid<pcl::PointXYZINormal> voxel_filter;
    voxel_filter.setLeafSize(size, size, size);
    voxel_filter.setInputCloud(cloud);
    voxel_filter.filter(*cloud);
}

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

    // Fill velocity from state estimation
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

// Point-LIO auxiliary functions
void LaserMapping::InitializeState() {
    // Initialize state vector
    kf_output_.x_.pos = Vec3::Zero();
    kf_output_.x_.rot = CommonConstants::IDENTITY_MATRIX;
    kf_output_.x_.offset_R_L_I = CommonConstants::IDENTITY_MATRIX;
    kf_output_.x_.offset_T_L_I = Vec3::Zero();
    kf_output_.x_.vel = Vec3::Zero();
    kf_output_.x_.omg = Vec3::Zero();
    kf_output_.x_.acc = Vec3::Zero();
    kf_output_.x_.gravity = CommonConstants::DEFAULT_GRAVITY;
    kf_output_.x_.bg = Vec3::Zero();
    kf_output_.x_.ba = Vec3::Zero();

    // Initialize covariance matrix
    P_init_output_ = Matrix<double, 30, 30>::Identity() * 0.1;
    kf_output_.change_P(P_init_output_);

    // Initialize process noise
    Q_output_ = Matrix<double, 30, 30>::Identity() * 0.01;
    Q_output_.block<3, 3>(0, 0) *= vel_cov_;
    Q_output_.block<3, 3>(3, 3) *= gyr_cov_output_;
    Q_output_.block<3, 3>(6, 6) *= acc_cov_output_;
    Q_output_.block<3, 3>(9, 9) *= b_gyr_cov_;
    Q_output_.block<3, 3>(12, 12) *= b_acc_cov_;

    ROS_INFO("ESEKF state initialized");
}

void LaserMapping::PointBodyToWorld(const PointType *pi, PointType *po) {
    Vec3 p_body(pi->x, pi->y, pi->z);

    // Transform from LiDAR to IMU frame
    Vec3 p_imu = Lidar_R_wrt_IMU_ * p_body + Lidar_T_wrt_IMU_;

    // Transform to world frame
    Vec3 p_world = kf_output_.x_.rot * (kf_output_.x_.offset_R_L_I * p_imu + kf_output_.x_.offset_T_L_I) + kf_output_.x_.pos;

    po->x = p_world(0);
    po->y = p_world(1);
    po->z = p_world(2);
    po->intensity = pi->intensity;
    po->curvature = pi->curvature;
}

void LaserMapping::PointBodyLidarToIMU(const PointType *pi, PointType *po) {
    Vec3 p_lidar(pi->x, pi->y, pi->z);
    Vec3 p_imu = Lidar_R_wrt_IMU_ * p_lidar + Lidar_T_wrt_IMU_;

    po->x = p_imu(0);
    po->y = p_imu(1);
    po->z = p_imu(2);
    po->intensity = pi->intensity;
    po->curvature = pi->curvature;
}

// ESEKF functions (placeholders for full implementation)
void LaserMapping::ESEKFPrediction(double dt, const input_ikfom &in) {
    // Simplified prediction - full implementation would use ESEKF toolkit
    ROS_INFO("ESEKF prediction with dt=%f", dt);
}

void LaserMapping::ESEKFUpdateWithLidar() {
    // Simplified update - full implementation would do point-to-plane matching
    ROS_INFO("ESEKF update with LiDAR");
}

void LaserMapping::ESEKFUpdateWithIMU() {
    // Simplified update - full implementation would do IMU measurement update
    ROS_INFO("ESEKF update with IMU");
}

// ESEKF measurement and process models (placeholders)
void LaserMapping::h_model_output() {
    ROS_INFO("ESEKF output measurement model");
}

void LaserMapping::h_model_IMU_output() {
    ROS_INFO("ESEKF IMU output measurement model");
}

void LaserMapping::get_f_output() {
    ROS_INFO("ESEKF output process model");
}

void LaserMapping::df_dx_output() {
    ROS_INFO("ESEKF output process Jacobian");
}

} // namespace d_point_lio





