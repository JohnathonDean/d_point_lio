#include "d_point_lio/laser_mapping.h"

#include <yaml-cpp/yaml.h>
#include <glog/logging.h>



namespace d_point_lio {

LaserMapping::LaserMapping() {
    p_imu_.reset(new ImuProcess());

}

LaserMapping::~LaserMapping() {

}

bool LaserMapping::InitROS(ros::NodeHandle &nh) {
    if (!LoadParams(nh)) {
        LOG(ERROR) << "Failed to load parameters.";
        return false;
    }

    SubAndPubToROS(nh);

    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    return true;
}

bool LaserMapping::LoadParams(ros::NodeHandle &nh) {
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    std::vector<double> extrinT{3, 0.0};  // lidar-imu translation
    std::vector<double> extrinR{9, 0.0};  // lidar-imu rotation
    Eigen::Vector3d lidar_T_wrt_IMU;
    Eigen::Matrix3d lidar_R_wrt_IMU;
    int ivox_nearby_type;

    // 示例参数加载
    nh.param<std::string>("lidar_topic", lidar_topic_, "/livox/lidar");
    nh.param<std::string>("imu_topic", imu_topic_, "/livox/imu");

    nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
    nh.param<std::vector<double>>("mapping/extrinsic_T", extrinT, std::vector<double>());
    nh.param<std::vector<double>>("mapping/extrinsic_R", extrinR, std::vector<double>());

    nh.param<float>("ivox_grid_resolution", ivox_options_.resolution_, 0.2);
    nh.param<int>("ivox_nearby_type", ivox_nearby_type, 18);

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    lidar_T_wrt_IMU = VecFromArray<double>(extrinT);
    lidar_R_wrt_IMU = MatFromArray<double>(extrinR);

    p_imu_->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    p_imu_->SetGyrCov(Eigen::Vector3d(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(Eigen::Vector3d(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(Eigen::Vector3d(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(Eigen::Vector3d(b_acc_cov, b_acc_cov, b_acc_cov));

    return true;
}

void LaserMapping::SubAndPubToROS(ros::NodeHandle &nh) {
    sub_pcl_ = nh.subscribe(lidar_topic_, 100, &LaserMapping::LivoxPCLCallBack, this);
    sub_imu_ = nh.subscribe(imu_topic_, 200, &LaserMapping::IMUCallBack, this);

    pub_path_ = nh.advertise<nav_msgs::Path>("d_point_lio/path", 10);
    pub_global_map_ = nh.advertise<sensor_msgs::PointCloud2>("d_point_lio/global_map", 10);
    pub_laser_odometry_ = nh.advertise<nav_msgs::Odometry>("d_point_lio/odometry", 10);
    pub_path_ = nh.advertise<nav_msgs::Path>("/path", 100000);

}

void LaserMapping::LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    mtx_buffer_.lock();

    if (msg->header.stamp.toSec() < last_timestamp_lidar_) {
        LOG(WARNING) << "lidar loop back, clear buffer";
        lidar_buffer_.clear();
    }
    last_timestamp_lidar_ = msg->header.stamp.toSec();

    if (!time_sync_en_ && std::abs(last_timestamp_imu_ - last_timestamp_lidar_) > 10.0 && !imu_buffer_.empty() &&
        !lidar_buffer_.empty()) {
        LOG(INFO) << "IMU and LiDAR not Synced, IMU time: " << last_timestamp_imu_
                    << ", lidar header time: " << last_timestamp_lidar_;
    }

    if (time_sync_en_ && !timediff_set_flg_ && std::abs(last_timestamp_lidar_ - last_timestamp_imu_) > 1 &&
        !imu_buffer_.empty()) {
        timediff_set_flg_ = true;
        timediff_lidar_wrt_imu_ = last_timestamp_lidar_ + 0.1 - last_timestamp_imu_;
        LOG(INFO) << "Self sync IMU and LiDAR, time diff is " << timediff_lidar_wrt_imu_;
    }

    PointCloudType::Ptr ptr(new PointCloudType());
    // preprocess_->Process(msg, ptr);
    lidar_buffer_.emplace_back(ptr);
    time_buffer_.emplace_back(last_timestamp_lidar_);

    mtx_buffer_.unlock();
}

void LaserMapping::IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in) {
    mtx_buffer_.lock();
    
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    if (std::abs(timediff_lidar_wrt_imu_) > 0.1 && time_sync_en_) {
        msg->header.stamp = ros::Time().fromSec(timediff_lidar_wrt_imu_ + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    if (timestamp < last_timestamp_imu_) {
        LOG(WARNING) << "imu loop back, clear buffer";
        imu_buffer_.clear();
    }

    last_timestamp_imu_ = timestamp;
    imu_buffer_.emplace_back(msg);
    mtx_buffer_.unlock();
}

bool LaserMapping::SyncPackages() {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed_) {
        measures_.lidar_ = lidar_buffer_.front();
        measures_.lidar_begin_time_ = time_buffer_.front();

        if (measures_.lidar_->points.size() <= 1) {
            LOG(WARNING) << "Too few input point cloud!";
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else if (measures_.lidar_->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ = measures_.lidar_begin_time_ + measures_.lidar_->points.back().curvature / double(1000);
            lidar_mean_scantime_ +=
                (measures_.lidar_->points.back().curvature / double(1000) - lidar_mean_scantime_) / scan_num_;
        }

        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) {
        return false;
    }

    /*** push imu_ data, and pop from imu_ buffer ***/
    double imu_time = imu_buffer_.front()->header.stamp.toSec();
    measures_.imu_.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = imu_buffer_.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time_) break;
        measures_.imu_.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;
    return true;
}

void LaserMapping::Run() {
    if (!SyncPackages()) {
        return;
    }

    CloudPtr scan_undistort{new PointCloudType()};   // scan after undistortion
    p_imu_->Process(measures_, kf_, scan_undistort);
    if (scan_undistort->empty() || (scan_undistort == nullptr)) {
        LOG(WARNING) << "No point, skip this scan!";
        return;
    }

    if (flg_first_scan_) {
        
    }



}

void LaserMapping::Finish() {
    LOG(INFO) << "Finishing laser mapping...";
    // 保存最终地图或清理资源
}





} // namespace d_point_lio





