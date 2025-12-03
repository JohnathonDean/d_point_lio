#include "d_point_lio/laser_mapping.h"
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <Eigen/Eigen>
#include <sensor_msgs/Imu.h>
#include <cmath>
#include <cstring>
#include <algorithm>

namespace d_point_lio {

LaserMapping::LaserMapping() {

}

LaserMapping::~LaserMapping() {

}

bool LaserMapping::InitROS(ros::NodeHandle &nh) {
    if (!LoadParams(nh)) {
        LOG(ERROR) << "Failed to load parameters.";
        return false;
    }

    SubAndPubToROS(nh);

    return true;
}

bool LaserMapping::LoadParams(ros::NodeHandle &nh) {
    // 示例参数加载
    nh.param<std::string>("lidar_topic", lidar_topic_, "/livox/lidar");
    nh.param<std::string>("imu_topic", imu_topic_, "/livox/imu");

    return true;
}

void LaserMapping::SubAndPubToROS(ros::NodeHandle &nh) {
    sub_pcl_ = nh.subscribe(lidar_topic_, 100, &LaserMapping::LivoxPCLCallBack, this);
    sub_imu_ = nh.subscribe(imu_topic_, 200, &LaserMapping::IMUCallBack, this);

    pub_path_ = nh.advertise<nav_msgs::Path>("d_point_lio/path", 10);
    pub_global_map_ = nh.advertise<sensor_msgs::PointCloud2>("d_point_lio/global_map", 10);
    pub_laser_odometry_ = nh.advertise<nav_msgs::Odometry>("d_point_lio/odometry", 10);
}

void LaserMapping::LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    mtx_buffer_.lock();

    if (msg->header.stamp.toSec() < last_timestamp_lidar_) {
        LOG(WARNING) << "lidar loop back, clear buffer";
        lidar_buffer_.clear();
    }
    last_timestamp_lidar_ = msg->header.stamp.toSec();

    if (!time_sync_en_ && abs(last_timestamp_imu_ - last_timestamp_lidar_) > 10.0 && !imu_buffer_.empty() &&
        !lidar_buffer_.empty()) {
        LOG(INFO) << "IMU and LiDAR not Synced, IMU time: " << last_timestamp_imu_
                    << ", lidar header time: " << last_timestamp_lidar_;
    }

    if (time_sync_en_ && !timediff_set_flg_ && abs(last_timestamp_lidar_ - last_timestamp_imu_) > 1 &&
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
    if (abs(timediff_lidar_wrt_imu_) > 0.1 && time_sync_en_) {
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

void LaserMapping::SyncPackages() {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed_) {
        measures_.lidar_ = lidar_buffer_.front();
        measures_.lidar_bag_time_ = time_buffer_.front();

        if (measures_.lidar_->points.size() <= 1) {
            LOG(WARNING) << "Too few input point cloud!";
            lidar_end_time_ = measures_.lidar_bag_time_ + lidar_mean_scantime_;
        } else if (measures_.lidar_->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_bag_time_ + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ = measures_.lidar_bag_time_ + measures_.lidar_->points.back().curvature / double(1000);
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

}

void LaserMapping::Finish() {
    LOG(INFO) << "Finishing laser mapping...";
    // 保存最终地图或清理资源
}





} // namespace d_point_lio





