#ifndef D_POINT_LIO_COMMON_DATA_H
#define D_POINT_LIO_COMMON_DATA_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/Imu.h>
#include <mutex>
#include <deque>
#include <memory>
#include <Eigen/Eigen>

using PointType = pcl::PointXYZINormal;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

namespace d_point_lio {

struct MeasureGroup {
    MeasureGroup() {
        lidar_.reset(new PointCloudType());
        this->lidar_begin_time_ = 0.0;
        this->lidar_end_time_ = 0.0;
    }
    
    double lidar_begin_time_;
    double lidar_end_time_;
    PointCloudType::Ptr lidar_ = nullptr;
    std::deque<sensor_msgs::Imu::ConstPtr> imu_;
};

} // namespace d_point_lio

#endif  // POINT_LIO_LASER_MAPPING_H