#ifndef POINT_LIO_PREPROCESS_H
#define POINT_LIO_PREPROCESS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
#include <ros/ros.h>
#include <vector>
#include <deque>

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

class Preprocess {
public:
    Preprocess();
    ~Preprocess();

    // Process standard PCL point cloud
    void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &out);

    // Process Livox point cloud
    void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &out);

    // Configuration parameters
    int point_filter_num = 2;
    int N_SCANS = 16;
    int SCAN_RATE = 10;
    int time_unit = 1;
    double blind = 1.0;
    int lidar_type = 1;

private:
    void processLivox(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &out);
    void processStandard(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &out);
};

#endif // POINT_LIO_PREPROCESS_H