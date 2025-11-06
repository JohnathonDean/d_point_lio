#include "d_point_lio/preprocess.h"
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <glog/logging.h>

Preprocess::Preprocess() {
    ROS_INFO("Preprocess initialized");
}

Preprocess::~Preprocess() {
    ROS_INFO("Preprocess destroyed");
}

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &out) {
    processStandard(msg, out);
}

void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &out) {
    processLivox(msg, out);
}

void Preprocess::processStandard(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &out) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *raw_cloud);

    out->clear();
    out->reserve(raw_cloud->size());

    for (size_t i = 0; i < raw_cloud->size(); i++) {
        if ((i % point_filter_num) != 0) continue;

        const auto &pt = raw_cloud->points[i];
        if (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z < blind * blind) continue;

        PointType point_temp;
        point_temp.x = pt.x;
        point_temp.y = pt.y;
        point_temp.z = pt.z;
        point_temp.intensity = pt.intensity;
        point_temp.curvature = 0.0; // No time info in standard PCL

        out->push_back(point_temp);
    }

    ROS_INFO("Processed standard PCL: %zu -> %zu points", raw_cloud->size(), out->size());
}

void Preprocess::processLivox(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &out) {
    out->clear();
    out->reserve(msg->point_num);

    for (int i = 0; i < msg->point_num; i++) {
        if ((i % point_filter_num) != 0) continue;

        if (std::abs(msg->points[i].x) < blind &&
            std::abs(msg->points[i].y) < blind &&
            std::abs(msg->points[i].z) < blind) continue;

        PointType point_temp;
        point_temp.x = msg->points[i].x;
        point_temp.y = msg->points[i].y;
        point_temp.z = msg->points[i].z;
        point_temp.intensity = msg->points[i].reflectivity;
        point_temp.curvature = msg->points[i].offset_time * 1e-3; // Convert to seconds

        out->push_back(point_temp);
    }

    ROS_INFO("Processed Livox: %u -> %zu points", msg->point_num, out->size());
}