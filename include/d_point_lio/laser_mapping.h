#ifndef POINT_LIO_LASER_MAPPING_H
#define POINT_LIO_LASER_MAPPING_H

#include <livox_ros_driver/CustomMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class LaserMapping {
   public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    LaserMapping() {}
    ~LaserMapping() {}

    /// init with ros
    bool InitROS(ros::NodeHandle &nh);

//     void Run();

//     void Finish();

//    private:
//     void SubAndPubToROS(ros::NodeHandle &nh);

//     bool LoadParams(ros::NodeHandle &nh);

   private:
    /// ros pub and sub stuffs
    ros::Subscriber sub_pcl_;
    ros::Subscriber sub_imu_;
};

#endif  // POINT_LIO_LASER_MAPPING_H