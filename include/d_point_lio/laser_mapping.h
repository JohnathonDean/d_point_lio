#ifndef D_POINT_LIO_LASER_MAPPING_H
#define D_POINT_LIO_LASER_MAPPING_H

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <livox_ros_driver/CustomMsg.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Eigen>
#include <deque>
#include <memory>
#include <mutex>

#include "d_point_lio/common_data.h"
#include "d_point_lio/imu_processing.h"
#include "d_point_lio/use-ikfom.hpp"
#include "ivox3d/ivox3d.h"

namespace d_point_lio {

class LaserMapping {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;

    LaserMapping();
    ~LaserMapping();

    /// 使用ROS初始化
    bool InitROS(ros::NodeHandle &nh);

    /// 主处理函数
    void Run();

    /// 清理函数
    void Finish();

   private:
    /// 订阅者和发布者设置
    void SubAndPubToROS(ros::NodeHandle &nh);

    /// 加载配置参数
    bool LoadParams(ros::NodeHandle &nh);

    /// 激光雷达数据回调
    void LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg);

    /// IMU数据回调
    void IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in);

    /// Point-LIO同步函数
    bool SyncPackages();

    // /// 处理同步的激光雷达和IMU数据
    // void ProcessMeasure(MeasureGroup &meas);

   private:
    /// 话题名称
    std::string lidar_topic_ = "/livox/lidar";
    std::string imu_topic_ = "/livox/imu";

    /// ROS发布和订阅相关
    ros::Subscriber sub_pcl_;
    ros::Subscriber sub_imu_;

    ros::Publisher pub_path_;
    ros::Publisher pub_global_map_;
    ros::Publisher pub_laser_odometry_;

    std::mutex mtx_buffer_;
    std::deque<double> time_buffer_;
    std::deque<PointCloudType::Ptr> lidar_buffer_;
    std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer_;

    std::shared_ptr<ImuProcess> p_imu_ = nullptr;                 // imu process
    IVoxType::Options ivox_options_;
    std::shared_ptr<IVoxType> ivox_ = nullptr;                    // localmap in ivox

    MeasureGroup measures_;
    esekfom::esekf<state_ikfom, 30, input_ikfom> kf_;  // esekf

    bool time_sync_en_ = false;
    double timediff_lidar_wrt_imu_ = 0.0;
    double last_timestamp_lidar_ = 0;
    double lidar_end_time_ = 0;
    double last_timestamp_imu_ = -1.0;
    double first_lidar_time_ = 0.0;
    bool lidar_pushed_ = false;
    double lidar_mean_scantime_ = 0.0;
    int scan_num_ = 0;
    bool timediff_set_flg_ = false;
    bool flg_first_scan_ = true;

};

}  // namespace d_point_lio

#endif  // D_POINT_LIO_LASER_MAPPING_H