#ifndef POINT_LIO_LASER_MAPPING_H
#define POINT_LIO_LASER_MAPPING_H

#include <ros/ros.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <mutex>
#include <deque>
#include <memory>
#include <Eigen/Eigen>

// #include <so3_math.h>  // Removed - now using simplified Eigen operations
// #include <IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp>  // Removed - now using simplified IESKF
#include "d_point_lio/common_data.h"
#include "d_point_lio/ieskf.h"

using namespace std;
using namespace Eigen;


namespace d_point_lio {

class LaserMapping {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    LaserMapping();
    ~LaserMapping();

    /// init with ros
    bool InitROS(ros::NodeHandle &nh);

    /// main processing function
    void Run();

    /// cleanup function
    void Finish();

   private:
    /// subscriber and publisher setup
    void SubAndPubToROS(ros::NodeHandle &nh);

    /// load configuration parameters
    bool LoadParams(ros::NodeHandle &nh);

    /// LiDAR data callback
    void LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg);

    /// IMU data callback
    void IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in);

    /// process synchronized LiDAR and IMU data
    void ProcessMeasure(MeasureGroup &meas);

    /// Point-LIO synchronization functions
    bool SyncPackages(MeasureGroup &meas);

    /// LiDAR point cloud preprocessing
    void PreprocessPoints(const livox_ros_driver::CustomMsg::ConstPtr &msg,
                         PointCloudXYZI::Ptr &pcl_out);

    /// Point-LIO processing functions
    void PointBodyToWorld(const PointType *pi, PointType *po);
    void PointBodyLidarToIMU(const PointType *pi, PointType *po);

    /// ESEKF functions for use_imu_as_input=false mode
    void InitializeState();
    void ESEKFPrediction(double dt, const Vec3& acc, const Vec3& gyro);
    void ESEKFUpdateWithLidar();
    void ESEKFUpdateWithIMU();

    /// point cloud processing
    void PointCloudFilter(PointCloudXYZI::Ptr &cloud, double size);
    void PointCloudVoxelFilter(PointCloudXYZI::Ptr &cloud, double size);

    /// publish functions
    void PublishPath();
    void PublishGlobalMap();
    void PublishOdometry(const double &time);
    void PublishFrameEffect();
    void PublishFrameBody();

    /// ESEKF measurement models
    void h_model_output();
    void h_model_IMU_output();

    /// Process models for ESEKF
    void get_f_output();
    void df_dx_output();

   private:
    /// Topic names
    std::string lidar_topic_ = "/livox/lidar";
    std::string imu_topic_ = "/livox/imu";

    // LiDAR parameters (Livox only)
    int scan_line_ = 6;
    double blind_ = 4.0;
    double time_scale_ = 1e-3;

    // IMU parameters - Enhanced for Point-LIO
    double acc_cov_ = 0.1;
    double gyr_cov_ = 0.1;
    double b_acc_cov_ = 0.0001;
    double b_gyr_cov_ = 0.0001;
    double vel_cov_ = 20.0;
    double acc_cov_input_ = 0.1;
    double gyr_cov_input_ = 0.1;
    double acc_cov_output_ = 0.1;
    double gyr_cov_output_ = 0.1;
    double imu_meas_acc_cov_ = 0.1;
    double imu_meas_omg_cov_ = 0.1;
    double laser_point_cov_ = 0.01;
    double imu_time_inte_ = 0.005;
    double lidar_time_inte_ = 0.1;
    double satu_acc_ = 3.0;
    double satu_gyro_ = 35.0;
    double acc_norm_ = 1.0;

    // Mapping parameters
    double fov_degree_ = 90.0;
    double det_range_ = 450.0;
    bool extrinsic_est_en_ = false;
    std::vector<double> extrinsic_T_ = {0.04165, 0.02326, -0.0284};
    std::vector<double> extrinsic_R_ = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float plane_thr_ = 0.1f;
    double match_s_ = 81;
    int init_map_size_ = 10;

    // Control parameters
    bool use_imu_as_input_ = false;  // Key parameter from Point-LIO
    bool prop_at_freq_of_imu_ = true;
    bool check_satu_ = true;
    bool con_frame_ = false;
    bool cut_frame_ = false;
    bool space_down_sample_ = true;
    bool publish_odometry_without_downsample_ = false;
    bool imu_en_ = true;
    bool runtime_pos_log_ = false;
    int con_frame_num_ = 1;
    double cut_frame_time_interval_ = 0.1;
    double time_diff_lidar_to_imu_ = 0.0;

    // Gravity settings - using common constants
    std::vector<double> gravity_ = {CommonConstants::DEFAULT_GRAVITY.x(),
                                   CommonConstants::DEFAULT_GRAVITY.y(),
                                   CommonConstants::DEFAULT_GRAVITY.z()};
    std::vector<double> gravity_init_ = {CommonConstants::DEFAULT_GRAVITY.x(),
                                        CommonConstants::DEFAULT_GRAVITY.y(),
                                        CommonConstants::DEFAULT_GRAVITY.z()};

    // Publish parameters
    bool path_publish_en_ = false;
    bool scan_publish_en_ = true;
    bool scan_effect_pub_en_ = true;
    bool dense_publish_en_ = false;
    bool scan_bodyframe_pub_en_ = true;
    bool pcd_save_en_ = false;
    int pcd_save_interval_ = -1;

    // Filter parameters
    int point_filter_num_ = 3;
    double filter_size_surf_ = 0.5;
    double filter_size_map_ = 0.5;
    int max_iteration_ = 3;
    double cube_side_length_ = 1000.0;

    // Time synchronization
    bool time_sync_en_ = false;

    /// ros pub and sub stuffs
    ros::Subscriber sub_pcl_;
    ros::Subscriber sub_imu_;

    ros::Publisher pub_path_;
    ros::Publisher pub_global_map_;
    ros::Publisher pub_laser_odometry_;
    ros::Publisher pub_laser_effect_;
    ros::Publisher pub_laser_body_;

    /// Point-LIO data buffers (matching Point-LIO structure)
    std::deque<PointCloudXYZIPtr> lidar_buffer_;
    std::deque<double> time_buffer_;
    std::deque<ImuConstPtr> imu_buffer_;

    /// synchronization
    std::mutex mtx_buffer_;
    bool running_ = false;
    bool flg_exit_ = false;
    bool flg_reset_ = false;

    /// processing status (Point-LIO style)
    bool first_point_ = true;
    bool is_first_frame_ = true;
    bool init_map_ = false;
    bool flg_first_scan_ = true;
    double last_timestamp_lidar_ = -1.0;
    double last_timestamp_imu_ = -1.0;
    double time_update_last_ = 0.0;
    double time_predict_last_const_ = 0.0;
    double time_current_ = 0.0;
    double first_lidar_time_ = 0.0;
    double first_imu_time_ = 0.0;

    /// Point-LIO state variables
    bool lidar_pushed_ = false;
    bool imu_pushed_ = false;
    bool lose_lid_ = false;
    int scan_count_ = 0;
    int frame_ct_ = 0;
    double time_con_ = 0.0;
    double lidar_end_time_ = 0.0;

    /// Point-LIO IMU variables
    IMUData imu_last_data_;
    IMUData imu_next_data_;
    Vec3 angvel_avr_;
    Vec3 acc_avr_;

    /// ESEKF state estimation
    // Simplified IESKF instance
    std::shared_ptr<IESKF> ieskf_;
    IESKFState current_state_;
    bool imu_need_init_ = true;
    bool after_imu_init_ = false;
    double mean_acc_[3] = {0.0, 0.0, 0.0};
    int acc_count_ = 0;

    /// Transformation matrices
    Mat33 Lidar_R_wrt_IMU_;
    Vec3 Lidar_T_wrt_IMU_;

    /// Point clouds (Point-LIO style)
    PointCloudXYZI::Ptr feats_undistort_;
    PointCloudXYZI::Ptr feats_down_body_;
    PointCloudXYZI::Ptr feats_down_world_;
    PointCloudXYZI::Ptr init_feats_world_;
    PointCloudXYZI::Ptr global_map_;  // Global map for PCD saving

    /// Filters
    pcl::VoxelGrid<pcl::PointXYZINormal> downSizeFilterSurf_;
    pcl::VoxelGrid<pcl::PointXYZINormal> downSizeFilterMap_;

    /// Processing variables
    int feats_down_size_ = 0;
    int effct_feat_num_ = 0;
    vector<int> time_seq_;

    /// pose and path
    geometry_msgs::PoseStamped current_pose_;
    nav_msgs::Path global_path_;

    /// current measurement group for processing
    MeasureGroup meas_group_;

    /// Process noise matrices
    Matrix<double, 30, 30> Q_output_;
    Matrix<double, 30, 30> P_init_output_;
};

} // namespace d_point_lio

#endif  // POINT_LIO_LASER_MAPPING_H