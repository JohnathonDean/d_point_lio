#ifndef D_POINT_LIO_IMU_PROCESSING_H
#define D_POINT_LIO_IMU_PROCESSING_H

#include <math.h>
#include <cmath>
#include <deque>
#include <mutex>
#include <thread>
#include <Eigen/Eigen>

#include "common_data.h"
#include "use-ikfom.hpp"

namespace d_point_lio {

constexpr int MAX_INI_COUNT = 20;
constexpr double G_m_s2 = 9.81;  // Gravity const in GuangDong/China


class ImuProcess {
   public:
    ImuProcess();
    ~ImuProcess();

    /**
     * @brief 重置IMU处理器状态
     * 将所有参数恢复到初始状态，用于重新初始化
     */
    void Reset();
    
    /**
     * @brief 设置激光雷达相对于IMU的外参
     * @param transl 平移向量 (IMU坐标系下的激光雷达位置)
     * @param rot 旋转矩阵 (IMU到激光雷达的旋转)
     */
    void SetExtrinsic(const Eigen::Vector3d &transl, const Eigen::Matrix3d &rot);
    
    /**
     * @brief 设置陀螺仪测量噪声协方差
     * @param scaler 陀螺仪噪声缩放因子 (x, y, z方向)
     */
    void SetGyrCov(const Eigen::Vector3d &scaler);
    
    /**
     * @brief 设置加速度计测量噪声协方差
     * @param scaler 加速度计噪声缩放因子 (x, y, z方向)
     */
    void SetAccCov(const Eigen::Vector3d &scaler);
    
    /**
     * @brief 设置陀螺仪零偏随机游走协方差
     * @param b_g 陀螺仪零偏噪声 (x, y, z方向)
     */
    void SetGyrBiasCov(const Eigen::Vector3d &b_g);
    
    /**
     * @brief 设置加速度计零偏随机游走协方差
     * @param b_a 加速度计零偏噪声 (x, y, z方向)
     */
    void SetAccBiasCov(const Eigen::Vector3d &b_a);

    /**
     * @brief 处理IMU和激光雷达测量数据
     * @param meas 测量数据组（包含IMU和激光雷达数据）
     * @param kf_state 卡尔曼滤波器状态
     * @param cur_pcl_un_ 去畸变后的点云（输出）
     */
    void Process(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 30, input_ikfom> &kf_state,
                 PointCloudType::Ptr cur_pcl_un_);

    Eigen::Matrix<double, 12, 12> Q_;           ///< 过程噪声协方差矩阵 (12维)
    Eigen::Vector3d cov_acc_;                    ///< 加速度计测量协方差
    Eigen::Vector3d cov_gyr_;                    ///< 陀螺仪测量协方差
    Eigen::Vector3d cov_acc_scale_;              ///< 加速度计协方差缩放因子
    Eigen::Vector3d cov_gyr_scale_;              ///< 陀螺仪协方差缩放因子
    Eigen::Vector3d cov_bias_gyr_;               ///< 陀螺仪零偏随机游走协方差
    Eigen::Vector3d cov_bias_acc_;               ///< 加速度计零偏随机游走协方差

    Eigen::Vector3d gravity_;                    ///< 重力向量 (世界坐标系)

   private:
    Eigen::Matrix3d Lidar_R_wrt_IMU_;            ///< 激光雷达相对于IMU的旋转矩阵
    Eigen::Vector3d Lidar_T_wrt_IMU_;            ///< 激光雷达相对于IMU的平移向量
    
    Eigen::Vector3d mean_acc_;                   ///< 加速度计均值（用于初始化）
    Eigen::Vector3d mean_gyr_;                   ///< 陀螺仪均值（用于初始化）

    int init_iter_num_ = 1;
    bool b_first_frame_ = true;                  ///< 是否为第一帧数据
    bool imu_need_init_ = true;                  ///< IMU是否需要初始化


   private:
    /**
     * @brief IMU初始化函数
     * 
     * 通过静止状态下的IMU数据计算初始状态：
     * 1. 估计重力方向和大小
     * 2. 计算陀螺仪零偏
     * 3. 计算加速度计和陀螺仪的协方差
     * 4. 初始化卡尔曼滤波器状态和协方差
     * 
     * @param meas 测量数据组
     * @param kf_state 卡尔曼滤波器状态
     * @param N 已处理的IMU数据帧数
     */
    void IMUInit(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 30, input_ikfom> &kf_state, int &N);

    /**
     * @brief 生成过程噪声协方差矩阵
     * @return 12x12的过程噪声协方差矩阵
     */
    Eigen::Matrix<double, 12, 12> process_noise_cov();


};

}  // namespace d_point_lio

#endif  // D_POINT_LIO_IMU_PROCESSING_H