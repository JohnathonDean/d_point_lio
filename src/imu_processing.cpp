#include "d_point_lio/imu_processing.h"

namespace d_point_lio {

/**
 * @brief 构造函数 - 初始化IMU处理器
 */
ImuProcess::ImuProcess() {
    Q_ = process_noise_cov();
    // 初始化加速度计和陀螺仪测量协方差（经验值）
    cov_acc_ = Eigen::Vector3d(0.1, 0.1, 0.1);
    cov_gyr_ = Eigen::Vector3d(0.1, 0.1, 0.1);
    // 初始化零偏随机游走协方差（较小的值）
    cov_bias_gyr_ = Eigen::Vector3d(0.0001, 0.0001, 0.0001);
    cov_bias_acc_ = Eigen::Vector3d(0.0001, 0.0001, 0.0001);
    // 假设IMU静止，加速度计测量值应该是-g（重力方向向下）
    mean_acc_ = Eigen::Vector3d(0, 0, -1.0);
    // 陀螺仪均值初始化为0（静止状态）
    mean_gyr_ = Eigen::Vector3d(0, 0, 0);
    // 外参初始化为单位变换（假设激光雷达和IMU重合）
    Lidar_T_wrt_IMU_ = Eigen::Vector3d::Zero();
    Lidar_R_wrt_IMU_ = Eigen::Matrix3d::Identity();

    // 设置状态标志
    init_iter_num_ = 1;
    b_first_frame_ = true;
    imu_need_init_ = true;
}

/**
 * @brief 析构函数
 */
ImuProcess::~ImuProcess() {}

/**
 * @brief 重置IMU处理器到初始状态
 * 
 * 用于重新初始化，例如：
 * - 系统重启
 * - 检测到大的跳变或异常
 * - 需要重新标定
 */
void ImuProcess::Reset() {
    mean_acc_ = Eigen::Vector3d(0, 0, -1.0);
    mean_gyr_ = Eigen::Vector3d(0, 0, 0);
    init_iter_num_ = 1;
    b_first_frame_ = true;
    imu_need_init_ = true;
}

/**
 * @brief 设置激光雷达相对于IMU的外参标定结果
 * @param transl 平移向量 T_I^L (IMU坐标系下表示的激光雷达原点)
 * @param rot 旋转矩阵 R_I^L (从IMU到激光雷达的旋转)
 */
void ImuProcess::SetExtrinsic(const Eigen::Vector3d &transl, const Eigen::Matrix3d &rot) {
    Lidar_T_wrt_IMU_ = transl;
    Lidar_R_wrt_IMU_ = rot;
}

/**
 * @brief 设置陀螺仪测量噪声协方差的缩放因子
 * @param scaler 缩放向量，分别对应x, y, z轴
 */
void ImuProcess::SetGyrCov(const Eigen::Vector3d &scaler) { 
    cov_gyr_scale_ = scaler; 
}

/**
 * @brief 设置加速度计测量噪声协方差的缩放因子
 * @param scaler 缩放向量，分别对应x, y, z轴
 */
void ImuProcess::SetAccCov(const Eigen::Vector3d &scaler) { 
    cov_acc_scale_ = scaler; 
}

/**
 * @brief 设置陀螺仪零偏的随机游走协方差
 * @param b_g 零偏协方差向量，分别对应x, y, z轴
 */
void ImuProcess::SetGyrBiasCov(const Eigen::Vector3d &b_g) { 
    cov_bias_gyr_ = b_g; 
}

/**
 * @brief 设置加速度计零偏的随机游走协方差
 * @param b_a 零偏协方差向量，分别对应x, y, z轴
 */
void ImuProcess::SetAccBiasCov(const Eigen::Vector3d &b_a) { 
    cov_bias_acc_ = b_a; 
}

/**
 * @brief IMU初始化函数 - 在静止状态下估计初始状态
 * 
 * 该函数执行以下操作：
 * 1. 收集静止状态下的IMU数据
 * 2. 计算加速度计和陀螺仪的均值和协方差（在线更新）
 * 3. 使用加速度计均值估计重力方向
 * 4. 使用陀螺仪均值估计零偏
 * 5. 初始化卡尔曼滤波器的状态向量和协方差矩阵
 * 
 * @param meas 测量数据组，包含IMU数据队列
 * @param kf_state 卡尔曼滤波器状态对象
 * @param N 已处理的IMU帧数（引用传递，会被更新）
 */
void ImuProcess::IMUInit(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 30, input_ikfom> &kf_state, int &N) {
    Eigen::Vector3d cur_acc, cur_gyr;

    // 如果是第一帧，重置所有统计量
    if (b_first_frame_) {
        Reset();
        N = 1;
        b_first_frame_ = false;
        // 使用第一帧IMU数据初始化均值
        const auto &imu_acc = meas.imu_.front()->linear_acceleration;
        const auto &gyr_acc = meas.imu_.front()->angular_velocity;
        mean_acc_ << imu_acc.x, imu_acc.y, imu_acc.z;
        mean_gyr_ << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    }

    // 遍历所有IMU测量数据，在线更新均值和协方差
    for (const auto &imu : meas.imu_) {
        const auto &imu_acc = imu->linear_acceleration;
        const auto &gyr_acc = imu->angular_velocity;
        cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

        // 在线更新均值：mean_new = mean_old + (x - mean_old) / N
        mean_acc_ += (cur_acc - mean_acc_) / N;
        mean_gyr_ += (cur_gyr - mean_gyr_) / N;

        // 在线更新协方差（使用Welford算法的变体）
        // cov = E[(x - mean)^2]
        cov_acc_ = cov_acc_ * (N - 1.0) / N + (cur_acc - mean_acc_).cwiseProduct(cur_acc - mean_acc_) * (N - 1.0) / (N * N);
        cov_gyr_ = cov_gyr_ * (N - 1.0) / N + (cur_gyr - mean_gyr_).cwiseProduct(cur_gyr - mean_gyr_) * (N - 1.0) / (N * N);

        N++;
    }

    // 设置卡尔曼滤波器的初始状态
    state_ikfom init_state = kf_state.get_x();
    // 重力方向：将加速度计均值归一化并乘以重力加速度大小
    init_state.gravity = vect3(-mean_acc_ / mean_acc_.norm() * G_m_s2);
    // 陀螺仪零偏初始化为均值
    init_state.bg = mean_gyr_;
    // 设置激光雷达到IMU的外参
    init_state.offset_T_L_I = Lidar_T_wrt_IMU_;
    init_state.offset_R_L_I = Lidar_R_wrt_IMU_;
    kf_state.change_x(init_state);

    // 初始化卡尔曼滤波器的协方差矩阵P
    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
    init_P.setIdentity();
    // 位置协方差 (索引6,7,8)
    init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
    // 速度协方差 (索引9,10,11)
    init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
    // 陀螺仪零偏协方差 (索引15,16,17)
    init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
    // 加速度计零偏协方差 (索引18,19,20)
    init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
    // 重力协方差 (索引21,22)
    init_P(21, 21) = init_P(22, 22) = 0.00001;
    kf_state.change_P(init_P);
}

/**
 * @brief 处理IMU和激光雷达测量数据的主函数
 * 
 * 该函数执行以下操作：
 * 1. 检查IMU数据是否为空
 * 2. 如果需要初始化，则执行IMU初始化
 * 3. 使用IMU数据进行前向传播（预测步骤）
 * 4. 对点云进行运动补偿（去畸变）
 * 
 * @param meas 测量数据组，包含激光雷达点云和IMU数据队列
 * @param kf_state 误差状态卡尔曼滤波器（ESKF）对象
 * @param cur_pcl_un_ 去畸变后的点云输出
 */
void ImuProcess::Process(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 30, input_ikfom> &kf_state,
                         PointCloudType::Ptr cur_pcl_un_) {
    // 检查IMU数据是否为空
    if(meas.imu_.empty()) {
        return;
    }

    // 如果IMU需要初始化
    if(imu_need_init_) {
        IMUInit(meas, kf_state, init_iter_num_);

        state_ikfom imu_state = kf_state.get_x();
        if (init_iter_num_ > MAX_INI_COUNT) {
            // 对加速度计协方差进行重力归一化修正
            cov_acc_ *= pow(G_m_s2 / mean_acc_.norm(), 2);
            // 初始化完成
            imu_need_init_ = false;
            // 用配置的协方差值替换在线估计的协方差
            cov_acc_ = cov_acc_scale_;
            cov_gyr_ = cov_gyr_scale_;
        }
        return;
    }

    // TODO: 实现IMU前向传播
    // 使用IMU测量进行状态预测：
    // 1. 遍历IMU数据队列
    // 2. 对每个IMU数据调用kf_state.predict()进行预测
    // 3. 更新状态（位置、速度、姿态）

    // TODO: 实现点云去畸变
    // 根据IMU积分的位姿，对点云进行运动补偿：
    // 1. 计算每个点的时间戳
    // 2. 根据时间戳插值得到该点对应的位姿
    // 3. 将点变换到统一的参考坐标系（通常是扫描结束时刻）
    // 4. 输出去畸变后的点云到cur_pcl_un_
}

/**
 * @brief 生成过程噪声协方差矩阵
 * 
 * 过程噪声协方差矩阵Q用于卡尔曼滤波器的预测步骤
 * 包含12个状态的噪声：
 * - 速度噪声 (3维)
 * - 陀螺仪噪声 (3维)
 * - 加速度计噪声 (3维)
 * - 陀螺仪零偏噪声 (3维)
 * 
 * @return 12x12的过程噪声协方差矩阵
 */
Eigen::Matrix<double, 12, 12> ImuProcess::process_noise_cov() {
    Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Zero();
    
    // 速度噪声 (0-2)
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr_;
    // 陀螺仪测量噪声 (3-5)
    Q.block<3, 3>(3, 3).diagonal() = cov_gyr_;
    // 加速度计测量噪声 (6-8)
    Q.block<3, 3>(6, 6).diagonal() = cov_acc_;
    // 陀螺仪零偏噪声 (9-11)
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_gyr_;
    
    return Q;
}

}  // namespace d_point_lio
