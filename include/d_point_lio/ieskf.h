/**
 * @file ieskf.h
 * @brief Incremental Error State Kalman Filter (IESKF) implementation for Point-LIO
 *
 * This class implements a simplified IESKF algorithm for LiDAR-IMU odometry,
 * following the algorithm flow from Point-LIO project. It provides a practical
 * implementation using standard Eigen matrix operations rather than complex
 * manifold toolkits.
 *
 * Key Features:
 * - SO(3) rotation representation using rotation matrices
 * - IMU prediction with motion model
 * - LiDAR measurement update with point-to-plane matching
 * - Error state estimation and correction
 * - Automatic covariances adjustment
 *
 * Algorithm Flow:
 * 1. State Prediction: IMU integration for state propagation
 * 2. LiDAR Update: Point cloud registration for pose correction
 * 3. State Correction: Error state injection and covariance update
 */

#ifndef POINT_LIO_IESKF_H
#define POINT_LIO_IESKF_H

#include <Eigen/Eigen>
#include <so3_math.h>
#include "d_point_lio/common_data.h"

namespace d_point_lio {

/**
 * @brief Simplified IESKF state structure
 *
 * This structure contains all the state variables needed for LiDAR-IMU odometry:
 * - Position (3D)
 * - Rotation (3x3 matrix)
 * - Velocity (3D)
 * - Accelerometer bias (3D)
 * - Gyroscope bias (3D)
 * - LiDAR extrinsic calibration (6D)
 * - Gravity vector (3D)
 */
struct IESKFState {
    Vec3 pos;           // Position [x, y, z]
    Mat33 rot;          // Rotation matrix
    Vec3 vel;           // Velocity [vx, vy, vz]
    Vec3 ba;            // Accelerometer bias
    Vec3 bg;            // Gyroscope bias
    Mat33 offset_R_L_I;  // LiDAR to IMU rotation
    Vec3 offset_T_L_I;  // LiDAR to IMU translation
    Vec3 gravity;       // Gravity vector
};

/**
 * @brief Simplified IESKF class for LiDAR-IMU odometry
 *
 * This class implements the Incremental Error State Kalman Filter following Point-LIO
 * methodology using standard Eigen operations for better understandability.
 */
class IESKF {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     * @brief Constructor
     * @param init_pose Initial pose [x, y, z, roll, pitch, yaw]
     * @param init_vel Initial velocity [vx, vy, vz]
     * @param gravity Gravity vector [gx, gy, gz]
     */
    IESKF(const VectorXd& init_pose = VectorXd::Zero(6),
           const Vec3& init_vel = Vec3::Zero(),
           const Vec3& gravity = CommonConstants::DEFAULT_GRAVITY);

    /**
     * @brief Destructor
     */
    ~IESKF() = default;

    /**
     * @brief Initialize the filter with initial state and covariances
     * @param init_cov Initial state covariance
     * @param process_noise Process noise covariance
     * @param meas_noise Measurement noise covariance
     */
    void Initialize(const Matrix<double, 24, 24>& init_cov = Matrix<double, 24, 24>::Identity() * 0.1,
                    const Matrix<double, 24, 24>& process_noise = Matrix<double, 24, 24>::Identity() * 1e-3,
                    const Matrix<double, 24, 24>& meas_noise = Matrix<double, 24, 24>::Identity() * 1e-2);

    /**
     * @brief Predict state using IMU measurements
     * @param acc Linear acceleration measurement (m/s²)
     * @param gyro Angular velocity measurement (rad/s)
     * @param dt Time interval (seconds)
     */
    void Predict(const Vec3& acc, const Vec3& gyro, double dt);

    /**
     * @brief Update state using IMU measurements
     * @param acc Linear acceleration measurement (m/s²)
     * @param gyro Angular velocity measurement (rad/s)
     * @param acc_cov Accelerometer covariance
     * @param gyro_cov Gyroscope covariance
     */
    void UpdateWithIMU(const Vec3& acc, const Vec3& gyro,
                       const Mat33& acc_cov, const Mat33& gyro_cov);

    /**
     * @brief Update state using LiDAR measurements
     * @param points LiDAR points in body frame
     * @param plane_points Corresponding plane points in world frame
     * @param normals Plane normals in world frame
     * @param point_cov Point measurement covariance
     */
    void UpdateWithLiDAR(const std::vector<Vec3>& points,
                        const std::vector<Vec3>& plane_points,
                        const std::vector<Vec3>& normals,
                        double point_cov = 0.01);

    /**
     * @brief Get current state
     * @return Current state structure
     */
    IESKFState GetState() const;

    /**
     * @brief Get current pose
     * @return Current pose [x, y, z, qx, qy, qz, qw]
     */
    VectorXd GetPose() const;

    /**
     * @brief Get current position
     * @return Current position [x, y, z]
     */
    Vec3 GetPosition() const;

    /**
     * @brief Get current rotation matrix
     * @return Current rotation matrix
     */
    Mat33 GetRotation() const;

    /**
     * @brief Get current velocity
     * @return Current velocity [vx, vy, vz]
     */
    Vec3 GetVelocity() const;

    /**
     * @brief Get current biases
     * @return Pair of [accelerometer_bias, gyroscope_bias]
     */
    std::pair<Vec3, Vec3> GetBiases() const;

    /**
     * @brief Get state covariance
     * @return State covariance matrix
     */
    Matrix<double, 24, 24> GetCovariance() const;

    /**
     * @brief Reset filter to initial state
     */
    void Reset();

    /**
     * @brief Set extrinsic calibration
     * @param T_L_I Translation from LiDAR to IMU [x, y, z]
     * @param R_L_I Rotation from LiDAR to IMU (3x3 matrix)
     */
    void SetExtrinsic(const Vec3& T_L_I, const Mat33& R_L_I);

    /**
     * @brief Get extrinsic calibration
     * @return Pair of [translation, rotation_matrix]
     */
    std::pair<Vec3, Mat33> GetExtrinsic() const;

    /**
     * @brief Check if filter is initialized
     * @return True if initialized
     */
    bool IsInitialized() const;

private:
    // Current state
    IESKFState state_;

    // Covariance matrices
    Matrix<double, 24, 24> P_;  // State covariance
    Matrix<double, 24, 24> Q_;  // Process noise

    // Noise parameters
    Mat33 acc_noise_cov_;
    Mat33 gyro_noise_cov_;
    Mat33 point_noise_cov_;

    // State flags
    bool initialized_;

    // Process model functions (simplified, using direct IMU inputs)
    void processModel(double dt, const Vec3& acc, const Vec3& gyro);
    void processJacobian(double dt, const Vec3& acc, const Vec3& gyro);

    // Measurement model functions (simplified)
    void imuMeasurementModel(const Vec3& acc, const Vec3& gyro);
    void imuMeasurementJacobian(const Vec3& acc, const Vec3& gyro);

    void lidarMeasurementModel(const std::vector<Vec3>& points,
                              const std::vector<Vec3>& plane_points,
                              const std::vector<Vec3>& normals);
    void lidarMeasurementJacobian(const std::vector<Vec3>& points,
                                 const std::vector<Vec3>& plane_points,
                                 const std::vector<Vec3>& normals);

    // Utility functions (simplified for Eigen rotation matrices)
    Vec4 rotationToQuaternion(const Mat33& rot) const;
    Mat33 quaternionToRotation(const Vec4& quat) const;
    Mat33 skewMatrix(const Vec3& vec) const;

    // State initialization
    void initializeState(const VectorXd& init_pose, const Vec3& init_vel, const Vec3& gravity);
};

} // namespace d_point_lio

#endif // POINT_LIO_IESKF_H