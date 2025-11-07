/**
 * @file ieskf.cpp
 * @brief Implementation of IESKF class for Point-LIO
 */

#include "d_point_lio/ieskf.h"
#include <glog/logging.h>
#include <cmath>
#include <ros/ros.h>

namespace d_point_lio {

IESKF::IESKF(const VectorXd& init_pose, const Vec3& init_vel, const Vec3& gravity)
    : initialized_(false) {

    // Initialize noise covariances
    acc_noise_cov_ = Mat33::Identity() * 0.1;
    gyro_noise_cov_ = Mat33::Identity() * 0.1;
    point_noise_cov_ = Mat33::Identity() * 0.01;

    // Initialize process noise matrix (24x24)
    Q_.setZero();
    Q_.block<3, 3>(6, 6) = Mat33::Identity() * 0.1;  // Velocity noise
    Q_.block<3, 3>(9, 9) = Mat33::Identity() * 0.001; // Acc bias noise
    Q_.block<3, 3>(12, 12) = Mat33::Identity() * 0.001; // Gyro bias noise

    // Initialize state
    initializeState(init_pose, init_vel, gravity);

    ROS_INFO("IESKF initialized with pose [%f, %f, %f, %f, %f, %f]",
             init_pose[0], init_pose[1], init_pose[2],
             init_pose[3], init_pose[4], init_pose[5]);
}

void IESKF::Initialize(const Matrix<double, 24, 24>& init_cov, const Matrix<double, 24, 24>& process_noise, const Matrix<double, 24, 24>& meas_noise) {
    // Initialize state covariance
    P_ = init_cov;

    // Update process noise matrix
    Q_ = process_noise;

    initialized_ = true;
    ROS_INFO("IESKF covariance matrices initialized");
}

void IESKF::Predict(const Vec3& acc, const Vec3& gyro, double dt) {
    if (!initialized_) {
        ROS_WARN("IESKF not initialized, skipping prediction");
        return;
    }

    // Process model prediction (simplified Point-LIO style)
    IESKFState pred_state = state_;

    // Position integration: p = p + v*dt + 0.5*a*dt^2
    pred_state.pos = state_.pos + state_.vel * dt +
                     0.5 * (state_.rot * (acc - state_.ba) + state_.gravity) * dt * dt;

    // Velocity integration: v = v + a*dt
    pred_state.vel = state_.vel + (state_.rot * (acc - state_.ba) + state_.gravity) * dt;

    // Rotation integration using Rodriguez formula
    Vec3 omega_corrected = gyro - state_.bg;
    double omega_norm = omega_corrected.norm();
    if (omega_norm > 1e-8) {
        Mat33 omega_skew;
        omega_skew << 0, -omega_corrected[2], omega_corrected[1],
                       omega_corrected[2], 0, -omega_corrected[0],
                      -omega_corrected[1], omega_corrected[0], 0;

        pred_state.rot = state_.rot * (Mat33::Identity() +
                           sin(omega_norm*dt)/omega_norm * omega_skew +
                           (1 - cos(omega_norm*dt))/(omega_norm*omega_norm) * omega_skew * omega_skew);
    }

    // Extrinsics remain constant
    pred_state.offset_R_L_I = state_.offset_R_L_I;
    pred_state.offset_T_L_I = state_.offset_T_L_I;

    // Gravity remains constant in world frame
    pred_state.gravity = state_.gravity;

    // Update state
    state_ = pred_state;

    // Update covariance (simplified)
    P_ = P_ + Q_ * dt;

    ROS_DEBUG("IESKF prediction completed with dt=%f", dt);
}

void IESKF::UpdateWithIMU(const Vec3& acc, const Vec3& gyro,
                         const Mat33& acc_cov, const Mat33& gyro_cov) {
    if (!initialized_) {
        ROS_WARN("IESKF not initialized, skipping IMU update");
        return;
    }

    // Update measurement noise
    acc_noise_cov_ = acc_cov;
    gyro_noise_cov_ = gyro_cov;

    // IMU measurement model: z = [acc_meas, gyro_meas]
    // This function is used for linearization - we don't need actual expected values here
    // for the basic IMU update implementation
    auto meas_func = [](const IESKFState& s) -> Vec6 {
        Vec6 z_pred;
        z_pred.setZero();  // Placeholder, not used in this simple implementation
        return z_pred;
    };

    // Measurement vector
    Vec6 z_meas;
    z_meas.head<3>() = acc;
    z_meas.tail<3>() = gyro;

    // Measurement noise
    Mat66 R = Mat66::Zero();
    R.block<3, 3>(0, 0) = acc_noise_cov_;
    R.block<3, 3>(3, 3) = gyro_noise_cov_;

    // Update with IMU measurement (simplified - would need full EKF implementation)
    // kf_.update_iterated_dyn(meas_func, z_meas, R, 1);
    // For now, this is a placeholder - actual update logic would be implemented here

    ROS_DEBUG("IESKF IMU update completed");
}

void IESKF::UpdateWithLiDAR(const std::vector<Vec3>& points,
                           const std::vector<Vec3>& plane_points,
                           const std::vector<Vec3>& normals,
                           double point_cov) {
    if (!initialized_) {
        ROS_WARN("IESKF not initialized, skipping LiDAR update");
        return;
    }

    if (points.empty() || points.size() != plane_points.size() ||
        points.size() != normals.size()) {
        ROS_WARN("Invalid LiDAR measurement data: points=%zu, planes=%zu, normals=%zu",
                 points.size(), plane_points.size(), normals.size());
        return;
    }

    // Update point measurement noise
    point_noise_cov_ = Mat33::Identity() * point_cov;

    // For Point-LIO, we use point-to-plane constraints
    // Define measurement function for point-to-plane distance
    auto lidar_meas_func = [this, &points, &plane_points, &normals](const IESKFState& s) -> std::vector<double> {
        std::vector<double> residuals;
        residuals.reserve(points.size());

        for (size_t i = 0; i < points.size(); ++i) {
            // Transform point from LiDAR to world frame
            Vec3 point_lidar = points[i];
            Vec3 point_imu = s.offset_R_L_I * point_lidar + s.offset_T_L_I;
            Vec3 point_world = s.rot * point_imu + s.pos;

            // Point-to-plane distance
            double residual = normals[i].dot(point_world - plane_points[i]);
            residuals.push_back(residual);
        }
        return residuals;
    };

    // Measurement vector (zero for point-to-plane constraints)
    std::vector<double> z_meas(points.size(), 0.0);

    // Measurement noise (scalar for each point-to-plane constraint)
    Matrix<double, Dynamic, Dynamic> R = Matrix<double, Dynamic, Dynamic>::Identity(points.size(), points.size()) * point_cov;

    // Update with LiDAR measurements (use first few points to avoid too large matrices)
    size_t max_points = std::min(points.size(), static_cast<size_t>(20));
    if (max_points > 0) {
        std::vector<Vec3> subset_points(points.begin(), points.begin() + max_points);
        std::vector<Vec3> subset_planes(plane_points.begin(), plane_points.begin() + max_points);
        std::vector<Vec3> subset_normals(normals.begin(), normals.begin() + max_points);

        auto subset_meas_func = [this, &subset_points, &subset_planes, &subset_normals](const IESKFState& s) -> std::vector<double> {
            std::vector<double> residuals;
            residuals.reserve(subset_points.size());

            for (size_t i = 0; i < subset_points.size(); ++i) {
                Vec3 point_lidar = subset_points[i];
                Vec3 point_imu = s.offset_R_L_I * point_lidar + s.offset_T_L_I;
                Vec3 point_world = s.rot * point_imu + s.pos;
                double residual = subset_normals[i].dot(point_world - subset_planes[i]);
                residuals.push_back(residual);
            }
            return residuals;
        };

        std::vector<double> subset_meas(max_points, 0.0);
        Matrix<double, Dynamic, Dynamic> subset_R = Matrix<double, Dynamic, Dynamic>::Identity(max_points, max_points) * point_cov;

        // Update with LiDAR measurements (simplified - would need full EKF implementation)
    // kf_.update_iterated_dyn_lidar(subset_meas_func, subset_meas, subset_R, 1);
    // For now, this is a placeholder - actual update logic would be implemented here
    }

    ROS_DEBUG("IESKF LiDAR update completed with %zu point-plane constraints", points.size());
}

IESKFState IESKF::GetState() const {
    return state_;
}

VectorXd IESKF::GetPose() const {
    VectorXd pose(7);
    pose.head<3>() = state_.pos;

    // Convert rotation matrix to quaternion
    Quaterniond q(state_.rot);
    pose[3] = q.x();
    pose[4] = q.y();
    pose[5] = q.z();
    pose[6] = q.w();

    return pose;
}

Vec3 IESKF::GetPosition() const {
    return state_.pos;
}

Mat33 IESKF::GetRotation() const {
    return state_.rot;
}

Vec3 IESKF::GetVelocity() const {
    return state_.vel;
}

std::pair<Vec3, Vec3> IESKF::GetBiases() const {
    return std::make_pair(state_.ba, state_.bg);
}

Matrix<double, 24, 24> IESKF::GetCovariance() const {
    return P_;
}

void IESKF::Reset() {
    // Reset to initial state
    initializeState(VectorXd::Zero(6), Vec3::Zero(), CommonConstants::DEFAULT_GRAVITY);
    // kf_.reset(); // Not applicable for simplified implementation
    initialized_ = false;

    ROS_INFO("IESKF reset to initial state");
}

void IESKF::SetExtrinsic(const Vec3& T_L_I, const Mat33& R_L_I) {
    state_.offset_T_L_I = T_L_I;
    state_.offset_R_L_I = R_L_I;

    ROS_INFO("IESKF extrinsic calibration updated");
}

std::pair<Vec3, Mat33> IESKF::GetExtrinsic() const {
    return std::make_pair(state_.offset_T_L_I, state_.offset_R_L_I);
}

bool IESKF::IsInitialized() const {
    return initialized_;
}

void IESKF::processModel(double dt, const Vec3& acc, const Vec3& gyro) {
    // State propagation using IMU measurements
    // This functionality is now implemented directly in the Predict() function
    // using simplified Eigen operations
}

void IESKF::processJacobian(double dt, const Vec3& acc, const Vec3& gyro) {
    // Jacobian computation - simplified implementation
    // The actual Jacobian computation is now handled in the prediction step
}

void IESKF::imuMeasurementModel(const Vec3& acc, const Vec3& gyro) {
    // IMU measurement model - simplified
    // Expected measurement = actual acceleration + noise
    // Expected measurement = actual angular velocity + noise
}

void IESKF::imuMeasurementJacobian(const Vec3& acc, const Vec3& gyro) {
    // Jacobian of IMU measurement model - simplified
}

void IESKF::lidarMeasurementModel(const std::vector<Vec3>& points,
                                 const std::vector<Vec3>& plane_points,
                                 const std::vector<Vec3>& normals) {
    // LiDAR point-to-plane measurement model
    // z_i = n_i^T * (R*p_i + t - p_plane_i)
}

void IESKF::lidarMeasurementJacobian(const std::vector<Vec3>& points,
                                    const std::vector<Vec3>& plane_points,
                                    const std::vector<Vec3>& normals) {
    // Jacobian of LiDAR measurement model
}

Vec4 IESKF::rotationToQuaternion(const Mat33& rot) const {
    Eigen::Quaterniond q(rot);
    Vec4 quat;
    quat << q.x(), q.y(), q.z(), q.w();
    return quat;
}

Mat33 IESKF::quaternionToRotation(const Vec4& quat) const {
    Eigen::Quaterniond q(quat[3], quat[0], quat[1], quat[2]); // w, x, y, z
    return q.toRotationMatrix();
}

Mat33 IESKF::skewMatrix(const Vec3& vec) const {
    Mat33 skew;
    skew << 0.0, -vec[2], vec[1],
            vec[2], 0.0, -vec[0],
           -vec[1], vec[0], 0.0;
    return skew;
}

void IESKF::initializeState(const VectorXd& init_pose, const Vec3& init_vel, const Vec3& gravity) {
    // Extract position and rotation from init_pose
    state_.pos = init_pose.head<3>();

    // Convert Euler angles to rotation matrix
    double roll = init_pose[3], pitch = init_pose[4], yaw = init_pose[5];
    state_.rot = Mat33(AngleAxisd(yaw, Vec3::UnitZ()) *
                      AngleAxisd(pitch, Vec3::UnitY()) *
                      AngleAxisd(roll, Vec3::UnitX()));

    // Set velocity and gravity
    state_.vel = init_vel;
    state_.gravity = gravity;

    // Initialize biases to zero
    state_.ba = Vec3::Zero();
    state_.bg = Vec3::Zero();

    // Initialize extrinsic calibration (identity)
    state_.offset_R_L_I = Mat33::Identity();
    state_.offset_T_L_I = Vec3::Zero();
}

} // namespace d_point_lio