/**
 * @file test_ieskf.cpp
 * @brief Test program for IESKF implementation
 */

#include "d_point_lio/ieskf.h"
#include <iostream>
#include <vector>

using namespace d_point_lio;

int main() {
    std::cout << "=== IESKF Test Program ===" << std::endl;

    // Initialize IESKF
    VectorXd init_pose = VectorXd::Zero(6);
    init_pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;  // [x, y, z, roll, pitch, yaw]
    Vec3 init_vel = Vec3::Zero();

    IESKF ieskf(init_pose, init_vel, CommonConstants::DEFAULT_GRAVITY);

    // Initialize filter
    Matrix<double, 24, 24> init_cov = Matrix<double, 24, 24>::Identity() * 0.1;
    Matrix<double, 24, 24> process_noise = Matrix<double, 24, 24>::Identity() * 1e-3;
    Matrix<double, 24, 24> meas_noise = Matrix<double, 24, 24>::Identity() * 1e-2;

    ieskf.Initialize(init_cov, process_noise, meas_noise);

    std::cout << "✓ IESKF initialized successfully" << std::endl;

    // Test basic functionality
    std::cout << "\n=== Testing Basic Functions ===" << std::endl;

    // Test initial state
    Vec3 pos = ieskf.GetPosition();
    std::cout << "Initial position: [" << pos.transpose() << "]" << std::endl;

    Mat33 rot = ieskf.GetRotation();
    std::cout << "Initial rotation matrix:" << std::endl << rot << std::endl;

    Vec3 vel = ieskf.GetVelocity();
    std::cout << "Initial velocity: [" << vel.transpose() << "]" << std::endl;

    // Test IMU prediction
    std::cout << "\n=== Testing IMU Prediction ===" << std::endl;

    Vec3 acc(0.1, 0.0, 9.81);  // Forward acceleration with gravity
    Vec3 gyro(0.0, 0.0, 0.1);   // Yaw rotation
    double dt = 0.01;             // 10ms

    for (int i = 0; i < 10; ++i) {
        ieskf.Predict(acc, gyro, dt);
        Vec3 current_pos = ieskf.GetPosition();
        std::cout << "Step " << i+1 << ": position = [" << current_pos.transpose() << "]" << std::endl;
    }

    // Test LiDAR update
    std::cout << "\n=== Testing LiDAR Update ===" << std::endl;

    std::vector<Vec3> points;
    std::vector<Vec3> plane_points;
    std::vector<Vec3> normals;

    // Create simple point-plane correspondences
    points.push_back(Vec3(1.0, 0.0, 0.0));
    plane_points.push_back(Vec3(1.0, 0.0, 0.0));
    normals.push_back(Vec3(0.0, 0.0, 1.0));

    ieskf.UpdateWithLiDAR(points, plane_points, normals, 0.01);

    Vec3 updated_pos = ieskf.GetPosition();
    std::cout << "Position after LiDAR update: [" << updated_pos.transpose() << "]" << std::endl;

    // Test extrinsic calibration
    std::cout << "\n=== Testing Extrinsic Calibration ===" << std::endl;

    Vec3 extrinsic_T(0.1, 0.05, -0.02);
    Mat33 extrinsic_R = Mat33::Identity();
    ieskf.SetExtrinsic(extrinsic_T, extrinsic_R);

    auto extrinsic = ieskf.GetExtrinsic();
    std::cout << "Extrinsic translation: [" << extrinsic.first.transpose() << "]" << std::endl;
    std::cout << "Extrinsic rotation matrix:" << std::endl << extrinsic.second << std::endl;

    // Test reset
    std::cout << "\n=== Testing Reset ===" << std::endl;
    ieskf.Reset();
    Vec3 reset_pos = ieskf.GetPosition();
    std::cout << "Position after reset: [" << reset_pos.transpose() << "]" << std::endl;

    std::cout << "\n✓ All tests completed successfully!" << std::endl;
    return 0;
}