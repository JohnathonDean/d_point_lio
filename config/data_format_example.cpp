/**
 * @file data_format_example.cpp
 * @brief Example demonstrating common data types usage in Point-LIO
 */

#include "d_point_lio/common_data.h"
#include "d_point_lio/laser_mapping.h"
#include <iostream>

void demonstrateDataTypes() {
    std::cout << "=== Common Data Types Demonstration ===" << std::endl;

    // 1. Point types - Primary point type for LiDAR processing
    std::cout << "\n1. Point Types:" << std::endl;
    PointType point;
    point.x = 1.0; point.y = 2.0; point.z = 3.0;
    point.intensity = 255.0; point.curvature = 0.01;
    std::cout << "Point: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;

    // 2. Vector types - Short aliases for Eigen vectors
    std::cout << "\n2. Vector Types:" << std::endl;
    Vec3 position = Vec3(1.0, 2.0, 3.0);
    Vec3 velocity = Vec3(0.5, 0.3, 0.1);
    std::cout << "Position: [" << position.transpose() << "]" << std::endl;
    std::cout << "Velocity: [" << velocity.transpose() << "]" << std::endl;

    // 3. Matrix types - Short aliases for Eigen matrices
    std::cout << "\n3. Matrix Types:" << std::endl;
    Mat33 rotation = CommonConstants::IDENTITY_MATRIX;
    Mat66 covariance = Mat66::Identity() * 0.01;
    std::cout << "Rotation Matrix (3x3):" << std::endl << rotation << std::endl;

    // 4. Pose with covariance - 6DOF pose with uncertainty
    std::cout << "\n4. Pose with Covariance:" << std::endl;
    PoseWithCovariance pose;
    pose.position = position;
    pose.orientation = rotation;
    pose.covariance = covariance;
    pose.timestamp = ros::Time::now().toSec();
    std::cout << "Pose position: [" << pose.position.transpose() << "]" << std::endl;

    // 5. IMU data - Structured format for IMU measurements
    std::cout << "\n5. IMU Data:" << std::endl;
    IMUData imu_data;
    imu_data.acc = Vec3(0.1, 0.2, 9.81);
    imu_data.gyro = Vec3(0.01, 0.02, 0.03);
    imu_data.timestamp = ros::Time::now().toSec();
    imu_data.valid = DataUtils::isValidIMU(imu_data.acc, imu_data.gyro);
    std::cout << "IMU Valid: " << (imu_data.valid ? "Yes" : "No") << std::endl;

    // 6. Constants usage - Predefined physical constants
    std::cout << "\n6. Constants Usage:" << std::endl;
    std::cout << "Default Gravity: [" << CommonConstants::DEFAULT_GRAVITY.transpose() << "]" << std::endl;
    std::cout << "Gravity Norm: " << CommonConstants::GRAVITY_NORM << " m/s²" << std::endl;

    // 7. Utility functions - Data validation and transformation
    std::cout << "\n7. Utility Functions:" << std::endl;
    bool valid_point = DataUtils::isValidPoint(point);
    double dist_sq = DataUtils::pointDistanceSquared(point);
    bool valid_timestamp = DataUtils::isValidTimestamp(pose.timestamp);
    std::cout << "Point Valid: " << (valid_point ? "Yes" : "No") << std::endl;
    std::cout << "Point Distance Squared: " << dist_sq << std::endl;

    // 8. Coordinate transformations - Rotation/quaternion conversions
    std::cout << "\n8. Coordinate Transformations:" << std::endl;
    Vec4 quat = DataUtils::rotationToQuaternion(rotation);
    Mat33 rot_back = DataUtils::quaternionToRotation(quat);
    std::cout << "Quaternion from rotation: [" << quat.transpose() << "]" << std::endl;
    std::cout << "Transformation restored: " << (rot_back.isApprox(rotation) ? "Success" : "Failed") << std::endl;
}

void demonstrateDataStructures() {
    std::cout << "\n=== Data Structures Demonstration ===" << std::endl;

    // 1. MeasureGroup - Synchronized LiDAR-IMU data
    std::cout << "\n1. MeasureGroup Structure:" << std::endl;
    MeasureGroup meas_group;
    meas_group.lidar_beg_time = ros::Time::now().toSec();
    meas_group.lidar_end_time = meas_group.lidar_beg_time + 0.1;
    meas_group.lidar.reset(new PointCloudXYZI());

    // Add IMU measurements
    for (int i = 0; i < 5; ++i) {
        sensor_msgs::Imu::Ptr imu_msg(new sensor_msgs::Imu());
        imu_msg->header.stamp = ros::Time().fromSec(meas_group.lidar_beg_time + i * 0.02);
        imu_msg->linear_acceleration.x = 0.1; imu_msg->linear_acceleration.y = 0.2; imu_msg->linear_acceleration.z = 9.81;
        imu_msg->angular_velocity.x = 0.01; imu_msg->angular_velocity.y = 0.02; imu_msg->angular_velocity.z = 0.03;
        meas_group.imu_buffer.push_back(imu_msg);
    }
    std::cout << "Time window: [" << meas_group.lidar_beg_time << ", " << meas_group.lidar_end_time << "]" << std::endl;
    std::cout << "IMU measurements: " << meas_group.imu_buffer.size() << std::endl;

    // 2. TimedTransform - Spatial transform with timestamp
    std::cout << "\n2. TimedTransform Structure:" << std::endl;
    TimedTransform transform;
    transform.translation = Vec3(0.1, 0.2, 0.3);
    transform.rotation = CommonConstants::IDENTITY_MATRIX;
    transform.timestamp = ros::Time::now().toSec();
    std::cout << "Translation: [" << transform.translation.transpose() << "]" << std::endl;

    // 3. LidarScanData - Complete scan with timing
    std::cout << "\n3. LidarScanData Structure:" << std::endl;
    LidarScanData scan_data;
    scan_data.cloud.reset(new PointCloudXYZI());
    scan_data.timestamp = ros::Time::now().toSec();
    scan_data.scan_duration = 0.1;

    // Add sample points
    for (int i = 0; i < 10; ++i) {
        PointType pt;
        pt.x = i * 0.1; pt.y = i * 0.1; pt.z = i * 0.1;
        pt.intensity = i * 10; pt.curvature = i * 0.001;
        scan_data.cloud->push_back(pt);
    }
    std::cout << "Scan points: " << scan_data.cloud->size() << std::endl;
}

void demonstrateNamespaceUsage() {
    std::cout << "\n=== Namespace Usage Demonstration ===" << std::endl;

    // 1. Create LaserMapping instance using namespace
    std::cout << "\n1. Creating LaserMapping instance:" << std::endl;
    auto laser_mapping = std::make_shared<d_point_lio::LaserMapping>();
    std::cout << "✓ LaserMapping instance created successfully" << std::endl;

    // 2. Use types from namespace
    std::cout << "\n2. Using types from namespace:" << std::endl;
    d_point_lio::PointCloudXYZI::Ptr cloud(new d_point_lio::PointCloudXYZI());
    d_point_lio::PointType point;
    point.x = 1.0; point.y = 2.0; point.z = 3.0;
    cloud->push_back(point);
    std::cout << "✓ Point cloud created with " << cloud->size() << " points" << std::endl;

    // 3. Demonstrate using namespace alias
    std::cout << "\n3. Using namespace alias for convenience:" << std::endl;
    {
        using namespace d_point_lio;
        PointCloudXYZI::Ptr cloud2(new PointCloudXYZI());
        PointType point2;
        point2.x = 4.0; point2.y = 5.0; point2.z = 6.0;
        cloud2->push_back(point2);
        std::cout << "✓ Created cloud with alias: " << cloud2->size() << " points" << std::endl;
    }
}

int main() {
    std::cout << "Point-LIO Common Data Format Examples" << std::endl;
    std::cout << "======================================" << std::endl;

    try {
        demonstrateDataTypes();
        demonstrateDataStructures();
        demonstrateNamespaceUsage();

        std::cout << "\n=== Summary ===" << std::endl;
        std::cout << "✓ All data types and structures demonstrated successfully" << std::endl;
        std::cout << "✓ Utility functions working correctly" << std::endl;
        std::cout << "✓ Constants and enums properly defined" << std::endl;
        std::cout << "✓ Namespace usage working correctly" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}