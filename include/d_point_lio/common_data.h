/**
 * @file common_data.h
 * @brief Common data types and structures for Point-LIO implementation
 *
 * This header file provides unified, type-safe data formats for Point-LIO implementation.
 * It includes type definitions for point clouds, matrices, ESEKF states, synchronized data groups,
 * and utility functions for data validation and coordinate transformations.
 *
 * Key Features:
 * - Type-safe data definitions using typedef
 * - Standardized data structures for LiDAR-IMU synchronization
 * - Common constants and enumerations for sensors
 * - Utility functions for data validation and transformation
 *
 * Usage:
 *   #include "d_point_lio/common_data.h"
 *   Vec3 position = CommonConstants::DEFAULT_GRAVITY;
 *   PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
 *   bool valid = DataUtils::isValidPoint(point);
 */

#ifndef POINT_LIO_COMMON_DATA_H
#define POINT_LIO_COMMON_DATA_H

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <livox_ros_driver/CustomMsg.h>
#include <deque>
#include <memory>
#include <vector>

using namespace std;
using namespace Eigen;

// =============================================================================
// Point Type Definitions
// =============================================================================

/**
 * @brief Point type with intensity and normal for LiDAR processing
 *
 * This is the primary point type used throughout Point-LIO for processing
 * LiDAR point clouds with intensity and curvature information.
 */
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

/**
 * @brief Standard PCL point types
 *
 * These are commonly used PCL point types for compatibility with existing
 * PCL-based code and algorithms.
 */
typedef pcl::PointXYZI PointXYZI;
typedef pcl::PointCloud<PointXYZI> PointCloudI;

// =============================================================================
// Eigen Type Definitions
// =============================================================================

// =============================================================================
// MTK Types Section - Removed
// =============================================================================
// MTK manifold types have been removed as we now use simplified Eigen-based IESKF
// The following types are now defined directly in the IESKF class:
// - IESKFState: Structure containing all state variables
// - SO3 operations are handled using Eigen rotation matrices

/**
 * @brief Matrix and vector type aliases for convenience
 *
 * These provide shorter, more readable names for commonly used Eigen types.
 * Using these aliases makes the code more consistent and readable.
 */
typedef Matrix3d Mat33;                    ///< 3x3 matrix (rotation, covariance)
typedef Vector3d Vec3;                     ///< 3D vector (position, velocity, etc.)
typedef Matrix4d Mat44;                    ///< 4x4 matrix (homogeneous transform)
typedef Vector4d Vec4;                     ///< 4D vector (quaternion xyzw)
typedef Matrix<double, 6, 6> Mat66;        ///< 6x6 matrix (pose covariance)
typedef Matrix<double, 6, 1> Vec6;         ///< 6D vector (IMU measurement, twist)

// =============================================================================
// State and Input Definitions for ESEKF
// =============================================================================

// =============================================================================
// MTK Manifold Definitions - Removed
// =============================================================================
// MTK manifold definitions have been removed as we now use simplified Eigen-based IESKF
// The state structure is now defined in the IESKF class as IESKFState
// This simplifies the implementation and removes complex manifold dependencies

// =============================================================================
// Remaining MTK Definitions - Removed
// =============================================================================
// All MTK manifold definitions have been removed to simplify the implementation
// The IESKF class now uses standard Eigen types for state representation

// =============================================================================
// ESEKF Type Definitions
// =============================================================================

// =============================================================================
// ESEKF Type Definition - Removed
// =============================================================================
// esekfom type definition has been removed as we now use simplified Eigen-based IESKF
// The IESKF class provides its own state estimation functionality without external dependencies

// =============================================================================
// Message Type Definitions
// =============================================================================

/**
 * @brief ROS message pointer type definitions
 *
 * These provide convenient aliases for commonly used ROS message types,
 * improving code readability and consistency.
 */
typedef sensor_msgs::PointCloud2::ConstPtr PointCloud2ConstPtr;   ///< ROS point cloud message
typedef sensor_msgs::Imu::ConstPtr ImuConstPtr;                   ///< ROS IMU message
typedef nav_msgs::Odometry::Ptr OdometryPtr;                       ///< ROS odometry message
typedef nav_msgs::Path::Ptr PathPtr;                               ///< ROS path message
typedef geometry_msgs::PoseStamped::Ptr PoseStampedPtr;           ///< ROS pose message
typedef livox_ros_driver::CustomMsg::ConstPtr LivoxMsgConstPtr;   ///< Livox point cloud message

/**
 * @brief PCL cloud pointer type definitions
 *
 * Point cloud pointer types for memory management and data passing.
 */
typedef PointCloudXYZI::Ptr PointCloudXYZIPtr;                     ///< PointXYZI shared pointer
typedef PointCloudXYZI::ConstPtr PointCloudXYZIConstPtr;          ///< PointXYZI const shared pointer
typedef PointCloudI::Ptr PointCloudIPtr;                           ///< PointXYZI shared pointer

// =============================================================================
// Data Structure Definitions
// =============================================================================

/**
 * @brief Measurement group containing synchronized LiDAR and IMU data
 *
 * This structure is fundamental for Point-LIO data synchronization. It holds
 * a LiDAR scan and all corresponding IMU measurements within the same time
 * window, ensuring proper temporal alignment for sensor fusion.
 *
 * The IMU buffer is sorted by timestamp and contains all measurements from
 * the beginning to the end of the LiDAR scan, enabling accurate motion
 * compensation and state estimation.
 */
struct MeasureGroup {
    double lidar_beg_time;                                    ///< LiDAR scan start time (seconds)
    double lidar_end_time;                                    ///< LiDAR scan end time (seconds)
    double lidar_last_time;                                   ///< Last LiDAR timestamp in scan
    std::vector<ImuConstPtr> imu_buffer;                     ///< IMU measurements sorted by time
    PointCloudXYZI::Ptr lidar;                               ///< Processed LiDAR point cloud
};

/**
 * @brief LiDAR scan data with timestamp information
 *
 * Contains a complete LiDAR scan along with timing metadata. This structure
 * is useful for managing multiple scans, scan-to-scan operations, and time-based
 * data analysis.
 */
struct LidarScanData {
    PointCloudXYZI::Ptr cloud;                               ///< Point cloud data
    double timestamp;                                         ///< Scan acquisition timestamp
    double scan_duration;                                     ///< Total scan duration in seconds
};

/**
 * @brief IMU measurement data in convenient format
 *
 * Provides a structured format for IMU measurements with validation flags.
 * This structure is used for internal processing and data validation,
 * offering better type safety than raw sensor messages.
 */
struct IMUData {
    Vec3 acc;                                                 ///< Linear acceleration (m/s²)
    Vec3 gyro;                                                ///< Angular velocity (rad/s)
    double timestamp;                                         ///< Measurement timestamp (seconds)
    bool valid;                                               ///< Data validity flag
};

/**
 * @brief Pose estimate with uncertainty quantification
 *
 * Represents a 6DOF pose (position + orientation) along with its covariance
 * matrix. This is essential for understanding the confidence of pose estimates
 * and for proper uncertainty propagation in sensor fusion algorithms.
 *
 * The covariance matrix follows the order: [x, y, z, roll, pitch, yaw]
 */
struct PoseWithCovariance {
    Vec3 position;                                            ///< Position vector (x, y, z)
    Mat33 orientation;                                        ///< Rotation matrix (3x3)
    Mat66 covariance;                                         ///< 6x6 pose covariance matrix
    double timestamp;                                         ///< Pose timestamp (seconds)
};

/**
 * @brief Spatial transform with temporal information
 *
 * Represents a rigid body transformation (rotation + translation) with
 * timestamp. Useful for tracking sensor poses, coordinate frame
 * transformations, and time-varying extrinsic calibrations.
 */
struct TimedTransform {
    Vec3 translation;                                         ///< Translation vector
    Mat33 rotation;                                           ///< Rotation matrix
    double timestamp;                                         ///< Transform timestamp (seconds)
};

// =============================================================================
// Common Constants
// =============================================================================

/**
 * @brief Common constants namespace for Point-LIO
 *
 * Contains physical constants, sensor specifications, and validation thresholds
 * used throughout the Point-LIO implementation. Using these constants ensures
 * consistency and makes parameter tuning easier.
 */
namespace CommonConstants {
    // Physical constants
    constexpr double GRAVITY_NORM = 9.81;                     ///< Standard gravity (m/s²)

    // Coordinate frame conventions
    const Vec3 DEFAULT_GRAVITY = Vec3(0.0, 0.0, 9.81);       ///< Default gravity vector (pointing up)
    const Mat33 IDENTITY_MATRIX = Mat33::Identity();           ///< 3x3 identity matrix

    // Sensor specifications
    constexpr double IMU_FREQ_MIN = 50.0;                      ///< Minimum expected IMU frequency (Hz)
    constexpr double IMU_FREQ_MAX = 1000.0;                    ///< Maximum expected IMU frequency (Hz)
    constexpr double LIDAR_FREQ_MIN = 5.0;                     ///< Minimum expected LiDAR frequency (Hz)
    constexpr double LIDAR_FREQ_MAX = 100.0;                   ///< Maximum expected LiDAR frequency (Hz)

    // Data validation thresholds
    constexpr double MAX_VALID_ACCEL = 50.0;                   ///< Maximum valid acceleration (m/s²)
    constexpr double MAX_VALID_GYRO = 35.0;                    ///< Maximum valid angular velocity (rad/s)
    constexpr double MIN_VALID_DIST = 0.1;                     ///< Minimum valid distance from origin (m)
    constexpr double MAX_VALID_DIST = 500.0;                   ///< Maximum valid distance from origin (m)
}

// =============================================================================
// LiDAR Type Enumeration
// =============================================================================

/**
 * @brief LiDAR type enumeration for different sensor models
 *
 * Defines supported LiDAR sensor types for preprocessing and data handling.
 * Each type may have specific requirements for data format and processing.
 */
enum class LidarType {
    UNKNOWN = 0,                                             ///< Unknown sensor type
    LIVOX_AVIA = 1,                                         ///< Livox Avia/Mid-70/Tele-15
    VELODYNE = 2,                                           ///< Velodyne VLP, HDL, Puck series
    OUSTER = 3,                                             ///< Ouster OS1, OS2 series
    HESAI = 4,                                              ///< Hesai Pandar series
    CUSTOM = 99                                             ///< Custom LiDAR implementation
};

/**
 * @brief IMU mode enumeration for different processing strategies
 *
 * Defines how IMU data is used in the state estimation framework:
 * - DISABLED: IMU data is ignored
 * - AS_INPUT: IMU provides high-frequency motion input (use_imu_as_input=true)
 * - AS_MEASUREMENT: IMU provides low-frequency correction (use_imu_as_input=false)
 */
enum class IMUMode {
    DISABLED = 0,                                           ///< IMU data ignored
    AS_INPUT = 1,                                           ///< IMU as motion input
    AS_MEASUREMENT = 2                                      ///< IMU as measurement update
};

// =============================================================================
// Utility Functions
// =============================================================================

/**
 * @brief Data utility functions namespace
 *
 * Provides commonly used validation and transformation functions for
 * Point-LIO data processing. All functions are inline for performance
 * and include comprehensive documentation.
 */
namespace DataUtils {
    /**
     * @brief Check if ROS timestamp is valid
     *
     * Validates that timestamp is within reasonable bounds:
     * - Greater than 0 (not negative or zero)
     * - Less than 1e15 (before year 2001, after year 2286)
     *
     * @param timestamp Timestamp in seconds since epoch
     * @return true if timestamp is valid, false otherwise
     */
    inline bool isValidTimestamp(const double timestamp) {
        return timestamp > 0.0 && timestamp < 1e15;
    }

    /**
     * @brief Check if point is valid (not NaN or infinite)
     *
     * Validates that a 3D point has finite, real-valued coordinates.
     * This is essential before performing mathematical operations.
     *
     * @param point Point to validate
     * @return true if point coordinates are valid, false otherwise
     */
    inline bool isValidPoint(const PointType& point) {
        return !std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z) &&
               !std::isinf(point.x) && !std::isinf(point.y) && !std::isinf(point.z);
    }

    /**
     * @brief Calculate point distance squared from origin
     *
     * Computes squared Euclidean distance from origin to avoid square root
     * operations when comparing distances.
     *
     * @param point Point to calculate distance for
     * @return Squared distance from origin
     */
    inline double pointDistanceSquared(const PointType& point) {
        return point.x * point.x + point.y * point.y + point.z * point.z;
    }

    /**
     * @brief Check if IMU measurement is valid
     *
     * Validates IMU measurements against physical limits and checks for
     * invalid values (NaN, infinity). Uses CommonConstants for validation
     * thresholds.
     *
     * @param acc Linear acceleration vector (m/s²)
     * @param gyro Angular velocity vector (rad/s)
     * @return true if measurements are within valid ranges, false otherwise
     */
    inline bool isValidIMU(const Vec3& acc, const Vec3& gyro) {
        return acc.norm() < CommonConstants::MAX_VALID_ACCEL &&
               gyro.norm() < CommonConstants::MAX_VALID_GYRO &&
               !acc.hasNaN() && !gyro.hasNaN();
    }

    /**
     * @brief Convert rotation matrix to quaternion (x, y, z, w)
     *
     * Converts a 3x3 rotation matrix to quaternion representation.
     * Output quaternion follows (x, y, z, w) order for consistency.
     *
     * @param R 3x3 rotation matrix
     * @return Quaternion as [x, y, z, w] vector
     */
    inline Vec4 rotationToQuaternion(const Mat33& R) {
        Quaterniond q(R);
        return Vec4(q.x(), q.y(), q.z(), q.w());
    }

    /**
     * @brief Convert quaternion to rotation matrix
     *
     * Converts quaternion to 3x3 rotation matrix.
     * Input quaternion should be in (x, y, z, w) order.
     *
     * @param q Quaternion as [x, y, z, w] vector
     * @return 3x3 rotation matrix
     */
    inline Mat33 quaternionToRotation(const Vec4& q) {
        Quaterniond quat(q.w(), q.x(), q.y(), q.z());  // Note: w is last in Vec4
        return quat.toRotationMatrix();
    }
}

// =============================================================================
// Usage Examples and Best Practices
// =============================================================================

/**
 * @brief Common usage patterns for the data types and structures
 *
 * Basic usage example:
 * @code
 * #include "d_point_lio/common_data.h"
 * #include "d_point_lio/laser_mapping.h"
 *
 * // Create point cloud
 * PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
 *
 * // Use constants
 * Vec3 gravity = CommonConstants::DEFAULT_GRAVITY;
 * Mat33 identity = CommonConstants::IDENTITY_MATRIX;
 *
 * // Validate data
 * if (DataUtils::isValidPoint(point)) {
 *     // process point
 * }
 *
 * // Create LaserMapping instance using namespace
 * auto laser_mapping = std::make_shared<d_point_lio::LaserMapping>();
 *
 * // Or use namespace alias for convenience
 * using namespace d_point_lio;
 * auto laser_mapping2 = std::make_shared<LaserMapping>();
 *
 * // Create synchronized data group
 * MeasureGroup meas_group;
 * meas_group.lidar_beg_time = ros::Time::now().toSec();
 * @endcode
 *
 * Best practices:
 * 1. Always validate data before processing
 * 2. Use CommonConstants for consistent parameter values
 * 3. Prefer type aliases (Vec3, Mat33) over full Eigen types
 * 4. Check timestamp validity with DataUtils::isValidTimestamp()
 * 5. Use structured data types (IMUData, PoseWithCovariance) for internal processing
 * 6. Use full namespace qualification (d_point_lio::) in header files
 * 7. Use 'using namespace d_point_lio' in source files for convenience
 */

#endif // POINT_LIO_COMMON_DATA_H