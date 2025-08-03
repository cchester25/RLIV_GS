/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef PARAMETER_HPP
#define PARAMETER_HPP

#include <string>
#include <vector>
namespace common {

struct LidarParameter {
    // Enable lidar 
    bool enable;
    // Lidar type
    enum class LidarType {
        kAvia = 0,
        kOuster = 1,
        kVelodyne = 2,
    };
    int type; // For compatibility with rosparam, use int to store enum
    // Lidar lines 
    int lines;
    // Lidar topic
    std::string topic;
    // Lidar frequency
    double frequency; // in Hz
    // Lidar blind
    double blind;
    // Lidar intensity threshold
    double min_intensity;
    double max_intensity;
    // Lidar noise
    double ranging_noise; // in meters
    double bearing_noise; // in radians
    // Lidar2imu extrinsic
    std::vector<double> lidar_imu_rotation;
    std::vector<double> lidar_imu_translation; // in meters
    // Lidar2imu time offset
    double lidar_imu_time_offset; // in seconds
    // Lidar2camera extrinsic
    std::vector<double> lidar_camera_rotation;
    std::vector<double> lidar_camera_translation; // in meters
    // Lidar2camera time offset
    double lidar_camera_time_offset; // in seconds
};

struct CameraParameter {
    // Camera enable
    bool enable;
    // Camera topic
    std::string topic;
    // Camera frequency
    double frequency; // in Hz
    // Camera exposure time
    double exposure_time; // in seconds
    // Camera inverted exposure time noise
    double inv_exposure_time_cov; // in Hz
    // Camera photometric noise
    double photometric_noise; // The covariance of photometric errors per pixel
};

struct ImuParameter {
    // Enable imu
    bool enable;
    // Imu type
    enum class ImuType {
        kAxis6 = 0,
        kAxis9 = 1,
    };
    int type; // For compatibility with rosparam, use int to store enum
    // Imu topic
    std::string topic;
    // Imu frequency
    double frequency; // in Hz
    // Imu noise covariance
    double acc_cov;
    double gyro_cov;
    double acc_bias_cov;
    double gyro_bias_cov;
    double gravity_constant;
};

struct GnssParameter {
    // Enable gnss
    bool enable;
    // Gnss type 0: RTK-Single Antenna, 1: RTK-Dual Antenna
    enum class GnssType {
        kSingleAntenna = 0,
        kDualAntenna = 1,
    };
    int type; // For compatibility with rosparam, use int to store enum
    // Gnss topic
    std::string topic;
    // Gnss frequency
    double frequency; // in Hz
    // Gnss noise covariance
    double position_cov; // in meters
    double velocity_cov; // in meters per second
    double attitude_cov; // in radians
    std::vector<double> gnss_imu_translation; // Translation from GNSS to IMU in meters
    std::vector<double> gnss_imu_rotation; // Rotation from GNSS to IMU as a 3x3 matrix
    double gnss_imu_time_offset; // Time offset between GNSS and IMU in seconds
};

struct FrontendParameter {
    // Maximum number of iterations
    int max_iterations;

    // Frontend mode
    enum class FrontendMode {
        kLo = 0, // Lidar-Odometry
        kLio = 1, // Lidar-Inertial-Odometry
        kLivo = 2, // Lidar-Inertial-Vision-Odometry
        kGlivo = 3, // GNSS-Lidar-Inertial-Vision-Odometry
    };
    FrontendMode mode; // For compatibility with rosparam, use int to store enum

    struct  LioParameter {
        double frame_voxel_size; // Voxel size in meters for one frame
        double map_voxel_size; // Voxel map size in meters
        double plane_threshold; // Plane threshold in meters
        double sigma_num; // Sigma number for outlier rejection
        int octree_max_depth; // Maximum depth of the octree
        int node_max_points; // Maximum number of points in a node
        int node_min_points; // Minimum number of points in a node
        int capacity; // Capacity of the octree
    };
    LioParameter lio_param;

    struct VioParameter {
        bool normal_enable; // Enable normal estimation
        bool raycast_enable; // Enable raycast
        int grid_size; // Grid size for raycast
        int patch_size; // Image patch size in pixels
        int pyrimid_level; // Number of pyramid levels
        double outlier_threshold; //Total photometric error threshold for outlier rejection
    };
    VioParameter vio_param;

    struct RelocationParameter {
        bool enable; // Enable relocation
    };
    RelocationParameter relocalization;

    struct PublishParameter {
        bool dense_map_enable; // Enable dense map publishing
        bool effect_point_enable; // Enable effect point publishing
        bool plane_enable; // Enable plane publishing
        int scan_num; // Number of scans to publish
    };
    PublishParameter publish_param;

    struct OutputParameter {
      bool colmap_save; // Enable COLMAP saving
      bool pcd_save; // Enable PCD saving
      double pcd_filter_size; // PCD filter size in meters
      int save_num; // Number of frames to save
    };
    OutputParameter output_param;
};

} // namespace common
#endif // PARAMETER_HPP