/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef SENSOR_MEASUREMENT_HPP
#define SENSOR_MEASUREMENT_HPP

#include <deque>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/opencv.hpp>




struct PointXYZIRT {
    PCL_ADD_POINT4D;                    // XYZ coordinates (with padding)
    float intensity;                    // Laser reflection intensity
    std::uint16_t ring;                      // LiDAR scan line index (0-63 for Velodyne 64-line)
    float timestamp;                   // Timestamp (seconds since epoch)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // Ensure memory alignment for Eigen operations
} EIGEN_ALIGN16;                        // Force 16-byte alignment



// Register the point structure with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
    (float, x, x)                      // X coordinate
    (float, y, y)                      // Y coordinate
    (float, z, z)                      // Z coordinate
    (float, intensity, intensity)      // Intensity value
    (std::uint16_t, ring, ring)             // LiDAR ring index
    (float, timestamp, timestamp)     // Timestamp (seconds)
)

namespace sensor {

struct ImuGroup {
  double vio_time;
  double lio_time;
  std::deque<sensor_msgs::Imu::ConstPtr> imu;
  cv::Mat img;
  ImuGroup() {
    vio_time = 0.0;
    lio_time = 0.0;
  };
};

struct MeasurementGroup {
    
    // Timestamp information
    double lidar_frame_begin_time; // Start time of the LiDAR frame (seconds since epoch)
    double lidar_frame_end_time;   // End time of the LiDAR frame (seconds since
    double latest_lio_end_time; // Start time of the last LiDAR-Inertial-Odometry
    double current_lio_end_time; // Start time of the current LiDAR-Inertial-Odometry
    double current_vio_start_time; // Start time of the current Vision-Inertial-Odometry

    

    // Sensor data
    pcl::PointCloud<PointXYZIRT>::Ptr lidar_data_current; // Pointer to current LiDAR point cloud data
    pcl::PointCloud<PointXYZIRT>::Ptr lidar_data_next; // Pointer to next LiDAR point cloud data
    pcl::PointCloud<PointXYZIRT>::Ptr lidar_undistorted; // Pointer to undistorted LiDAR point cloud data
    std::deque<struct ImuGroup> imu_measures;
    std::deque<sensor_msgs::NavSatFix::ConstPtr> gnss_data;

    enum class FrontendStage {
        kInitial = 0, // Initial stage
        kLiFusion = 1, // Lidar-Inertial Fusion stage
        kViFusion = 2, // Vision-Inertial Fusion stage
        kGiFusion =3, // GNSS-Inertial Fusion stage
    };
    FrontendStage frontend_stage; // Current frontend stage
    // Constructor
    MeasurementGroup() {
        lidar_frame_begin_time = 0.0; // Initialize to zero
        lidar_frame_end_time = 0.0; // Initialize to zero
        latest_lio_end_time = 0.0; // Initialize to zero
        current_lio_end_time = 0.0; // Initialize to zero

        lidar_data_current.reset(new pcl::PointCloud<PointXYZIRT>());
        lidar_data_next.reset(new pcl::PointCloud<PointXYZIRT>());
        lidar_undistorted.reset(new pcl::PointCloud<PointXYZIRT>());
        imu_measures.clear();
        frontend_stage = FrontendStage::kInitial; // Initialize to initial stage

    }
};
} // namespace sensor
#endif // SENSOR_MEASUREMENT_HPP