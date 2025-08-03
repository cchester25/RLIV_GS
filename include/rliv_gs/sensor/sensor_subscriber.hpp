/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef SENSOR_SUBSCRIBER_HPP
#define SENSOR_SUBSCRIBER_HPP


#include <cv_bridge/cv_bridge.h>

#include <livox_ros_driver/CustomMsg.h>
#include "rliv_gs/common/config.hpp"
#include "sensor_measurement.hpp"

namespace sensor {

class SensorSubscriber {

public:
    // Initialize subscribers here if needed
    SensorSubscriber(ros::NodeHandle& nh, const common::Config& config);

    // Destructor
    ~SensorSubscriber();

    // Get Sensor config
    const common::Config& getConfig() const;

    // Get data buffers
    std::deque<pcl::PointCloud<PointXYZIRT>::Ptr>& getLidarDataBuffer();
    std::deque<cv::Mat>& getCameraDataBuffer();
    std::deque<sensor_msgs::Imu::ConstPtr>& getImuDataBuffer();
    std::deque<sensor_msgs::NavSatFix::ConstPtr>& getGnssDataBuffer();
    // Get timestamps buffers
    std::deque<double>& getLidarTimestampsBuffer();
    std::deque<double>& getCameraTimestampsBuffer();

    // Check if the callback functions are called
    bool isLoActive() const;
    bool isLioActive() const;
    bool isLivoActive() const;
    bool isGlivoActive() const;

private:
    // Config object to access parameters
    const common::Config& config_;

    // Subscribers for different sensors
    ros::Subscriber lidar_sub_;
    ros::Subscriber camera_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber gnss_sub_;

    // Check if the callback functions are called
    bool isLidarCallbackCalled_ = false;
    bool isCameraCallbackCalled_ = false;
    bool isImuCallbackCalled_ = false;
    bool isGnssCallbackCalled_ = false;

    // Callback functions for each sensor
    void aviaCallback(const livox_ros_driver::CustomMsg::ConstPtr& msg_in);
    void standardCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_in);
    void cameraCallback(const sensor_msgs::Image::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

    // The timestamp of the latest sensor data
    double latest_lidar_timestamp_ = -1.0;
    double latest_camera_timestamp_ = -1.0;
    double latest_imu_timestamp_ = -1.0;
    double latest_gnss_timestamp_ = -1.0;

    // Sensor data storage deques
    std::deque<pcl::PointCloud<PointXYZIRT>::Ptr> lidar_data_buffer_;
    std::deque<cv::Mat> camera_data_buffer_;
    std::deque<sensor_msgs::Imu::ConstPtr> imu_data_buffer_;
    std::deque<sensor_msgs::NavSatFix::ConstPtr> gnss_data_buffer_;

    // Sensor timestamps storage deques
    std::deque<double> lidar_timestamps_buffer_;
    std::deque<double> camera_timestamps_buffer_;


};


}



#endif // SENSOR_SUBSCRIBER_HPP