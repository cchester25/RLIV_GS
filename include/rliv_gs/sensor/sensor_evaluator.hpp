/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef SENSOR_EVALUATOR_HPP
#define SENSOR_EVALUATOR_HPP

#include "sensor_subscriber.hpp"

namespace sensor {

class SensorEvaluator {

public:
    SensorEvaluator(SensorSubscriber& sensor_subscriber);
    ~SensorEvaluator();

    // TODO：Evaluate sensor data


    // Gather synchronized data to the measurement group
    bool gatherSynchronizedData();

    // Get the measurement group
    MeasurementGroup& getMeasurementGroup();


private:
    SensorSubscriber& sensor_subscriber_;
    const common::Config& config_;
    MeasurementGroup measurement_group_; // Group to hold synchronized sensor data

    std::deque<pcl::PointCloud<PointXYZIRT>::Ptr>& lidar_data_buffer_;
    std::deque<cv::Mat>& image_data_buffer_;
    std::deque<sensor_msgs::Imu::ConstPtr>& imu_data_buffer_;
    std::deque<sensor_msgs::NavSatFix::ConstPtr>& gnss_data_buffer_;

    bool lidar_enabled_;
    bool camera_enabled_;
    bool imu_enabled_;
    bool gnss_enabled_;

};

} // namespace sensor


#endif // SENSOR_EVALUATOR_HPP