/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "rliv_gs/sensor/sensor_evaluator.hpp"

namespace sensor {


SensorEvaluator::SensorEvaluator(SensorSubscriber& sensor_subscriber) : sensor_subscriber_(sensor_subscriber), 
                                                                        config_(sensor_subscriber_.getConfig()), 
                                                                        lidar_data_buffer_(sensor_subscriber_.getLidarDataBuffer()), 
                                                                        image_data_buffer_(sensor_subscriber_.getCameraDataBuffer()),
                                                                        imu_data_buffer_(sensor_subscriber_.getImuDataBuffer()),
                                                                        gnss_data_buffer_(sensor_subscriber_.getGnssDataBuffer()){
    // TODO: Initialize the sensor evaluator
    lidar_enabled_ = config_.getLidarParam().enable;
    camera_enabled_ = config_.getCameraParam().enable;
    imu_enabled_ = config_.getImuParam().enable;
    gnss_enabled_ = config_.getGnssParam().enable;

    common::Utils::printColored(" SensorEvaluator initialized successfully! ", common::Color::kGreen, common::Style::kBold);
}
SensorEvaluator::~SensorEvaluator() {
    
}

bool SensorEvaluator::gatherSynchronizedData() {

    // Based on the frontend mode, gather synchronized data
    switch (config_.getFrontendParam().mode) {
        case common::FrontendParameter::FrontendMode::kLo: {
            // Lidar-Odometry mode
            break;
        }
        case common::FrontendParameter::FrontendMode::kLio: {
            break;
        }
        case common::FrontendParameter::FrontendMode::kLivo: {
            switch (measurement_group_.frontend_stage) {
                case measurement_group_.FrontendStage::kInitial: {
                    common::Utils::printColored(" Initializing frontend ... ", common::Color::kGreen, common::Style::kBold);
                }
                case measurement_group_.FrontendStage::kLiFusion: {

                    // Set the latest LiDAR inertial start time
                    if (measurement_group_.latest_lio_end_time == 0.0) {
                        measurement_group_.latest_lio_end_time = sensor_subscriber_.getLidarTimestampsBuffer().front();
                    }

                    // Uses the image capture timestamp as the starting time of LiDAR-inertial fusion
                    double image_capture_time = sensor_subscriber_.getCameraTimestampsBuffer().front() + config_.getCameraParam().exposure_time;
                    double newest_lidarpoint_time = 
                            sensor_subscriber_.getLidarTimestampsBuffer().back() + 
                            sensor_subscriber_.getLidarDataBuffer().back()->points.back().timestamp / double(1000);
                    double newest_imu_time = sensor_subscriber_.getImuDataBuffer().back()->header.stamp.toSec();
                    double oldest_imu_time = sensor_subscriber_.getImuDataBuffer().front()->header.stamp.toSec();

                    // Check if the image capture time is within the range of LiDAR and IMU data
                    if (image_capture_time > newest_lidarpoint_time || image_capture_time > newest_imu_time) {
                        return false;
                    }
                    if (image_capture_time < measurement_group_.latest_lio_end_time + 0.00001) {
                        common::Utils::printColored(" Image capture time is before the oldest_imu_time and the last LiDAR-inertial fusion start time! ", common::Color::kRed, common::Style::kBold);
                        sensor_subscriber_.getCameraDataBuffer().pop_front();
                        sensor_subscriber_.getCameraTimestampsBuffer().pop_front();
                        return false;
                    }

                    common::Utils::printColored(" [ Data Cut ] Newest IMU time is " + std::to_string(newest_imu_time), 
                                                    common::Color::kYellow, common::Style::kBold);
                    common::Utils::printColored(" [ Data Cut ] Oldest IMU time is " + std::to_string(oldest_imu_time), 
                                                    common::Color::kYellow, common::Style::kBold);
                    common::Utils::printColored(" [ Data Cut ] Image capture time is " + std::to_string(image_capture_time), 
                                                    common::Color::kYellow, common::Style::kBold);
                    common::Utils::printColored(" [ Data Cut ] Newest LiDAR point time is " + std::to_string(newest_lidarpoint_time), 
                                                    common::Color::kYellow, common::Style::kBold);

                    measurement_group_.current_lio_end_time = image_capture_time;
                    ImuGroup imu_measure;
                    imu_measure.lio_time = image_capture_time;
                    // Get the imu data within the latest LiDAR inertial start time and the image capture time
                    while (!sensor_subscriber_.getImuDataBuffer().empty()) {
                        if (sensor_subscriber_.getImuDataBuffer().front()->header.stamp.toSec() <= measurement_group_.current_lio_end_time) {
                            if (sensor_subscriber_.getImuDataBuffer().front()->header.stamp.toSec() > measurement_group_.latest_lio_end_time) {
                                imu_measure.imu.push_back(sensor_subscriber_.getImuDataBuffer().front());
                                sensor_subscriber_.getImuDataBuffer().pop_front();
                            } else {
                                common::Utils::printColored(" imu time < latest_lio_end_time ", common::Color::kYellow, common::Style::kBold);
                                sensor_subscriber_.getImuDataBuffer().pop_front();
                            }
                        } else {
                            break; // Stop when the IMU data is after the image capture time
                        }
                    }
                    // Get the LiDAR data within the latest LiDAR inertial start time and the image capture time
                    *(measurement_group_.lidar_data_current) = *(measurement_group_.lidar_data_next);

                    // for (auto& point : measurement_group_.lidar_data_next->points) {
                    //     common::Utils::fout_dt << point.x << " " << point.y << " " << point.z << " " << point.timestamp << std::endl;
                    // }


                    pcl::PointCloud<PointXYZIRT>().swap(*measurement_group_.lidar_data_next);
                    int lidar_frame_num = sensor_subscriber_.getLidarDataBuffer().size();
                    int max_size = measurement_group_.lidar_data_current->size() + 24000 * lidar_frame_num;
                    measurement_group_.lidar_data_current->reserve(max_size);
                    measurement_group_.lidar_data_next->reserve(max_size);
                    while (!sensor_subscriber_.getLidarDataBuffer().empty()) {
                        if (sensor_subscriber_.getLidarTimestampsBuffer().front() <= image_capture_time) {
                            // Set the maximum timesoffset of measurement group
                            auto pcl(sensor_subscriber_.getLidarDataBuffer().front()->points);
                            double frame_header_time = sensor_subscriber_.getLidarTimestampsBuffer().front();
                            float max_time_offset = (imu_measure.lio_time - frame_header_time) * 1000.0f;
                            int num_points = pcl.size();
                            for (int i = 0; i < num_points; i++) {
                                auto point = pcl[i];
                                if (point.timestamp < max_time_offset) {
                                    point.timestamp += (frame_header_time - measurement_group_.latest_lio_end_time) * 1000.0f;
                                    measurement_group_.lidar_data_current->points.push_back(point);
                                } else { 
                                    point.timestamp += (frame_header_time - imu_measure.lio_time) * 1000.0f;
                                    measurement_group_.lidar_data_next->points.push_back(point);
                                }
                            }
                            sensor_subscriber_.getLidarDataBuffer().pop_front();
                            sensor_subscriber_.getLidarTimestampsBuffer().pop_front();
                        } else {
                            break;
                        }
                    }

                    measurement_group_.imu_measures.push_back(imu_measure);
                    measurement_group_.frontend_stage = measurement_group_.FrontendStage::kLiFusion;
                    return true; // Successfully gathered synchronized data
                }
                case measurement_group_.FrontendStage::kViFusion: {
                    // Uses the image capture timestamp as the starting time of Vision-inertial fusion
                    double image_capture_time = sensor_subscriber_.getCameraTimestampsBuffer().front() + config_.getCameraParam().exposure_time;
                    // Get the imgage data
                    measurement_group_.imu_measures.clear();
                    struct ImuGroup m;
                    m.vio_time = image_capture_time;
                    m.lio_time = measurement_group_.latest_lio_end_time;

                    m.img = sensor_subscriber_.getCameraDataBuffer().front();
                    sensor_subscriber_.getCameraDataBuffer().pop_front();
                    sensor_subscriber_.getCameraTimestampsBuffer().pop_front();
                    measurement_group_.imu_measures.push_back(m);   

                    return true; // Successfully gathered synchronized data
                }
            }
            break;
        }
        case common::FrontendParameter::FrontendMode::kGlivo: {
            break;
        }
    }

    // Return true if successful, false otherwise
    return true;

}

MeasurementGroup& SensorEvaluator::getMeasurementGroup() {
    // Return the measurement group
    return measurement_group_;
}

}