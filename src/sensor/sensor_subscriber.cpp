/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "rliv_gs/sensor/sensor_subscriber.hpp"

namespace sensor {

SensorSubscriber::SensorSubscriber(ros::NodeHandle& nh, const common::Config& config) : config_(config) {

    // Lidar subscriber
    const common::LidarParameter& lidar_params = config_.getLidarParam();
    if(lidar_params.enable)
    {
        if(lidar_params.type == static_cast<int>(common::LidarParameter::LidarType::kAvia)) {
            lidar_sub_ = nh.subscribe(lidar_params.topic, 200000, &SensorSubscriber::aviaCallback, this);
        } else {
            lidar_sub_ = nh.subscribe(lidar_params.topic, 200000, &SensorSubscriber::standardCallback, this);
        }
    } else {
        common::Utils::printColored(" Lidar is not enabled! ", common::Color::kYellow, common::Style::kBold);
    }

    // Camera subscriber
    const common::CameraParameter& camera_params = config_.getCameraParam();
    if(camera_params.enable) {
        camera_sub_ = nh.subscribe(camera_params.topic, 200000, &SensorSubscriber::cameraCallback, this);
    } else {
        common::Utils::printColored(" Camera is not enabled! ", common::Color::kYellow, common::Style::kBold);
    }

    // IMU subscriber
    const common::ImuParameter& imu_params = config_.getImuParam();
    if(imu_params.enable) {
        imu_sub_ = nh.subscribe(imu_params.topic, 200000, &SensorSubscriber::imuCallback, this);
    } else {
        common::Utils::printColored(" IMU is not enabled! ", common::Color::kYellow, common::Style::kBold);
    }

    // GNSS subscriber
    const common::GnssParameter& gnss_params = config_.getGnssParam();
    if(gnss_params.enable) {
        gnss_sub_ = nh.subscribe(gnss_params.topic, 200000, &SensorSubscriber::gnssCallback, this);
    } else {
        common::Utils::printColored(" GNSS is not enabled! ", common::Color::kYellow, common::Style::kBold);
    }
    common::Utils::printColored(" SensorSubscriber initialized successfully! ", common::Color::kGreen, common::Style::kBold);
}

// Handle AVIA lidar data to pcl
void SensorSubscriber::aviaCallback(const livox_ros_driver::CustomMsg::ConstPtr& msg_in) {
    // Chack if the lidar timestamp loop back
    double current_lidar_timestamp = msg_in->header.stamp.toSec();
    common::Utils::printColored(" Get lidar header timestamp: " + std::to_string(current_lidar_timestamp), common::Color::kGreen, common::Style::kBold);
    if (current_lidar_timestamp < latest_lidar_timestamp_) {
        common::Utils::printColored(" Lidar timestamp loop back! ", common::Color::kRed, common::Style::kBold);
        lidar_data_buffer_.clear(); 
        lidar_timestamps_buffer_.clear();
        return;
    }

    // Check the point number
    uint point_num = msg_in->point_num;
    if (point_num == 0) {
        common::Utils::printColored(" No points in the lidar data! ", common::Color::kRed, common::Style::kBold);
        return;
    } else {
        common::Utils::printColored(" Input point number: " + std::to_string(point_num), common::Color::kGreen, common::Style::kBold);
    }

    // Transform the CustomMsg to pcl::PointCloud<PointXYZIRT>
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_ptr(new pcl::PointCloud<PointXYZIRT>());
    cloud_ptr->reserve(point_num); // Reserve memory for the point cloud

    // Fill the cloud_ptr
    const common::LidarParameter& lidar_params = config_.getLidarParam();
    for (uint i = 0; i < point_num; i++) {

        float distance = std::sqrt(msg_in->points[i].x * msg_in->points[i].x +
                        msg_in->points[i].y * msg_in->points[i].y +
                        msg_in->points[i].z * msg_in->points[i].z);

        if (msg_in->points[i].line < lidar_params.lines) {

            // Set the point data
            PointXYZIRT point;
            point.x = msg_in->points[i].x;
            point.y = msg_in->points[i].y;
            point.z = msg_in->points[i].z;
            point.intensity = msg_in->points[i].reflectivity;
            point.ring = msg_in->points[i].line;
            point.timestamp = msg_in->points[i].offset_time / float(1000000);
            // Set the first point timestamp to [0.0, 1.0) range]
            if (i == 0) {
                point.timestamp = std::fabs(point.timestamp) < 1.0 ? point.timestamp : 0.0;
            } else {
                if (std::fabs(point.timestamp - msg_in->points[i - 1].offset_time / float(1000000)) > 1.0) {
                    // ！There is a hidden danger that if some points of the lidar are lost, it will cause timestamp errors ！
                    point.timestamp =msg_in->points[i - 1].offset_time / float(1000000) + 0.004166667f; // float(100/24000)
                    common::Utils::printColored(" Warning: lidar timestamps range over 1.0ms ! ", 
                                                    common::Color::kYellow, common::Style::kBold); 
                }
            }
            if (distance >= lidar_params.blind) {
                cloud_ptr->points.push_back(point);
            }
        }
    }

    common::Utils::printColored(" Output point number: " + std::to_string(cloud_ptr->points.size()), 
                                common::Color::kGreen, common::Style::kBold);

    // Push the data to buffer
    lidar_data_buffer_.push_back(cloud_ptr);
    lidar_timestamps_buffer_.push_back(current_lidar_timestamp);
    latest_lidar_timestamp_ = current_lidar_timestamp;
    if (!isLidarCallbackCalled_)
        isLidarCallbackCalled_ = true;
}

// Handle standard lidar data
void SensorSubscriber::standardCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_in) {




}

// Handle camera data
void SensorSubscriber::cameraCallback(const sensor_msgs::Image::ConstPtr& msg_in) {

    if (latest_lidar_timestamp_ < 0.0)
        return;
    // Check if the camera timestamp loop back
    const common::LidarParameter& lidar_params = config_.getLidarParam();
    double current_camera_timestamp = msg_in->header.stamp.toSec() + lidar_params.lidar_camera_time_offset;
    common::Utils::printColored(" Get camera header timestamp: " + std::to_string(current_camera_timestamp), 
                                    common::Color::kGreen, common::Style::kBold);
    if (current_camera_timestamp < latest_camera_timestamp_) {
        common::Utils::printColored(" Camera timestamp loop back! ", common::Color::kRed, common::Style::kBold);
        camera_data_buffer_.clear();
        camera_timestamps_buffer_.clear();
        return;
    }

    // Transform the sensor_msgs::Image to cv::Mat
    cv::Mat camera_image;
    camera_image = cv_bridge::toCvCopy(msg_in, "bgr8")->image;

    // Push the data to buffer
    camera_data_buffer_.push_back(camera_image);
    camera_timestamps_buffer_.push_back(current_camera_timestamp);
    latest_camera_timestamp_ = current_camera_timestamp;

}

// Handle IMU data
void SensorSubscriber::imuCallback(const sensor_msgs::Imu::ConstPtr& msg_in) {

    if (latest_lidar_timestamp_ < 0.0)
        return;
    // Chack if the imu timestamp loop back
    double current_imu_timestamp = msg_in->header.stamp.toSec();
    if (current_imu_timestamp < latest_imu_timestamp_) {
        common::Utils::printColored(" IMU timestamp loop back! ", common::Color::kRed, common::Style::kBold);
        imu_data_buffer_.clear();
        return;
    }

    // Push the data to buffer
    imu_data_buffer_.push_back(msg_in);
    latest_imu_timestamp_ = current_imu_timestamp;

}

// Handle GNSS data 
void SensorSubscriber::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg_in) {

}

const common::Config& SensorSubscriber::getConfig() const {
    return config_;
}

std::deque<pcl::PointCloud<PointXYZIRT>::Ptr>& SensorSubscriber::getLidarDataBuffer() {
    return lidar_data_buffer_;
}

std::deque<cv::Mat>& SensorSubscriber::getCameraDataBuffer() {
    return camera_data_buffer_;
}

std::deque<sensor_msgs::Imu::ConstPtr>& SensorSubscriber::getImuDataBuffer() {
    return imu_data_buffer_;
}

std::deque<sensor_msgs::NavSatFix::ConstPtr>& SensorSubscriber::getGnssDataBuffer() {
    return gnss_data_buffer_;
}

std::deque<double>& SensorSubscriber::getLidarTimestampsBuffer() {
    return lidar_timestamps_buffer_;
}

std::deque<double>& SensorSubscriber::getCameraTimestampsBuffer() {
    return camera_timestamps_buffer_;
}

bool SensorSubscriber::isLoActive() const {
    if (!lidar_data_buffer_.empty()) {
        return true;
    } else {
        return false;
    }
}

bool SensorSubscriber::isLioActive() const {
    if (!lidar_data_buffer_.empty() && !imu_data_buffer_.empty()) {
        return true;
    } else {
        return false;
    }
}

bool SensorSubscriber::isLivoActive() const {
    if (!lidar_data_buffer_.empty() && !imu_data_buffer_.empty() && !camera_data_buffer_.empty()) {
        return true;
    } else {
        return false;
    }
}

bool SensorSubscriber::isGlivoActive() const {
    if (!lidar_data_buffer_.empty() && !imu_data_buffer_.empty() && !camera_data_buffer_.empty() && !gnss_data_buffer_.empty()) {
        return true;
    } else {
        return false;
    }
}



SensorSubscriber::~SensorSubscriber() {
    // Destructor
}

} // namespace sensor