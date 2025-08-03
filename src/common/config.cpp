/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "rliv_gs/common/config.hpp"

namespace common {

Config::Config(ros::NodeHandle& nh){
    // Load lidar parameters
    nh.param("sensors/lidar/enable", lidar_param_.enable, true);
    nh.param("sensors/lidar/type", lidar_param_.type, 0);
    nh.param("sensors/lidar/topic", lidar_param_.topic, std::string("/livox/lidar"));
    nh.param("sensors/lidar/frequency", lidar_param_.frequency, 10.0);
    nh.param("sensors/lidar/lines", lidar_param_.lines, 6);
    nh.param("sensors/lidar/blind", lidar_param_.blind, 0.8);
    nh.param("sensors/lidar/min_intensity", lidar_param_.min_intensity, 0.0);
    nh.param("sensors/lidar/max_intensity", lidar_param_.max_intensity, 255.0);
    nh.param("sensors/lidar/ranging_noise", lidar_param_.ranging_noise, 0.02);
    nh.param("sensors/lidar/bearing_noise", lidar_param_.bearing_noise, 0.05);
    nh.param("sensors/lidar/extrinsics/lidar_camera_translation", lidar_param_.lidar_camera_translation, std::vector<double>());
    nh.param("sensors/lidar/extrinsics/lidar_camera_rotation", lidar_param_.lidar_camera_rotation, std::vector<double>());
    nh.param("sensors/lidar/extrinsics/lidar_camera_time_offset", lidar_param_.lidar_camera_time_offset, 0.0);
    nh.param("sensors/lidar/extrinsics/lidar_imu_translation", lidar_param_.lidar_imu_translation, std::vector<double>());
    nh.param("sensors/lidar/extrinsics/lidar_imu_rotation", lidar_param_.lidar_imu_rotation, std::vector<double>());
    nh.param("sensors/lidar/extrinsics/lidar_imu_time_offset", lidar_param_.lidar_imu_time_offset, 0.0);
    
    // Load camera parameters
    nh.param("sensors/camera/enable", camera_param_.enable, true);
    nh.param("sensors/camera/topic", camera_param_.topic, std::string("/left_camera/image"));
    nh.param("sensors/camera/frequency", camera_param_.frequency, 10.0);
    nh.param("sensors/camera/exposure_time_init", camera_param_.exposure_time, 0.0);
    nh.param("sensors/camera/inv_exposure_time_cov", camera_param_.inv_exposure_time_cov, 0.1);
    nh.param("sensors/camera/photometric_noise", camera_param_.photometric_noise, 100.0);
    camera_base_config_ = YAML::LoadFile(std::string(ROOT_DIR) + "/config/camera_model.yaml");

    // Load IMU parameters
    nh.param("sensors/imu/enable", imu_param_.enable, true);
    nh.param("sensors/imu/type", imu_param_.type, 0);
    nh.param("sensors/imu/topic", imu_param_.topic, std::string("/livox/imu"));
    nh.param("sensors/imu/frequency", imu_param_.frequency, 200.0);
    nh.param("sensors/imu/acc_cov", imu_param_.acc_cov, 0.01);
    nh.param("sensors/imu/gyro_cov", imu_param_.gyro_cov, 0.01);
    nh.param("sensors/imu/acc_bias_cov", imu_param_.acc_bias_cov, 0.001);
    nh.param("sensors/imu/gyro_bias_cov", imu_param_.gyro_bias_cov, 0.001);
    nh.param("sensors/imu/gravity_constant", imu_param_.gravity_constant, 9.81);

    // Load GNSS parameters
    nh.param("sensors/gnss/enable", gnss_param_.enable, false);
    nh.param("sensors/gnss/type", gnss_param_.type, 0);
    nh.param("sensors/gnss/topic", gnss_param_.topic, std::string("/gnss"));
    nh.param("sensors/gnss/frequency", gnss_param_.frequency, 10.0);
    nh.param("sensors/gnss/position_cov", gnss_param_.position_cov, 0.1);
    nh.param("sensors/gnss/velocity_cov", gnss_param_.velocity_cov, 0.1);
    nh.param("sensors/gnss/attitude_cov", gnss_param_.attitude_cov, 0.01);
    nh.param("sensors/gnss/extrinsics/gnss_imu_translation", gnss_param_.gnss_imu_translation, std::vector<double>());
    nh.param("sensors/gnss/extrinsics/gnss_imu_rotation", gnss_param_.gnss_imu_rotation, std::vector<double>());
    nh.param("sensors/gnss/extrinsics/gnss_imu_time_offset", gnss_param_.gnss_imu_time_offset, 0.0);

    // Load frontend parameters
    nh.param("frontend/max_iterations", frontend_param_.max_iterations, 5);
    nh.param("frontend/lio/frame_voxel_size", frontend_param_.lio_param.frame_voxel_size, 0.1);
    nh.param("frontend/lio/map_voxel_size", frontend_param_.lio_param.map_voxel_size, 0.5);
    nh.param("frontend/lio/plane_threshold", frontend_param_.lio_param.plane_threshold, 0.05);
    nh.param("frontend/lio/sigma_num", frontend_param_.lio_param.sigma_num, 3.0);
    nh.param("frontend/lio/octree_max_depth", frontend_param_.lio_param.octree_max_depth, 16);
    nh.param("frontend/lio/node_max_points", frontend_param_.lio_param.node_max_points, 1000);
    nh.param("frontend/lio/node_min_points", frontend_param_.lio_param.node_min_points, 10);
    nh.param("frontend/lio/capacity", frontend_param_.lio_param.capacity, 1000000);
    nh.param("frontend/vio/normal_enable", frontend_param_.vio_param.normal_enable, true);
    nh.param("frontend/vio/grid_size", frontend_param_.vio_param.grid_size, 5);
    nh.param("frontend/vio/patch_size", frontend_param_.vio_param.patch_size, 8);
    nh.param("frontend/vio/pyrimid_level", frontend_param_.vio_param.pyrimid_level, 3);
    nh.param("frontend/vio/outlier_threshold", frontend_param_.vio_param.outlier_threshold, 0.1);
    nh.param("frontend/vio/relocalization/enable", frontend_param_.relocalization.enable, true);
    nh.param("frontend/publish/dense_map_enable", frontend_param_.publish_param.dense_map_enable, true);
    nh.param("frontend/publish/effect_point_enable", frontend_param_.publish_param.effect_point_enable, true);
    nh.param("frontend/publish/plane_enable", frontend_param_.publish_param.plane_enable, true);
    nh.param("frontend/publish/scan_num", frontend_param_.publish_param.scan_num, 10);
    nh.param("frontend/output/colmap_save", frontend_param_.output_param.colmap_save, false);
    nh.param("frontend/output/pcd_save", frontend_param_.output_param.pcd_save, false);
    nh.param("frontend/output/pcd_filter_size", frontend_param_.output_param.pcd_filter_size, 0.1);
    nh.param("frontend/output/save_num", frontend_param_.output_param.save_num, 100);

    // TODO: Load other parameters similarly...

    // validate the configuration
    if(validate()) {
        Utils::printColored(" Configuration loaded successfully! ", Color::kGreen, Style::kBold);
    } else {
        Utils::printColored(" Configuration validation failed! ", Color::kRed, Style::kBold);
        ros::shutdown(); // Shutdown ROS if validation fails
    }

    if (!setFrontendMode()) {
        Utils::printColored(" Failed to set frontend mode! ", Color::kRed, Style::kBold);
        ros::shutdown(); // Shutdown ROS if setting frontend mode fails
    }
}

bool Config::validate() const {
    // Validate lidar parameters
    if (lidar_param_.enable && (lidar_param_.type < 0 || lidar_param_.type > 2)) {
        Utils::printColored(" Lidar type is not set correctly!! ", Color::kRed, Style::kBold);
        return false;
    }

    // Validate IMU parameters
    if (imu_param_.enable && (imu_param_.type < 0 || imu_param_.type > 1)) {
        Utils::printColored(" IMU type is not set correctly!! ", Color::kRed, Style::kBold);
        return false;
    }

    // Validate GNSS parameters
    if (gnss_param_.enable && (gnss_param_.type < 0 || gnss_param_.type > 1)) {
        Utils::printColored(" GNSS type is not set correctly!! ", Color::kRed, Style::kBold);
        return false;
    }

    return true; // All validations passed
}

const LidarParameter& Config::getLidarParam() const {
    return lidar_param_;
}

const CameraParameter& Config::getCameraParam() const {
    return camera_param_;
}

const YAML::Node& Config::getCameraBaseConfig() const {
    return camera_base_config_;
}

const ImuParameter& Config::getImuParam() const {
    return imu_param_;
}

const GnssParameter& Config::getGnssParam() const {
    return gnss_param_;
}

const FrontendParameter& Config::getFrontendParam() const {
    return frontend_param_;
}

bool Config::setFrontendMode() {
    // Set the frontend mode based on the parameter
    if (lidar_param_.enable && camera_param_.enable && imu_param_.enable && gnss_param_.enable) {
        frontend_param_.mode = frontend_param_.FrontendMode::kGlivo; // GNSS-Lidar-Inertial-Vision-Odometry
        Utils::printColored(" Frontend mode set to GNSS-Lidar-Inertial-Vision-Odometry ", Color::kGreen, Style::kBold);
        return true;
    }
    else if (lidar_param_.enable && camera_param_.enable && imu_param_.enable) {
        frontend_param_.mode = frontend_param_.FrontendMode::kLivo; // Lidar-Inertial-Vision-Odometry
        Utils::printColored(" Frontend mode set to Lidar-Inertial-Vision-Odometry ", Color::kGreen, Style::kBold);
        return true;
    } 
    else if (lidar_param_.enable && imu_param_.enable) {
        frontend_param_.mode = frontend_param_.FrontendMode::kLio; // Lidar-Inertial-Odometry
        Utils::printColored(" Frontend mode set to Lidar-Inertial-Odometry ", Color::kGreen, Style::kBold);
        return true;
    } 
    else if (lidar_param_.enable) {
        frontend_param_.mode = frontend_param_.FrontendMode::kLo; // Lidar-Odometry
        Utils::printColored(" Frontend mode set to Lidar-Odometry ", Color::kGreen, Style::kBold);
        return true;
    } else {
        Utils::printColored(" No valid frontend mode set! ", Color::kRed, Style::kBold);
        return false; // No valid frontend mode set
    }
}

Config::~Config() {
    // Destructor implementation if needed
}

}