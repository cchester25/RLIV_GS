/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "rliv_gs/rlivgs_system.hpp"

namespace rlivgs {

RlivgsSystem::RlivgsSystem(ros::NodeHandle& nh) : config_(nh), 
                                                  sensor_subscriber_(sensor::SensorSubscriber(nh, config_)), 
                                                  sensor_evaluator_(sensor::SensorEvaluator(sensor_subscriber_)), 
                                                  sensor_publisher_(sensor::SensorPublisher(nh)) {
    // New objects are initialized here
    
    common::Utils::printColored(" RlivgsSystem initialized successfully ! ", common::Color::kGreen, common::Style::kBold);
    esikf_.getMapManager().loadVoxelMapConfig(config_);
    all_points_.reset(new pcl::PointCloud<PointXYZIRT>());
    points_lio_.reset(new pcl::PointCloud<PointXYZIRT>());
    // common::Utils::printColored(" Frontend mode: " + std::to_string(mode), common::Color::kGreen, common::Style::kBold);
}

void RlivgsSystem::run() {
    ros::Rate loop_rate(5000);
    while (ros::ok()) {
        ros::spinOnce();
        if (isCallbackActive()) {
            if (!sensor_evaluator_.gatherSynchronizedData()) {
                loop_rate.sleep();
                continue; // Continue to the next iteration if data gathering fails
            } else {
                if (first_frame_) {
                    common::Utils::first_lidar_header_time = sensor_evaluator_.getMeasurementGroup().latest_lio_end_time;
                    first_frame_ = false;
                }
            }

            if (sensor_evaluator_.getMeasurementGroup().frontend_stage == sensor::MeasurementGroup::FrontendStage::kLiFusion) {
                handleLio();
                sensor_publisher_.publishGlobalLidar(esikf_.getMapManager().getDownsampledGlobalCloud());
                sensor_evaluator_.getMeasurementGroup().frontend_stage = sensor::MeasurementGroup::FrontendStage::kViFusion;
            } else if (sensor_evaluator_.getMeasurementGroup().frontend_stage == sensor::MeasurementGroup::FrontendStage::kViFusion) {
                if (!is_imu_init_) {
                    double vio_time = sensor_evaluator_.getMeasurementGroup().imu_measures.front().vio_time;
                    sensor_evaluator_.getMeasurementGroup().latest_lio_end_time = vio_time;
                    esikf_.setPropagateState(esikf_.getCurrentState());
                }
                if (!first_lio_) {
                    handleVio();
                    sensor_publisher_.publishImage(img_plot_);
                    sensor_publisher_.publishGlobalPclRGB(esikf_.getMapManager().getDownsampledGlobalCloud(),
                                                        esikf_.getMapManager().getVisualMapPtr(), img_rgb_);
                }
                sensor_evaluator_.getMeasurementGroup().frontend_stage = sensor::MeasurementGroup::FrontendStage::kLiFusion;
            } else {
                common::Utils::printColored(" Unknown frontend stage! ", common::Color::kRed, common::Style::kBold);
            }
        }
    }

    if (config_.getFrontendParam().output_param.pcd_save) {
        std::string downsampled_points_dir = std::string(ROOT_DIR) + "log/PCD/all_downsampled_points.pcd";
        savePCD(downsampled_points_dir, all_points_);
    }

}

void RlivgsSystem::handleLio() {

    // If IMU is enabled, initialize the IMU state
    if (is_imu_init_) {
        if (esikf_.isImuInitialized(config_,sensor_evaluator_.getMeasurementGroup())) {
            is_imu_init_ = false;
            return; // Continue to the next iteration if IMU is initialized
        } else {
            return; // Continue to the next iteration if IMU is initialized
        }
    }

    if (esikf_.pridictState(sensor_evaluator_.getMeasurementGroup())) {

        Eigen::Vector3d euler_cur = common::Utils::RotMtoEuler(esikf_.getCurrentState().getSystemRotation());
        common::Utils::fout_pridicted_state << std::setw(20)
                    << sensor_evaluator_.getMeasurementGroup().latest_lio_end_time - common::Utils::first_lidar_header_time << " "
                    << euler_cur.transpose() * 57.3 << " "
                    << esikf_.getCurrentState().getSystemPosition().transpose() << " " << esikf_.getCurrentState().getSystemVelocity().transpose()
                    << " " << esikf_.getCurrentState().getImuGyroBias().transpose() << " "
                    << esikf_.getCurrentState().getImuAccBias().transpose() << " "
                    << Eigen::Vector3d(esikf_.getCurrentState().getInverseExposureTime(), 0, 0).transpose() << std::endl;
        
        pcl::PointCloud<PointXYZIRT>::Ptr lidar_undistorted = sensor_evaluator_.getMeasurementGroup().lidar_undistorted;
        if (lidar_undistorted->empty()) {
            common::Utils::printColored(" Lidar undistorted is empty! ", common::Color::kRed, common::Style::kBold);
            return;
        } else {
            common::Utils::printColored(" [ LIO ] Raw point num: " + std::to_string(lidar_undistorted->size()), common::Color::kBlue, common::Style::kBold);
        }

        // Voxel filter
        map::MapManager& map_manager = esikf_.getMapManager();
        map_manager.voxelGridFilter(lidar_undistorted);

        esikf_.transformLidarToGlobal(map_manager.getDownsampledLidarCloud(), map_manager.getDownsampledGlobalCloud());

        if (first_lio_) {
            first_lio_ = false;
            common::Utils::printColored(" Start building voxel map! ", common::Color::kYellow, common::Style::kBold);
            map_manager.buildMap(esikf_.getCurrentState());
            common::Utils::printColored(" Voxel map built! ", common::Color::kYellow, common::Style::kBold);

        }
        test++;

        if (test == 3) {
            std::cout << "[ LIO ]: Point cloud size: "
                    << lidar_undistorted->points.size() << std::endl;
            // for (auto &point : feats_undistort_->points) {
            //   fout_dt_ << point.x << " " << point.y << " " << point.z << " " << point.curvature << std::endl;
            // }
            // std::cin.get(); // Wait for user input to continue
        }
    
        esikf_.updateStateWithLidar(); // Update the state with lidar data

        esikf_.transformLidarToGlobal(map_manager.getDownsampledLidarCloud(), map_manager.getDownsampledGlobalCloud());

        map_manager.updateMap(esikf_.getCurrentState());

        euler_cur = common::Utils::RotMtoEuler(esikf_.getCurrentState().getSystemRotation());
        common::Utils::fout_updated_state << std::setw(20)
                    << sensor_evaluator_.getMeasurementGroup().latest_lio_end_time - common::Utils::first_lidar_header_time << " "
                    << euler_cur.transpose() * 57.3 << " "
                    << esikf_.getCurrentState().getSystemPosition().transpose() << " " << esikf_.getCurrentState().getSystemVelocity().transpose()
                    << " " << esikf_.getCurrentState().getImuGyroBias().transpose() << " "
                    << esikf_.getCurrentState().getImuAccBias().transpose() << " "
                    << Eigen::Vector3d(esikf_.getCurrentState().getInverseExposureTime(), 0, 0).transpose() << std::endl;

        
    }
}

void RlivgsSystem::handleVio() {
    
    map::MapManager& map_manager = esikf_.getMapManager();
    int points_num = map_manager.getDownsampledGlobalCloud()->size();
    common::Utils::printColored(" [ VIO ] Raw point num: " + std::to_string(points_num), common::Color::kBlue, common::Style::kBold);

    // Get image in MeasurementGroup
    cv::Mat& image = sensor_evaluator_.getMeasurementGroup().imu_measures.back().img;
    img_plot_ = image.clone();
    img_rgb_ = image.clone();

    if (image.channels() == 3) cv::cvtColor(image, image, CV_BGR2GRAY);

    // Reset new frame 
    map_manager.getVisualMapPtr()->resetNewFrame(image);

    map_manager.getVisualMapPtr()->updateFrameState(esikf_.getCurrentState());

    map_manager.getVisualMapPtr()->resetGrid();

    map_manager.getVisualMapPtr()->retrieveFromVisualSparseMap(image, 
                                                                map_manager.getPointsWithCovariancesList(), 
                                                                map_manager.getVoxelMap(),
                                                                esikf_.getCurrentState().getInverseExposureTime());

    int pyrimid_level = config_.getFrontendParam().vio_param.pyrimid_level;
    if (map_manager.getVisualMapPtr()->getTotalPoints() > 0) {
        for (int level = pyrimid_level - 1; level >= 0; level--) {
            esikf_.updateStateWithVision(image, level);
        }
        map_manager.getVisualMapPtr()->updateFrameState(esikf_.getCurrentState());
    }

    map_manager.getVisualMapPtr()->generateVisualMapPoints(image, 
                                                            map_manager.getPointsWithCovariancesList(),
                                                            esikf_.getCurrentState().getInverseExposureTime());
    

    map_manager.getVisualMapPtr()->plotTrackedPoints(img_plot_);

    map_manager.getVisualMapPtr()->updateVisualMap(image, esikf_.getCurrentState().getInverseExposureTime());
    map_manager.getVisualMapPtr()->updateReferencePatch(map_manager.getVoxelMap());
    // Print the state of VIO update
    Eigen::Vector3d euler_cur = common::Utils::RotMtoEuler(esikf_.getCurrentState().getSystemRotation());
    common::Utils::fout_updated_state << std::setw(20)
                << sensor_evaluator_.getMeasurementGroup().latest_lio_end_time - common::Utils::first_lidar_header_time << " "
                << euler_cur.transpose() * 57.3 << " "
                << esikf_.getCurrentState().getSystemPosition().transpose() << " " << esikf_.getCurrentState().getSystemVelocity().transpose()
                << " " << esikf_.getCurrentState().getImuGyroBias().transpose() << " "
                << esikf_.getCurrentState().getImuAccBias().transpose() << " "
                << Eigen::Vector3d(esikf_.getCurrentState().getInverseExposureTime(), 0, 0).transpose() << std::endl;


}



bool RlivgsSystem::isCallbackActive() const {

    if (config_.getFrontendParam().mode == common::FrontendParameter::FrontendMode::kLo) {
        return sensor_subscriber_.isLoActive();
    } else if (config_.getFrontendParam().mode == common::FrontendParameter::FrontendMode::kLio) {
        return sensor_subscriber_.isLioActive();
    } else if (config_.getFrontendParam().mode == common::FrontendParameter::FrontendMode::kLivo) {
        return sensor_subscriber_.isLivoActive();
    } else if (config_.getFrontendParam().mode == common::FrontendParameter::FrontendMode::kGlivo) {
        return sensor_subscriber_.isGlivoActive();
    } else {
        return false;
    }

}



void RlivgsSystem::savePCD(const std::string& filename, const pcl::PointCloud<PointXYZIRT>::Ptr cloud) {
    common::Utils::printColored(" Saving PCD file: " + filename, common::Color::kYellow, common::Style::kBold);
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(filename, *cloud);
}

RlivgsSystem::~RlivgsSystem() {
    // Destructor
    // Cleanup if necessary

}

} // namespace rlivgs