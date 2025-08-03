/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "rliv_gs/frontend/esikf.hpp"

namespace frontend {

Esikf::Esikf() {
    // Constructor implementation
    
}

bool Esikf::isImuInitialized(const common::Config& config, sensor::MeasurementGroup& measurement_group) {
    // Print the percentage of IMU initialization progress
    // imu_init_data_count_ = 0.0; // Reset the counter

    sensor::ImuGroup imu_measure = measurement_group.imu_measures.back();
    if (imu_measure.imu.empty()) {
      return false;
    }
    float imu_init_progress = imu_init_data_count_ / imu_init_threshold_ * 100.0f;
    common::Utils::printColored(" IMU Initialization Progress: " + 
                                std::to_string(imu_init_progress) + "%", common::Color::kBlue, common::Style::kBold);
    // Check if the IMU is first initialization
    if (first_initialization_) {
        last_imu_.reset(new sensor_msgs::Imu());
        imu_init_data_count_ = 1.0f;
        imu_mean_acc_ << imu_measure.imu.front()->linear_acceleration.x,
                        imu_measure.imu.front()->linear_acceleration.y,
                        imu_measure.imu.front()->linear_acceleration.z;
        imu_mean_gyro_ << imu_measure.imu.front()->angular_velocity.x,
                         imu_measure.imu.front()->angular_velocity.y,
                         imu_measure.imu.front()->angular_velocity.z;
        first_initialization_ = false;
    }

    // Get mean imu data 
    Eigen::Vector3d current_acc;
    Eigen::Vector3d current_gyro;
    for (const auto& imu_data_temp : imu_measure.imu) {
        current_acc << imu_data_temp->linear_acceleration.x,
                        imu_data_temp->linear_acceleration.y,
                        imu_data_temp->linear_acceleration.z;
        current_gyro << imu_data_temp->angular_velocity.x,
                         imu_data_temp->angular_velocity.y, 
                         imu_data_temp->angular_velocity.z;
        imu_mean_acc_ += (current_acc - imu_mean_acc_) / imu_init_data_count_;
        imu_mean_gyro_ += (current_gyro - imu_mean_gyro_) / imu_init_data_count_;
        imu_init_data_count_++;
    }

    gravity_const_ = config.getImuParam().gravity_constant;
    Eigen::Vector3d gravity = -imu_mean_acc_ / imu_mean_acc_.norm() * gravity_const_;
    
    current_state_.setGravity(gravity);
    last_imu_ = imu_measure.imu.back();
    if (imu_init_data_count_ >= imu_init_threshold_) {
        common::Utils::printColored(" IMU Initialization Complete!", common::Color::kGreen, common::Style::kBold);
        // Print IMU Initials: gravity, mean acc, mean gyro, acc_covariance, gyro covariance
        double acc_covariance = config.getImuParam().acc_cov;
        acc_cov_ << acc_covariance, acc_covariance, acc_covariance;
        double gyro_covariance = config.getImuParam().gyro_cov;
        gyro_cov_ << gyro_covariance, gyro_covariance, gyro_covariance;
        common::Utils::printColored(" IMU Initials: gravity: " + std::to_string(gravity[0]) + ", " + std::to_string(gravity[1]) + ", " + std::to_string(gravity[2]) +
            ", mean acc: " + std::to_string(imu_mean_acc_[0]) + ", " + std::to_string(imu_mean_acc_[1]) + ", " + std::to_string(imu_mean_acc_[2]) +
            ", mean gyro: " + std::to_string(imu_mean_gyro_[0]) + ", " + std::to_string(imu_mean_gyro_[1]) + ", " + std::to_string(imu_mean_gyro_[2]) +
            ", acc_covariance: " + std::to_string(acc_covariance) + ", " + std::to_string(acc_covariance) + ", " + std::to_string(acc_covariance) +
            ", gyro_covariance: " + std::to_string(gyro_covariance) + ", " + std::to_string(gyro_covariance) + ", " + std::to_string(gyro_covariance), 
            common::Color::kGreen, common::Style::kBold);
        // Print IMU Initials: ba covarience, bg covarience
        double acc_bias_covariance = config.getImuParam().acc_bias_cov;
        acc_bias_cov_ << acc_bias_covariance, acc_bias_covariance, acc_bias_covariance;
        double gyro_bias_covariance = config.getImuParam().gyro_bias_cov;
        gyro_bias_cov_ << gyro_bias_covariance, gyro_bias_covariance, gyro_bias_covariance;
        inv_expo_cov_ = config.getCameraParam().inv_exposure_time_cov;
        std::vector<double> lidar_imu_rotation = config.getLidarParam().lidar_imu_rotation;
        lidar_imu_rotation_ << lidar_imu_rotation[0], lidar_imu_rotation[1], lidar_imu_rotation[2], 
                                lidar_imu_rotation[3], lidar_imu_rotation[4], lidar_imu_rotation[5],
                                lidar_imu_rotation[6], lidar_imu_rotation[7], lidar_imu_rotation[8];
        std::vector<double> lidar_imu_translation = config.getLidarParam().lidar_imu_translation;
        lidar_imu_translation_ << lidar_imu_translation[0], lidar_imu_translation[1], lidar_imu_translation[2];
        common::Utils::printColored(" IMU Initials: acc_bias_covariance: " + 
                                    std::to_string(acc_bias_covariance) + ", " + std::to_string(acc_bias_covariance) + ", " + std::to_string(acc_bias_covariance) +
                                    ", gyro_bias_covariance: " + std::to_string(gyro_bias_covariance) + ", " + std::to_string(gyro_bias_covariance) + ", " + std::to_string(gyro_bias_covariance), 
                                    common::Color::kGreen, common::Style::kBold);
        photometric_noise_ = config.getCameraParam().photometric_noise;
        return true;
    } else {
        return false;
    }
}



bool Esikf::pridictState(sensor::MeasurementGroup& measurement_group) {
    // Temporary IMU queue
    // std::deque<sensor_msgs::Imu::ConstPtr> imu_queue_temp = measurement_group.imu_data;
    sensor::ImuGroup &meas = measurement_group.imu_measures.back();
    last_prediction_end_time_ = measurement_group.imu_measures.front().vio_time;

    // Set the IMU data for the start of prediction
    auto imu_queue_temp = meas.imu;
    imu_queue_temp.push_front(last_imu_);
    // Set the start time of the prediction to the end time of the last prediction
    const double start_time = last_prediction_end_time_;
    // Set the end time of the prediction
    const double end_time = meas.lio_time;

    imu_poses_.clear();
    imu_poses_.push_back(set_pose6d(0.0, system_last_acc_, system_last_gyro_, 
                            current_state_.getSystemVelocity(), current_state_.getSystemPosition(), current_state_.getSystemRotation()));
    double tau;
    if (first_pridiction_) {
        tau = 1.0;
        first_pridiction_ = false;
    } else {
        tau = current_state_.getInverseExposureTime();
    }
    velocity_imu_ = current_state_.getSystemVelocity();
    position_imu_ = current_state_.getSystemPosition();
    rotation_imu_ = current_state_.getSystemRotation();

    acc_imu_ = system_last_acc_;
    Eigen::Vector3d mean_acc;
    Eigen::Vector3d mean_gyro(system_last_gyro_);
    double delta_time = 0;
    // Predict each IMU point in the deque
    for (uint i = 0; i < imu_queue_temp.size() - 1; i++) {
        auto current_imu = imu_queue_temp[i];
        auto next_imu = imu_queue_temp[i + 1];

        if (next_imu->header.stamp.toSec() >= start_time) {
            // Calculate the mean acceleration and gyroscope
            
            mean_acc << 0.5 * (current_imu->linear_acceleration.x + next_imu->linear_acceleration.x),
                                       0.5 * (current_imu->linear_acceleration.y + next_imu->linear_acceleration.y),
                                       0.5 * (current_imu->linear_acceleration.z + next_imu->linear_acceleration.z);
            
            mean_gyro << 0.5 * (current_imu->angular_velocity.x + next_imu->angular_velocity.x),
                                        0.5 * (current_imu->angular_velocity.y + next_imu->angular_velocity.y),
                                        0.5 * (current_imu->angular_velocity.z + next_imu->angular_velocity.z);
            
            // common::Utils::fout_imu << std::setw(10) << current_imu->header.stamp.toSec() - common::Utils::first_lidar_header_time
            //         << " " << mean_gyro.transpose() << " " << mean_acc.transpose()
            //         << std::endl;

            // Calculate the time difference between the current and next IMU points
            
            if (current_imu->header.stamp.toSec() < start_time) {
                delta_time = next_imu->header.stamp.toSec() - last_prediction_end_time_;
                undistorted_time_ = next_imu->header.stamp.toSec() - start_time;
            } else if (i != imu_queue_temp.size() -2) {
                delta_time = next_imu->header.stamp.toSec() - current_imu->header.stamp.toSec();
                undistorted_time_ = next_imu->header.stamp.toSec() - start_time;
            } else {
                delta_time = end_time - current_imu->header.stamp.toSec();
                undistorted_time_ = end_time - start_time;
            }
            // common::Utils::fout_dt << undistorted_time_ << " " << delta_time << std::endl;
            mean_gyro -= current_state_.getImuGyroBias();
            mean_acc = mean_acc * gravity_const_ / imu_mean_acc_.norm() - current_state_.getImuAccBias();

            // Run the forward propagation
            runForwardPropagation(delta_time, mean_acc, mean_gyro);
        } else {
            // If the next IMU point is before the start time, skip it
            continue;
        }
    }
    current_state_.setSystemVelocity(velocity_imu_);
    current_state_.setSystemPosition(position_imu_);
    current_state_.setSystemRotation(rotation_imu_);

    current_state_.setInverseExposureTime(tau);
    measurement_group.latest_lio_end_time = end_time; // Update the end time of the latest IMU measurement group
    last_imu_ = imu_queue_temp.back(); // Update the last IMU point

    runBackwardPropagation(measurement_group);
    propagate_state_ = current_state_;
    common::Utils::printColored(" Esikf Prediction Done !", common::Color::kBlue, common::Style::kBold);
    return true; // Return true if prediction is successful
}

void Esikf::runForwardPropagation(double dt, Eigen::Vector3d mean_acc, Eigen::Vector3d mean_gyro) {

    Eigen::Matrix3d mean_acc_skew = common::Utils::getAntiSymmetricMatrix(mean_acc);
    Eigen::Matrix3d delta_rotation = common::Utils::Exp(mean_gyro, dt);
    Eigen::Matrix3d Eye3d = Eigen::Matrix3d::Identity();

    F_x_.setIdentity();
    Q_.setZero();
    // Set the state transition matrix, only the first 3 lines are set, the rest are defaulted
    // Set the state transition matrix first line
    F_x_.block<3, 3>(0, 0) = common::Utils::Exp(mean_gyro, -dt); // about d_rot
    F_x_.block<3, 3>(0, 10) = - Eye3d * dt; // about d_bg
    // Set the state transition matrix second line
    F_x_.block<3, 3>(3, 7) = Eye3d * dt; // about d_vel
    // Set the state transition matrix third line
    F_x_.block<3, 3>(7, 0) = -rotation_imu_ * mean_acc_skew * dt; // about d_rot
    F_x_.block<3, 3>(7, 13) = -rotation_imu_ * dt; // about d_ba
    F_x_.block<3, 3>(7, 16) = Eye3d * dt; // about d_gravity
    
    // Set the process noise covariance matrix
    Q_.block<3, 3>(0, 0).diagonal() = gyro_cov_ * dt * dt; // delta_r about n_gyro
    Q_.block<3, 3>(7, 7) = rotation_imu_ * acc_cov_.asDiagonal() * rotation_imu_.transpose() * dt * dt; // delta_v about n_acc
    Q_.block<3, 3>(10, 10).diagonal() = gyro_bias_cov_ * dt * dt; // delta_bg about n_gyro_bias
    Q_.block<3, 3>(13, 13).diagonal() = acc_bias_cov_ * dt * dt; // delta_ba about n_acc_bias
    Q_(6,6) = inv_expo_cov_ * dt * dt; // delta_expo about w_expo

    // Propagate the state and covariance
    Eigen::Matrix<double, DIM_STATE, DIM_STATE> state_covariance = current_state_.getStateCovariance();
    state_covariance = F_x_ * state_covariance * F_x_.transpose() + Q_;
    current_state_.setStateCovariance(state_covariance);

    rotation_imu_ = rotation_imu_ * delta_rotation;

    acc_imu_ = rotation_imu_ * mean_acc + current_state_.getGravity();

    position_imu_ = position_imu_ + velocity_imu_ * dt + 0.5 * acc_imu_ * dt * dt;

    velocity_imu_ = velocity_imu_ + acc_imu_ * dt;

    system_last_gyro_ = mean_gyro;
    system_last_acc_ = acc_imu_;

    imu_poses_.push_back(set_pose6d(undistorted_time_, acc_imu_, mean_gyro, velocity_imu_, position_imu_, rotation_imu_));
}

void Esikf::runBackwardPropagation(sensor::MeasurementGroup& measurement_group) {
    
    Eigen::Matrix3d system_rotation = current_state_.getSystemRotation();
    Eigen::Matrix3d global_lidar_rotation = lidar_imu_rotation_.transpose() * system_rotation.transpose();
    Eigen::Vector3d global_lidar_traslation = lidar_imu_rotation_.transpose() * lidar_imu_translation_;

    measurement_group.lidar_undistorted->points.clear();
    measurement_group.lidar_undistorted->points.resize(measurement_group.lidar_data_current->points.size());
    *(measurement_group.lidar_undistorted) = *(measurement_group.lidar_data_current);

    auto point = measurement_group.lidar_undistorted->points.end() - 1;
    Eigen::Matrix3d imu_rotation;
    Eigen::Vector3d imu_acc;
    Eigen::Vector3d imu_vel;
    Eigen::Vector3d imu_pos;
    Eigen::Vector3d imu_gyro;
    double delta_time = 0.0;

    for (auto imu_pose_it = imu_poses_.end() -1; imu_pose_it != imu_poses_.begin(); imu_pose_it--) {
        
        auto last_imu = imu_pose_it - 1;

        imu_rotation << MAT_FROM_ARRAY(last_imu->rot);
        imu_acc << VEC_FROM_ARRAY(last_imu->acc);
        imu_vel << VEC_FROM_ARRAY(last_imu->vel);
        imu_pos << VEC_FROM_ARRAY(last_imu->pos);
        imu_gyro << VEC_FROM_ARRAY(last_imu->gyr);

        for (; point->timestamp / double(1000) > last_imu->offset_time; point--) {
            delta_time = point->timestamp / double(1000) - last_imu->offset_time;

            Eigen::Matrix3d delta_rotation = imu_rotation * common::Utils::Exp(imu_gyro, delta_time);
            Eigen::Vector3d delta_translation = imu_pos + imu_vel * delta_time + 0.5 * imu_acc * delta_time * delta_time - current_state_.getSystemPosition();

            Eigen::Vector3d lidar_point(point->x, point->y, point->z);
            Eigen::Vector3d lidar_point_compensate = (global_lidar_rotation * 
                                                    (delta_rotation * (lidar_imu_rotation_ * lidar_point + lidar_imu_translation_) + delta_translation) - 
                                                        global_lidar_traslation);
            point->x = lidar_point_compensate(0);
            point->y = lidar_point_compensate(1);
            point->z = lidar_point_compensate(2);
            if (point == measurement_group.lidar_undistorted->points.begin()) {
                break;
            }
        }
    }

}

void Esikf::updateStateWithLidar() {

    pcl::PointCloud<PointXYZIRT>::Ptr lidar_downsampled_cloud = map_manager_.getDownsampledLidarCloud();
    int num_downsample_points = lidar_downsampled_cloud->points.size();
    map_manager_.getSkewSymmMatricesList().clear();
    map_manager_.getSkewSymmMatricesList().reserve(num_downsample_points);
    map_manager_.getLidarPointCovariancesList().clear();
    map_manager_.getLidarPointCovariancesList().reserve(num_downsample_points);

    for (int i = 0; i < num_downsample_points; i++) {
        Eigen::Vector3d lidar_point(lidar_downsampled_cloud->points[i].x, 
                                    lidar_downsampled_cloud->points[i].y, 
                                    lidar_downsampled_cloud->points[i].z);
        if (lidar_point[2] == 0) {
            lidar_point[2] = 0.001; // avoid division by zero
        }
        Eigen::Matrix3d point_covariance;
        map_manager_.calculateLidarPointCovariance(lidar_point, point_covariance);
        map_manager_.getLidarPointCovariancesList().push_back(point_covariance);
        lidar_point = lidar_imu_rotation_ * lidar_point + lidar_imu_translation_; // lidar point in imu frame
        Eigen::Matrix3d skew_symm_matrix = common::Utils::getAntiSymmetricMatrix(lidar_point);
        map_manager_.getSkewSymmMatricesList().push_back(skew_symm_matrix);
    }

    std::vector<map::PointWithCovarience>& points_with_covariances = map_manager_.getPointsWithCovariancesList();
    std::vector<map::PointWithCovarience>().swap(points_with_covariances);
    points_with_covariances.resize(num_downsample_points);
    
    int rematch_num = 0;
    Eigen::Matrix<double, DIM_STATE, DIM_STATE> G, H_T_H, I_STATE;
    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();

    bool flg_EKF_converged, EKF_stop_flg = 0;

    int max_iter = map_manager_.getMaximumIteration();

    for (int iterCount = 0; iterCount < max_iter; iterCount++) {

        double total_residual = 0.0;
        pcl::PointCloud<PointXYZIRT>::Ptr world_lidar(new pcl::PointCloud<PointXYZIRT>);
        transformLidarToGlobal(lidar_downsampled_cloud, world_lidar);

        Eigen::Matrix3d rotation_covariance = current_state_.getStateCovariance().block<3, 3>(0, 0);
        Eigen::Matrix3d position_covariance = current_state_.getStateCovariance().block<3, 3>(3, 3);
        Eigen::Matrix3d system_rotation = current_state_.getSystemRotation();

        for (int i = 0; i < num_downsample_points; i++) {

            map::PointWithCovarience& point = map_manager_.getPointsWithCovariancesList()[i];
            point.lidar_point << lidar_downsampled_cloud->points[i].x,
                lidar_downsampled_cloud->points[i].y, lidar_downsampled_cloud->points[i].z;
            point.global_point << world_lidar->points[i].x, world_lidar->points[i].y,
                world_lidar->points[i].z;

            Eigen::Matrix3d cov = map_manager_.getLidarPointCovariancesList()[i];
            Eigen::Matrix3d point_crossmat = map_manager_.getSkewSymmMatricesList()[i];
            cov = system_rotation * cov * system_rotation.transpose() +
                    (-point_crossmat) * rotation_covariance * (-point_crossmat.transpose()) + position_covariance;
            point.global_point_covariance = cov;
            point.lidar_point_covariance = map_manager_.getLidarPointCovariancesList()[i];
        }

        buildLidarResidual(map_manager_.getPointsWithCovariancesList(), map_manager_.getPointToPlaneList());

        for (size_t i = 0; i < map_manager_.getPointToPlaneList().size(); i++) {
            total_residual += std::fabs(map_manager_.getPointToPlaneList()[i].distance_to_plane);
           
        }

        
        effctive_point_count_ = map_manager_.getPointToPlaneList().size();

        common::Utils::printColored(" downsampled point num: " + std::to_string(num_downsample_points), common::Color::kBlue, common::Style::kBold);
        common::Utils::printColored(" effective point num: " + std::to_string(effctive_point_count_), common::Color::kBlue, common::Style::kBold);   
        common::Utils::printColored(" average residual: " + std::to_string(total_residual / effctive_point_count_), common::Color::kBlue, common::Style::kBold);        
        
        Eigen::MatrixXd Hsub(effctive_point_count_, 6);
        Eigen::MatrixXd Hsub_T_R_inv(6, effctive_point_count_);
        Eigen::VectorXd R_inv(effctive_point_count_);
        Eigen::VectorXd meas_vec(effctive_point_count_);
        meas_vec.setZero();

        for (int i = 0; i < effctive_point_count_; i++) {
            auto& ptpl = map_manager_.getPointToPlaneList()[i];
            Eigen::Vector3d point_this(ptpl.lidar_point);
            point_this = lidar_imu_rotation_ * point_this + lidar_imu_translation_;
            Eigen::Matrix3d point_crossmat;
            point_crossmat << common::Utils::getAntiSymmetricMatrix(point_this);
            
            Eigen::Vector3d global_point = propagate_state_.getSystemRotation() * point_this + propagate_state_.getSystemPosition();
            Eigen::Matrix<double, 1, 6> J_nq;
            J_nq.block<1, 3>(0, 0) = global_point - ptpl.plane_center;
            J_nq.block<1, 3>(0, 3) = -ptpl.plane_normal;

            Eigen::Matrix3d global_point_cov;
            global_point_cov = propagate_state_.getSystemRotation() * lidar_imu_rotation_ * 
                                    ptpl.lidar_point_covariance * 
                                (propagate_state_.getSystemRotation() * lidar_imu_rotation_).transpose();
            double sigma_l = J_nq * ptpl.plane_covariance * J_nq.transpose();

            R_inv(i) = 1.0 / (0.001 + sigma_l +
                                ptpl.plane_normal.transpose() * global_point_cov * ptpl.plane_normal);

            Eigen::Vector3d A(point_crossmat * current_state_.getSystemRotation().transpose() * ptpl.plane_normal);
            Hsub.row(i) << VEC_FROM_ARRAY(A), ptpl.plane_normal[0], ptpl.plane_normal[1], ptpl.plane_normal[2];
            Hsub_T_R_inv.col(i) << A[0] * R_inv(i), A[1] * R_inv(i), A[2] * R_inv(i),
                ptpl.plane_normal[0] * R_inv(i),
                ptpl.plane_normal[1] * R_inv(i),
                ptpl.plane_normal[2] * R_inv(i);
            meas_vec(i) = -ptpl.distance_to_plane;
        }
        EKF_stop_flg = false;
        flg_EKF_converged = false;

        auto &&HTz = Hsub_T_R_inv * meas_vec;

        H_T_H.block<6, 6>(0, 0) = Hsub_T_R_inv * Hsub;

        Eigen::Matrix<double, DIM_STATE, DIM_STATE>&& K = (H_T_H + current_state_.getStateCovariance().inverse()).inverse();
        
        G.block<DIM_STATE, 6>(0, 0) = K.block<DIM_STATE, 6>(0, 0) * H_T_H.block<6, 6>(0, 0);

        auto vec = propagate_state_ - current_state_;

        Eigen::Matrix<double, DIM_STATE, 1> delta_state = K.block<DIM_STATE, 6>(0, 0) * HTz +
                                                        vec.block<DIM_STATE, 1>(0, 0) -
                                                        G.block<DIM_STATE, 6>(0, 0) * vec.block<6, 1>(0, 0);

        current_state_ += delta_state;

        auto rot_add = delta_state.block<3, 1>(0, 0);
        auto t_add = delta_state.block<3, 1>(3, 0);
        if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015)) {
            flg_EKF_converged = true;
        }
        
        if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == (max_iter - 2)))) {
            rematch_num++;
        }  

        if (!EKF_stop_flg && (rematch_num >= 2 ||
                            (iterCount == max_iter - 1))) {
        /*** Covariance Update ***/
            auto state_cov = (I_STATE - G) * current_state_.getStateCovariance();
            current_state_.setStateCovariance(state_cov);
            EKF_stop_flg = true;
        }
        if (EKF_stop_flg) break;
    }
}

void Esikf::buildLidarResidual(std::vector<map::PointWithCovarience>& pc_list, std::vector<map::PointToPlane>& ptpl_list) {
    double map_voxel_size =  map_manager_.getVoxelMapConfig().map_voxel_size;

    ptpl_list.clear();
    std::vector<map::PointToPlane> all_ptpl_list(pc_list.size());
    std::vector<bool> useful_ptpl(pc_list.size());
    std::vector<size_t> index(pc_list.size());

    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
        useful_ptpl[i] = false;
    }

    for (size_t i = 0; i < index.size(); i++) {
        map::PointWithCovarience& point = pc_list[i];
        float local_xyz[3];
        for (int j = 0; j < 3; j++) {
            local_xyz[j] = point.global_point[j] / map_voxel_size;
            if (local_xyz[j] < 0) {
                local_xyz[j] -= 1.0;
            }
        }
        VoxelLocation position((int64_t)local_xyz[0], (int64_t)local_xyz[1], (int64_t)local_xyz[2]);
        auto iter = map_manager_.getVoxelMap().find(position);
        if (iter != map_manager_.getVoxelMap().end()) {
            map::VoxelOctoTree* current_octo = iter->second->second;
            map::PointToPlane single_ptpl;
            bool is_sucess = false;
            double prob = 0;
            buildPointResidual(point, current_octo, 0, is_sucess, prob, single_ptpl);
            if (!is_sucess) {
                VoxelLocation near_position = position;
                if (local_xyz[0] >
                    (current_octo->getVoxelCenter()[0] + current_octo->getQuaterLength())) {
                    near_position.x = near_position.x + 1;
                } else if (local_xyz[0] < (current_octo->getVoxelCenter()[0] -
                                        current_octo->getQuaterLength())) {
                    near_position.x = near_position.x - 1;
                }
                if (local_xyz[1] >
                    (current_octo->getVoxelCenter()[1] + current_octo->getQuaterLength())) {
                    near_position.y = near_position.y + 1;
                } else if (local_xyz[1] < (current_octo->getVoxelCenter()[1] -
                                        current_octo->getQuaterLength())) {
                    near_position.y = near_position.y - 1;
                }
                if (local_xyz[2] >
                    (current_octo->getVoxelCenter()[2] + current_octo->getQuaterLength())) {
                    near_position.z = near_position.z + 1;
                } else if (local_xyz[2] < (current_octo->getVoxelCenter()[2] -
                                        current_octo->getQuaterLength())) {
                    near_position.z = near_position.z - 1;
                }
                auto iter_near = map_manager_.getVoxelMap().find(near_position);
                if (iter_near != map_manager_.getVoxelMap().end()) {
                    buildPointResidual(point, iter_near->second->second, 0, is_sucess, prob, single_ptpl);
                }
            }
            if (is_sucess) {
                useful_ptpl[i] = true;
                all_ptpl_list[i] = single_ptpl;
            } else {
                useful_ptpl[i] = false;
            }
        }
    }

    for (size_t i = 0; i < useful_ptpl.size(); i++) {
        if (useful_ptpl[i]) {
            ptpl_list.push_back(all_ptpl_list[i]);
        }
        
    }
}

void Esikf::buildPointResidual(map::PointWithCovarience& point,
                                const map::VoxelOctoTree* current_octo,
                                const int current_layer,
                                bool& is_sucess, double& prob,
                                map::PointToPlane& single_ptpl) {
    int octree_max_depth = map_manager_.getVoxelMapConfig().octree_max_depth;
    double sigma_num = map_manager_.getVoxelMapConfig().sigma_num;

    double radius_k = 3;
    Eigen::Vector3d global_point = point.global_point;
    if (current_octo->getPlanePtr()->is_plane) {
        map::VoxelPlane& plane = *current_octo->getPlanePtr();

        float dis_to_plane = std::fabs(plane.plane_normal(0) * global_point(0) + 
                                        plane.plane_normal(1) * global_point(1) +
                                        plane.plane_normal(2) * global_point(2) + plane.plane_intercept);
        float dis_to_center =
            (plane.plane_center(0) - global_point(0)) * (plane.plane_center(0) - global_point(0)) +
            (plane.plane_center(1) - global_point(1)) * (plane.plane_center(1) - global_point(1)) +
            (plane.plane_center(2) - global_point(2)) * (plane.plane_center(2) - global_point(2));
        float range_dis = std::sqrt(dis_to_center - dis_to_plane * dis_to_plane);

        if (range_dis <= radius_k * plane.radius) {
            Eigen::Matrix<double, 1, 6> J_nq;
            J_nq.block<1, 3>(0, 0) = global_point - plane.plane_center;
            J_nq.block<1, 3>(0, 3) = -plane.plane_normal;
            double sigma_l = J_nq * plane.plane_covariance * J_nq.transpose();
            sigma_l += plane.plane_normal.transpose() * point.global_point_covariance * plane.plane_normal;
            if (dis_to_plane < sigma_num * std::sqrt(sigma_l)) {
                is_sucess = true;
                double this_prob = 1.0 / (std::sqrt(sigma_l)) *
                                std::exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
                if (this_prob > prob) {
                    prob = this_prob;
                    point.normal = plane.plane_normal;
                    single_ptpl.lidar_point_covariance = point.lidar_point_covariance;
                    single_ptpl.lidar_point = point.lidar_point;
                    single_ptpl.global_point = point.global_point;
                    single_ptpl.plane_covariance = plane.plane_covariance;
                    single_ptpl.plane_normal = plane.plane_normal;
                    single_ptpl.plane_center = plane.plane_center;
                    single_ptpl.plane_intercept = plane.plane_intercept;
                    single_ptpl.layer = current_layer;
                    single_ptpl.distance_to_plane = plane.plane_normal(0) * global_point(0) +
                                                plane.plane_normal(1) * global_point(1) +
                                                plane.plane_normal(2) * global_point(2) + plane.plane_intercept;
                }
                return;
            } else {
                return;
            }
        } else {
            return;
        }
    } else {
        if (current_layer < octree_max_depth) {
            for (size_t leafnum = 0; leafnum < 8; leafnum++) {
                if (current_octo->getLeaves(leafnum) != nullptr) {
                    map::VoxelOctoTree* leaf_octo = current_octo->getLeaves(leafnum);
                    buildPointResidual(point, leaf_octo, current_layer + 1, is_sucess, prob,
                                        single_ptpl);
                }
            }
            return;
        } else {
            return;
        }
    }
}

void Esikf::updateStateWithVision(cv::Mat img, int level) {

    int total_points = map_manager_.getVisualMapPtr()->getTotalPoints();
    if (total_points == 0) return;
    int pixels_per_patch = map_manager_.getVisualMapPtr()->getPixelsPerPatch();

    SystemState old_state = current_state_;

    Eigen::VectorXd z;
    Eigen::MatrixXd H_sub;
    bool EKF_end = false;
    float last_error = std::numeric_limits<float>::max();

    const int H_DIM = total_points * pixels_per_patch;
    z.resize(H_DIM);
    z.setZero();
    H_sub.resize(H_DIM, 7);  // R, t, τ
    H_sub.setZero();

    Eigen::Matrix3d& Rci = map_manager_.getVisualMapPtr()->getRci();
    Eigen::Vector3d& Pci = map_manager_.getVisualMapPtr()->getPci();
    Eigen::Matrix3d& Rcw = map_manager_.getVisualMapPtr()->getRcw();
    Eigen::Vector3d& Pcw = map_manager_.getVisualMapPtr()->getPcw();
    Eigen::Matrix3d& Jdphi_dR = map_manager_.getVisualMapPtr()->getJdphidR();  // ∂φ/∂R
    Eigen::Matrix3d& Jdp_dR = map_manager_.getVisualMapPtr()->getJdpdR();  // ∂p/∂R
    Eigen::Matrix3d& Jdp_dt = map_manager_.getVisualMapPtr()->getJdpdt();  // ∂p/∂t
    int max_iterations =  map_manager_.getMaximumIteration();


    map::SubSparseMap* visual_submap = map_manager_.getVisualMapPtr()->getVisualSubmap();
    vk::AbstractCamera *cam = map_manager_.getVisualMapPtr()->getAbstractCamera();
    Eigen::Matrix<double, DIM_STATE, DIM_STATE> G, H_T_H;

    for (int iteration = 0; iteration < max_iterations; iteration++) {

        Eigen::Matrix3d Rwi(current_state_.getSystemRotation());
        Eigen::Vector3d Pwi(current_state_.getSystemPosition());
        Rcw = Rci * Rwi.transpose();
        Pcw = -Rci * Rwi.transpose() * Pwi + Pci;
        Jdp_dt = Rci * Rwi.transpose();

        float error = 0.0;
        int n_meas = 0;
        for (int i = 0; i < total_points; i++) {
            Eigen::Matrix<double, 1, 2> Jimg; 
            Eigen::Matrix<double, 2, 3> Jdpi;
            Eigen::Matrix<double, 1, 3> Jdphi, Jdp, JdR, Jdt;

            float patch_error = 0.0;
            int search_level = visual_submap->search_levels[i];
            int pyramid_level = level + search_level;
            int scale = (1 << pyramid_level);
            float inv_scale = 1.0f / scale;

            visual::Point *pt = visual_submap->visual_points[i];

            if (pt == nullptr) continue; // Skip if point is not found

            Eigen::Vector3d pf = Rcw * pt->getPosition() + Pcw;
            Eigen::Vector2d pc = cam->world2cam(pf);

            buildVProjectionJacobian(pf, Jdpi);
            Eigen::Matrix3d p_hat = common::Utils::getAntiSymmetricMatrix(pf); 
            // Floating point coordinates in the image
            float u_ref = pc[0];
            float v_ref = pc[1];
            // Coordinate integer part
            int u_ref_i = floorf(pc[0] / scale) * scale;
            int v_ref_i = floorf(pc[1] / scale) * scale;
            // Decimal part of coordinates
            float subpix_u_ref = (u_ref - u_ref_i) / scale;
            float subpix_v_ref = (v_ref - v_ref_i) / scale;
            // Weighting coefficients for bilinear interpolation
            float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
            float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref);
            float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
            float w_ref_br = subpix_u_ref * subpix_v_ref;

            // Reference frame intensity and exposure time
            std::vector<float> P = visual_submap->warp_patch[i];
            double inv_ref_expo = visual_submap->inv_expo_list[i];

            int patch_size = map_manager_.getVisualMapPtr()->getPatchSize();
            int half_patch_size = patch_size / 2;
            for (int x = 0; x < patch_size; x++) {
                uint8_t *img_ptr = (uint8_t *)img.data + 
                                    (v_ref_i + x * scale - half_patch_size * scale) * cam->width() +
                                    u_ref_i - half_patch_size * scale;
                for (int y = 0; y < patch_size; ++y, img_ptr += scale) {
                    // I_k
                    double cur_value = w_ref_tl * img_ptr[0] + w_ref_tr * img_ptr[scale] +
                                        w_ref_bl * img_ptr[scale * cam->width()] +
                                        w_ref_br * img_ptr[scale * cam->width() + scale];
                    // Formula (22) ,z = τ_k * I_k - τ_r * I_r
                    double res = current_state_.getInverseExposureTime() * cur_value -
                        inv_ref_expo * P[pixels_per_patch * level + x * patch_size + y];
                    z(i * pixels_per_patch + x * patch_size + y) = res;
                    patch_error += res * res;
                    n_meas += 1;
                    visual_submap->errors[i] = patch_error;
                    error += patch_error;
                    
                    // Pixel gradient calculation，∂I/∂u
                    float du =
                        0.5f *
                        ((w_ref_tl * img_ptr[scale] + w_ref_tr * img_ptr[scale * 2] +
                            w_ref_bl * img_ptr[scale * cam->width() + scale] +
                            w_ref_br * img_ptr[scale * cam->width() + scale * 2]) -
                        (w_ref_tl * img_ptr[-scale] + w_ref_tr * img_ptr[0] +
                            w_ref_bl * img_ptr[scale * cam->width() - scale] +
                            w_ref_br * img_ptr[scale * cam->width()]));
                    float dv =
                        0.5f * ((w_ref_tl * img_ptr[scale * cam->width()] +
                                w_ref_tr * img_ptr[scale + scale * cam->width()] +
                                w_ref_bl * img_ptr[cam->width() * scale * 2] +
                                w_ref_br * img_ptr[cam->width() * scale * 2 + scale]) -
                                (w_ref_tl * img_ptr[-scale * cam->width()] +
                                w_ref_tr * img_ptr[-scale * cam->width() + scale] +
                                w_ref_bl * img_ptr[0] + w_ref_br * img_ptr[scale]));
                    
                    Jimg << du, dv;
                    Jimg = Jimg * current_state_.getInverseExposureTime();
                    Jimg = Jimg * inv_scale;

                    Jdphi = Jimg * Jdpi * p_hat;
                    Jdp = -Jimg * Jdpi;
                    JdR = Jdphi * Jdphi_dR + Jdp * Jdp_dR;
                    Jdt = Jdp * Jdp_dt;

                    H_sub.block<1, 7>(i * pixels_per_patch + x * patch_size + y, 0)<< JdR, Jdt, cur_value;
                }
            }
        }
        error = error / n_meas;

        if (error <= last_error) {
            old_state = current_state_;
            last_error = error;
            auto &&H_sub_T = H_sub.transpose();
            H_T_H.setZero();
            G.setZero();
            H_T_H.block<7, 7>(0, 0) = H_sub_T * H_sub;
            Eigen::Matrix<double, DIM_STATE, DIM_STATE>&& K = (H_T_H + (current_state_.getStateCovariance() / photometric_noise_).inverse()).inverse();
            auto &&HTz = H_sub_T * z; 
            auto vec = propagate_state_ - current_state_;
            G.block<DIM_STATE, 7>(0, 0) = K.block<DIM_STATE, 7>(0, 0) * H_T_H.block<7, 7>(0, 0);
            Eigen::Matrix<double, DIM_STATE, 1> delta_state = -K.block<DIM_STATE, 7>(0, 0) * HTz + vec -
                        G.block<DIM_STATE, 7>(0, 0) * vec.block<7, 1>(0, 0);
            current_state_ += delta_state;

            common::Utils::fout_dt << propagate_state_.getSystemRotation() << std::endl;
            auto &&rot_add = delta_state.block<3, 1>(0, 0);
            auto &&t_add = delta_state.block<3, 1>(3, 0);

            if ((rot_add.norm() * 57.3f < 0.001f) && (t_add.norm() * 100.0f < 0.001f)) {
                EKF_end = true;
            }  
        } else {
            current_state_ = old_state;
            EKF_end = true;
        }
        if (iteration == max_iterations || EKF_end) break;
    }


    if (level == 0) {
        auto state_covariance = current_state_.getStateCovariance() - G * current_state_.getStateCovariance();
        current_state_.setStateCovariance(state_covariance);
    }
}

void Esikf::transformLidarToGlobal(const pcl::PointCloud<PointXYZIRT>::Ptr input_cloud, pcl::PointCloud<PointXYZIRT>::Ptr output_cloud) {
    pcl::PointCloud<PointXYZIRT>().swap(*output_cloud);

    output_cloud->reserve(input_cloud->size());
    const Eigen::Matrix3d& rot = current_state_.getSystemRotation();
    const Eigen::Vector3d& t = current_state_.getSystemPosition();

    for (size_t i = 0; i < input_cloud->size(); i++) {
        PointXYZIRT point = input_cloud->points[i];
        Eigen::Vector3d p(point.x, point.y, point.z);
        p = (rot * (lidar_imu_rotation_ * p + lidar_imu_translation_) + t);
        PointXYZIRT pi;
        pi.x = p(0);
        pi.y = p(1);
        pi.z = p(2);
        pi.intensity = point.intensity;
        pi.ring = point.ring;
        pi.timestamp = point.timestamp;
        output_cloud->points.push_back(pi);
    }
}

SystemState& Esikf::getCurrentState() {
    return current_state_;
}

void Esikf::setPropagateState(const SystemState& state) {
    propagate_state_ = state;
}

map::MapManager& Esikf::getMapManager() {
    return map_manager_;
}

void Esikf::buildVProjectionJacobian(Eigen::Vector3d p, Eigen::Matrix<double, 2, 3>& J) {
    const double x = p[0];
    const double y = p[1];
    const double z_inv = 1. / p[2];
    const double z_inv_2 = z_inv * z_inv;
    const double fx = map_manager_.getVisualMapPtr()->getAbstractCamera()->fx();
    const double fy = map_manager_.getVisualMapPtr()->getAbstractCamera()->fy();

    J(0, 0) = fx * z_inv;
    J(0, 1) = 0.0;
    J(0, 2) = -fx * x * z_inv_2;
    J(1, 0) = 0.0;
    J(1, 1) = fy * z_inv;
    J(1, 2) = -fy * y * z_inv_2;
}

Esikf::~Esikf() {
    // Destructor implementation
}

}