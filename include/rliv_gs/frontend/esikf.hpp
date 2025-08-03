/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef ESIKF_HPP
#define ESIKF_HPP

#include "system_state.hpp"
#include "rliv_gs/sensor/sensor_measurement.hpp"
#include "rliv_gs/map/map_manager.hpp"

#include <vikit/math_utils.h>

namespace frontend {

class Esikf {
public:
    Esikf();
    ~Esikf();

    bool isImuInitialized(const common::Config& config, sensor::MeasurementGroup& measurement_group);
    bool pridictState(sensor::MeasurementGroup& measurement_group);
    void updateStateWithLidar();
    void updateStateWithVision(cv::Mat img, int level);


    // Set state transition matrix F_x
    void runForwardPropagation(double dt, Eigen::Vector3d acc, Eigen::Vector3d gyro);
    void runBackwardPropagation(sensor::MeasurementGroup& measurement_group);

    // Calculate the residual of lidar measurements
    void buildLidarResidual(std::vector<map::PointWithCovarience>& pc_list, std::vector<map::PointToPlane>& ptpl_list);
    void buildPointResidual(map::PointWithCovarience& point,
                            const map::VoxelOctoTree* current_octo,
                            const int current_layer,
                            bool& is_sucess, double& prob,
                            map::PointToPlane& single_ptpl);
    void buildVProjectionJacobian(Eigen::Vector3d p, Eigen::Matrix<double, 2, 3>& J);

    void transformLidarToGlobal(const pcl::PointCloud<PointXYZIRT>::Ptr input_cloud, pcl::PointCloud<PointXYZIRT>::Ptr output_cloud);

    // Getters
    SystemState& getCurrentState();
    void setPropagateState(const SystemState& state);

    map::MapManager& getMapManager();

private:
    SystemState current_state_; // Current system state
    SystemState propagate_state_; // Previous system state

    map::MapManager map_manager_; // Map manager

    // Initialize the IMU
    bool first_initialization_ = true; // Flag to check if it's the first initialization
    double gravity_const_ = 9.81; // Gravity constant
    float imu_init_data_count_ = 0.0f;
    float imu_init_threshold_ = 30.0f; // Threshold for IMU initialization, can be adjusted based on requirements
    Eigen::Matrix3d rotation_imu_; // Imu Rotation
    Eigen::Vector3d position_imu_; // Imu Translation
    Eigen::Vector3d velocity_imu_; // Imu Velocity
    Eigen::Vector3d acc_imu_; // Imu Accelerometer
    Eigen::Vector3d gyro_imu_; // Imu Gyroscope
    Eigen::Vector3d imu_mean_acc_;
    Eigen::Vector3d imu_mean_gyro_;
    double undistorted_time_;
    sensor_msgs::ImuConstPtr last_imu_;

    // pridict the state 
    double last_prediction_end_time_; // End time of the last prediction
    Eigen::Vector3d system_last_acc_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d system_last_gyro_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_cov_;
    Eigen::Vector3d gyro_cov_;
    Eigen::Vector3d acc_bias_cov_;
    Eigen::Vector3d gyro_bias_cov_;
    double inv_expo_cov_;
    Eigen::Matrix<double, DIM_STATE, DIM_STATE> F_x_ = Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Identity(); // State transition matrix
    // process noise covariance matrix
    Eigen::Matrix<double, DIM_STATE, DIM_STATE> Q_ = Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Zero();

    // Backward propagation
    std::vector<Pose6D> imu_poses_;
    Eigen::Matrix3d lidar_imu_rotation_;
    Eigen::Vector3d lidar_imu_translation_;

    // Update the state with lidar measurements
    int effctive_point_count_ = 0;
    bool first_pridiction_ = true;

    // Update the state with vision measurements
    double photometric_noise_;

};

}

#endif // ESIKF_HPP