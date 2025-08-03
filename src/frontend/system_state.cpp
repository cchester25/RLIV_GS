/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "rliv_gs/frontend/system_state.hpp"

namespace frontend {

SystemState::SystemState() {
    rotation_.setIdentity();
    position_.setZero();
    velocity_.setZero();
    imu_gyro_bias_.setZero();
    imu_acc_bias_.setZero();
    gravity_.setZero();
    inverse_exposure_time_ = 1.0;
    state_covariance_ = Eigen::MatrixXd::Identity(DIM_STATE, DIM_STATE) * 0.01;
    state_covariance_(6, 6) = 0.00001;
    state_covariance_.block<9, 9>(10, 10) = Eigen::MatrixXd::Identity(9, 9) * 0.00001;
}

SystemState& SystemState::operator=(const SystemState& other) {

    this->rotation_ = other.getSystemRotation();
    this->position_ = other.getSystemPosition();
    this->velocity_ = other.getSystemVelocity();
    this->imu_gyro_bias_ = other.getImuGyroBias();
    this->imu_acc_bias_ = other.getImuAccBias();
    this->gravity_ = other.getGravity();
    this->inverse_exposure_time_ = other.getInverseExposureTime();
    this->state_covariance_ = other.getStateCovariance();

    return *this;
}

SystemState SystemState::operator+(const SystemState& other) {
    SystemState result;

    result.setSystemRotation(this->rotation_ * other.getSystemRotation());
    result.setSystemPosition(this->position_ + other.getSystemPosition());
    result.setSystemVelocity(this->velocity_ + other.getSystemVelocity());
    result.setImuGyroBias(this->imu_gyro_bias_ + other.getImuGyroBias());
    result.setImuAccBias(this->imu_acc_bias_ + other.getImuAccBias());
    result.setGravity(this->gravity_ + other.getGravity());
    result.setInverseExposureTime(this->inverse_exposure_time_ + other.getInverseExposureTime());
    result.setStateCovariance(this->state_covariance_);
    return result;
}

Eigen::Matrix<double, DIM_STATE, 1> SystemState::operator-(const SystemState& other) {
    Eigen::Matrix<double, DIM_STATE, 1> a;
    Eigen::Matrix3d rotd(other.getSystemRotation().transpose() * this->rotation_);

    a.block<3, 1>(0, 0) = common::Utils::Log(rotd);
    a.block<3, 1>(3, 0) = this->position_ - other.getSystemPosition();
    a.block<3, 1>(7, 0) = this->velocity_ - other.getSystemVelocity();
    a.block<3, 1>(10, 0) = this->imu_gyro_bias_ - other.getImuGyroBias();
    a.block<3, 1>(13, 0) = this->imu_acc_bias_ - other.getImuAccBias();
    a.block<3, 1>(16, 0) = this->gravity_ - other.getGravity();

    a(6, 0) = this->inverse_exposure_time_ - other.getInverseExposureTime();

    return a;
}

SystemState& SystemState::operator+=(const Eigen::Matrix<double, DIM_STATE, 1> &state_add) {
    
    this->rotation_ = this->rotation_ * common::Utils::Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    this->position_ += state_add.block<3, 1>(3, 0);
    this->velocity_ += state_add.block<3, 1>(7, 0);
    this->imu_gyro_bias_ += state_add.block<3, 1>(10, 0);
    this->imu_acc_bias_ += state_add.block<3, 1>(13, 0);
    this->gravity_ += state_add.block<3, 1>(16, 0);
    this->inverse_exposure_time_ += state_add(6, 0);

    return *this;
}

const Eigen::Matrix3d& SystemState::getSystemRotation() const {
    return rotation_;
}

const Eigen::Vector3d& SystemState::getSystemPosition() const {
    return position_;
}

const Eigen::Vector3d& SystemState::getSystemVelocity() const {
    return velocity_;
}

const Eigen::Vector3d& SystemState::getImuGyroBias() const {
    return imu_gyro_bias_;
}

const Eigen::Vector3d& SystemState::getImuAccBias() const {
    return imu_acc_bias_;
}

const Eigen::Vector3d& SystemState::getGravity() const {
    return gravity_;
}

const double& SystemState::getInverseExposureTime() const {
    return inverse_exposure_time_;
}

const Eigen::Matrix<double, DIM_STATE, DIM_STATE>& SystemState::getStateCovariance() const {
    return state_covariance_;
}

void SystemState::setSystemRotation(Eigen::Matrix3d rotation) {
    rotation_ = rotation;
}

void SystemState::setSystemPosition(Eigen::Vector3d position) {
    position_ = position;
}

void SystemState::setSystemVelocity(Eigen::Vector3d velocity) {
    velocity_ = velocity;
}

void SystemState::setImuGyroBias(Eigen::Vector3d imu_gyro_bias) {
    imu_gyro_bias_ = imu_gyro_bias;
}

void SystemState::setImuAccBias(Eigen::Vector3d imu_acc_bias) {
    imu_acc_bias_ = imu_acc_bias;
}

void SystemState::setGravity(Eigen::Vector3d gravity) {
    gravity_ = gravity;
}


void SystemState::setInverseExposureTime(double inverse_exposure_time) {
    inverse_exposure_time_ = inverse_exposure_time;
}

void SystemState::setStateCovariance(Eigen::Matrix<double, DIM_STATE, DIM_STATE> state_covariance) {
    state_covariance_ = state_covariance;
}

SystemState::~SystemState() {
    // Destructor implementation if needed
}

}