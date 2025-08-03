/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef SYSTEM_STATE_HPP
#define SYSTEM_STATE_HPP

#include <Eigen/Eigen>

#include "rliv_gs/common/config.hpp"

namespace frontend {

#define DIM_STATE (19)

class SystemState {
public:
  SystemState();
  ~SystemState();
  SystemState& operator=(const SystemState& other);
  SystemState operator+(const SystemState& other);
  Eigen::Matrix<double, DIM_STATE, 1> operator-(const SystemState& other);
  SystemState& operator+=(const Eigen::Matrix<double, DIM_STATE, 1> &state_add);

  const Eigen::Matrix3d& getSystemRotation() const;
  const Eigen::Vector3d& getSystemPosition() const;
  const Eigen::Vector3d& getSystemVelocity() const;
  const Eigen::Vector3d& getImuGyroBias() const;
  const Eigen::Vector3d& getImuAccBias() const;
  const Eigen::Vector3d& getGravity() const;
  const double& getInverseExposureTime() const;
  const Eigen::Matrix<double, DIM_STATE, DIM_STATE>& getStateCovariance() const;

  void setSystemRotation(Eigen::Matrix3d rotation);
  void setSystemPosition(Eigen::Vector3d position);
  void setSystemVelocity(Eigen::Vector3d velocity);
  void setImuGyroBias(Eigen::Vector3d imu_gyro_bias);
  void setImuAccBias(Eigen::Vector3d imu_acc_bias);
  void setGravity(Eigen::Vector3d gravity);
  void setInverseExposureTime(double inverse_exposure_time);
  void setStateCovariance(Eigen::Matrix<double, DIM_STATE, DIM_STATE> state_covariance);

private:

  Eigen::Matrix3d rotation_; // system rotation matrix
  Eigen::Vector3d position_; // system position vector
  Eigen::Vector3d velocity_; // system velocity vector
  Eigen::Vector3d imu_gyro_bias_; // imu gyroscope bias
  Eigen::Vector3d imu_acc_bias_; // imu accelerometer bias
  Eigen::Vector3d gravity_; // gravity vector
  double inverse_exposure_time_; // inverse exposure time

  Eigen::Matrix<double, DIM_STATE, DIM_STATE> state_covariance_; // state covariance matrix

};

// IMU pose for backpropagation
struct Pose6D
{
  double offset_time; // the offset time of IMU measurement w.r.t the first lidar point
  double acc[3];      // the preintegrated total acceleration (global frame) at the Lidar origin
  double gyr[3];      // the unbiased angular velocity (body frame) at the Lidar origin
  double vel[3];      // the preintegrated velocity (global frame) at the Lidar origin
  double pos[3];      // the preintegrated position (global frame) at the Lidar origin
  double rot[9];      // the preintegrated rotation (global frame) at the Lidar origin
};

template <typename T>
auto set_pose6d(const double t, 
                const Eigen::Matrix<T, 3, 1> &a, 
                const Eigen::Matrix<T, 3, 1> &g, 
                const Eigen::Matrix<T, 3, 1> &v, 
                const Eigen::Matrix<T, 3, 1> &p,
                const Eigen::Matrix<T, 3, 3> &R)
{
  Pose6D rot_kp;
  rot_kp.offset_time = t;
  for (int i = 0; i < 3; i++)
  {
    rot_kp.acc[i] = a(i);
    rot_kp.gyr[i] = g(i);
    rot_kp.vel[i] = v(i);
    rot_kp.pos[i] = p(i);
    for (int j = 0; j < 3; j++)
      rot_kp.rot[i * 3 + j] = R(i, j);
  }
  // Map<M3D>(rot_kp.rot, 3,3) = R;
  return rot_kp;
}

}


#endif // SYSTEM_STATE_HPP