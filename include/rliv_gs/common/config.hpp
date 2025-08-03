/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>


#include "parameter.hpp"
#include "utils.hpp"

namespace common {

class Config {
    
public:
    //Constructor: get config from ros parameter server
    Config(ros::NodeHandle& nh);

    // Destructor
    ~Config();

    // Validate the configuration
    bool validate() const;

    // Getters

    const LidarParameter& getLidarParam() const;
    const CameraParameter& getCameraParam() const;
    const YAML::Node& getCameraBaseConfig() const;
    const ImuParameter& getImuParam() const;
    const GnssParameter& getGnssParam() const;
    const FrontendParameter& getFrontendParam() const;

    // Set Frontend mode
    bool setFrontendMode();
    

private:

    //Lidar parameters
    LidarParameter lidar_param_;
    //Camera parameters
    YAML::Node camera_base_config_;
    CameraParameter camera_param_;
    //Imu parameters
    ImuParameter imu_param_;
    //Gnss parameters
    GnssParameter gnss_param_;
    //Frontend parameters
    FrontendParameter frontend_param_;
};

}
#endif // CONFIG_HPP