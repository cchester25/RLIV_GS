/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef RLIVGS_SYSTEM_HPP
#define RLIVGS_SYSTEM_HPP

#include <pcl/io/pcd_io.h>	

#include "rliv_gs/common/config.hpp"
#include "rliv_gs/sensor/sensor_subscriber.hpp"
#include "rliv_gs/sensor/sensor_evaluator.hpp"
#include "rliv_gs/sensor/sensor_publisher.hpp"
#include "rliv_gs/frontend/esikf.hpp"

namespace rlivgs {
class RlivgsSystem {
public:
    // Constructor
    RlivgsSystem(ros::NodeHandle& nh);

    // Destructor
    ~RlivgsSystem();

    // Run the main loop
    void run();

    bool isCallbackActive() const;

    void handleLio();

    void handleVio();

    void savePCD(const std::string& filename, const pcl::PointCloud<PointXYZIRT>::Ptr cloud);



private:
    // Configuration object to access parameters
    const common::Config config_;
    // Sensor subscriber
    sensor::SensorSubscriber sensor_subscriber_;
    // Sensor evaluator
    sensor::SensorEvaluator sensor_evaluator_;
    // Sensor publisher
    sensor::SensorPublisher sensor_publisher_;
    // ESKF frontend
    frontend::Esikf esikf_;

    pcl::PointCloud<PointXYZIRT>::Ptr points_lio_;
    pcl::PointCloud<PointXYZIRT>::Ptr all_points_;
    cv::Mat img_rgb_;
    cv::Mat img_plot_;


    
    bool isDataGathered_ = false; // Flag to check if data is gathered
    bool is_imu_init_ = true; // Flag to check if imu initialization is needed
    bool first_lio_ = true; // Flag to check if it's the first lio
    bool first_frame_ = true; // Flag to check if it's the first frame

    int test=0;
};
}
#endif // RLIVGS_SYSTEM_HPP