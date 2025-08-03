/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef SENSOR_PUBLISHER_HPP
#define SENSOR_PUBLISHER_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>

#include "sensor_measurement.hpp"
#include "rliv_gs/common/utils.hpp"
#include "rliv_gs/map/visual_map.hpp"

namespace sensor {

class SensorPublisher {
public:
    SensorPublisher(ros::NodeHandle &nh);
    ~SensorPublisher();

    void publishGlobalLidar(const pcl::PointCloud<PointXYZIRT>::Ptr &cloud);
    void publishGlobalPclRGB(const pcl::PointCloud<PointXYZIRT>::Ptr &cloud,
                            map::VisualMap* visual_map_ptr, 
                            cv::Mat &image);
    void publishImage(const cv::Mat &image);

private:
    ros::Publisher pub_global_lidar_;
    ros::Publisher pub_global_pcl_rgb_;
    image_transport::Publisher pub_image_;

};

} // namespace sensor

#endif // SENSOR_PUBLISHER_HPP