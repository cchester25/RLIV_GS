/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "rliv_gs/sensor/sensor_publisher.hpp"

namespace sensor {

SensorPublisher::SensorPublisher(ros::NodeHandle &nh) {
    // Constructor
    pub_global_lidar_ = nh.advertise<sensor_msgs::PointCloud2>("/global_lidar", 1);
    pub_global_pcl_rgb_ = nh.advertise<sensor_msgs::PointCloud2>("/global_pcl_rgb", 1);
    image_transport::ImageTransport image_transport(nh);
    pub_image_ = image_transport.advertise("/image_plot", 1);
    common::Utils::printColored(" SensorPublisher initialized successfully! ", common::Color::kGreen, common::Style::kBold);
}

SensorPublisher::~SensorPublisher() {
    // Destructor
}

void SensorPublisher::publishGlobalLidar(const pcl::PointCloud<PointXYZIRT>::Ptr &cloud) {
    // Transform custom lidar pointcloud to ROS pointcloud
    sensor_msgs::PointCloud2 msg;
    uint num_points = cloud->points.size();
    common::Utils::printColored(" Publishing global lidar with " + std::to_string(num_points) + " points ", common::Color::kYellow, common::Style::kBold);
    
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = "camera_init";
    msg.header.stamp = ros::Time::now();
    pub_global_lidar_.publish(msg);
}

void SensorPublisher::publishImage(const cv::Mat &image) {
    // Transform cv::Mat to ROS image
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    msg->header.frame_id = "camera_init";
    msg->header.stamp = ros::Time::now();
    pub_image_.publish(msg);
}

void SensorPublisher::publishGlobalPclRGB(const pcl::PointCloud<PointXYZIRT>::Ptr &global_cloud,
                                            map::VisualMap* visual_map_ptr, 
                                            cv::Mat &image) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
    size_t size = global_cloud->points.size();
    global_cloud_rgb->reserve(size);
    for (size_t i = 0; i < size; i++) {
        pcl::PointXYZRGB point_rgb;
        point_rgb.x = global_cloud->points[i].x;
        point_rgb.y = global_cloud->points[i].y;
        point_rgb.z = global_cloud->points[i].z;

        Eigen::Vector3d point_global(global_cloud->points[i].x, global_cloud->points[i].y,
                            global_cloud->points[i].z);
        Eigen::Vector3d point_camera(visual_map_ptr->transGpointToCamera(point_global));
        if (point_camera[2] < 0) continue;
        Eigen::Vector2d point_image(visual_map_ptr->transGpointToPixel(point_global));
        if (visual_map_ptr->checkPixelInFrame(point_image, 3)) {
           Eigen::Vector3f  pixel = visual_map_ptr->getInterpolatedPixel(image, point_image);
            point_rgb.r = pixel[2];
            point_rgb.g = pixel[1];
            point_rgb.b = pixel[0];
            if (point_camera.norm() > 0.01) {
                global_cloud_rgb->push_back(point_rgb);
            }
                
        }
    }
    sensor_msgs::PointCloud2 cloud_rgb_msg;
    pcl::toROSMsg(*global_cloud_rgb, cloud_rgb_msg);
    cloud_rgb_msg.header.stamp = ros::Time::now();  //.fromSec(last_timestamp_lidar);
    cloud_rgb_msg.header.frame_id = "camera_init";
    pub_global_pcl_rgb_.publish(cloud_rgb_msg);
} 
}// namespace sensor