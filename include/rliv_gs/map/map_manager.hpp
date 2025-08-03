/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef MAP_MANAGER_HPP
#define MAP_MANAGER_HPP

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include "rliv_gs/common/config.hpp"
#include "rliv_gs/sensor/sensor_measurement.hpp"
#include "voxel_map.hpp"
#include "visual_map.hpp"
#include "rliv_gs/frontend/system_state.hpp"


namespace map {

class MapManager {

public:
    MapManager();
    ~MapManager();

    void buildMap(const frontend::SystemState& init_system_state);
    void updateMap(const frontend::SystemState& system_state);

    void loadVoxelMapConfig(const common::Config& config);

    void calculateLidarPointCovariance(Eigen::Vector3d& point, Eigen::Matrix3d& cov);

    void voxelGridFilter(const pcl::PointCloud<PointXYZIRT>::Ptr input_cloud);


    // For relocation
    // void loadVoxelMap();


    std::list<std::pair<VoxelLocation, VoxelOctoTree*>>& getVoxelMapCache();
    std::unordered_map<VoxelLocation, std::list<std::pair<VoxelLocation, VoxelOctoTree*>>::iterator>& getVoxelMap();
    VisualMap* getVisualMapPtr();
    pcl::PointCloud<PointXYZIRT>::Ptr getDownsampledLidarCloud();
    pcl::PointCloud<PointXYZIRT>::Ptr getDownsampledGlobalCloud();
    std::vector<Eigen::Matrix3d>& getSkewSymmMatricesList();
    std::vector<Eigen::Matrix3d>& getLidarPointCovariancesList();
    std::vector<PointWithCovarience>& getPointsWithCovariancesList();
    std::vector<PointToPlane>& getPointToPlaneList();
    int getMaximumIteration();
    VoxelMapConfig& getVoxelMapConfig();


private:

    VoxelMapConfig voxel_map_config_;
    std::list<std::pair<VoxelLocation, VoxelOctoTree*>> voxel_map_cache_;
    std::unordered_map<VoxelLocation, typename std::list<std::pair<VoxelLocation, VoxelOctoTree*>>::iterator> voxel_map_;
    VisualMap* visual_map_ptr_;
    pcl::PointCloud<PointXYZIRT>::Ptr downsampled_lidar_cloud_; // downsampled lidar point cloud
    pcl::PointCloud<PointXYZIRT>::Ptr downsampled_global_cloud_; // downsampled global point cloud

    std::vector<Eigen::Matrix3d> skew_symm_matrices_list_;
    std::vector<Eigen::Matrix3d> lidar_point_covariances_list_;
    std::vector<PointWithCovarience> points_with_covariances_list_;
    std::vector<PointToPlane> point_to_plane_list_;
    
    Eigen::Matrix3d lidar_imu_rotation_;
    Eigen::Vector3d lidar_imu_translation_;

    int maximum_iteration;

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

};
}




#endif // MAP_MANAGER_HPP