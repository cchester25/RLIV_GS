/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "rliv_gs/map/map_manager.hpp"

namespace map {

MapManager::MapManager(){
    // TODO: implement
    downsampled_lidar_cloud_.reset(new pcl::PointCloud<PointXYZIRT>());
    downsampled_global_cloud_.reset(new pcl::PointCloud<PointXYZIRT>());
}

void MapManager::buildMap(const frontend::SystemState& init_system_state) {
    double map_voxel_size = voxel_map_config_.map_voxel_size;
    double plane_threshold = voxel_map_config_.plane_threshold;
    int octree_max_depth = voxel_map_config_.octree_max_depth;
    int node_max_points = voxel_map_config_.node_max_points;
    int node_min_points = voxel_map_config_.node_min_points;
 
    std::vector<PointWithCovarience> input_points;
    auto state_rotation = init_system_state.getSystemRotation();
    auto state_covariance = init_system_state.getStateCovariance();
    for (size_t i = 0; i < downsampled_global_cloud_->size(); i++) {
        PointWithCovarience point;
        point.global_point << downsampled_global_cloud_->points[i].x, 
                                downsampled_global_cloud_->points[i].y, 
                                downsampled_global_cloud_->points[i].z;
        Eigen::Vector3d lidar_point(downsampled_lidar_cloud_->points[i].x, 
                                    downsampled_lidar_cloud_->points[i].y, 
                                    downsampled_lidar_cloud_->points[i].z);
        Eigen::Matrix3d point_cov;                            
        calculateLidarPointCovariance(lidar_point, point_cov);
        Eigen::Matrix3d lidar_point_crossmat;
        lidar_point_crossmat << common::Utils::getAntiSymmetricMatrix(lidar_point);

        point_cov = (state_rotation * lidar_imu_rotation_) * point_cov * (state_rotation * lidar_imu_rotation_).transpose() + 
                          (-lidar_point_crossmat) * state_covariance.block<3, 3>(0, 0) * (-lidar_point_crossmat).transpose() + 
                          state_covariance.block<3, 3>(3, 3);
        point.global_point_covariance = point_cov;

        input_points.push_back(point);
    }

    int plsize = input_points.size();

    for (int i = 0; i < plsize; i++) {
        const PointWithCovarience point = input_points[i];
        float local_xyz[3];
        for (int j = 0; j < 3; j++) {
            local_xyz[j] = point.global_point[j] / map_voxel_size;
            if (local_xyz[j] < 0) {
                local_xyz[j] -= 1.0;
            }
        }

        VoxelLocation position((int64_t)local_xyz[0], (int64_t)local_xyz[1], (int64_t)local_xyz[2]);
        auto iter = voxel_map_.find(position);
        if (iter != voxel_map_.end()) {
            voxel_map_[position]->second->addTempPoints(point);
            voxel_map_[position]->second->addNewPoints();

            voxel_map_cache_.splice(voxel_map_cache_.begin(), voxel_map_cache_, iter->second);
            iter->second = voxel_map_cache_.begin();
        } else {
            VoxelOctoTree* octree = new VoxelOctoTree(octree_max_depth, 0, node_min_points, node_max_points, plane_threshold);
            double voxel_center[3] = {(0.5 + position.x) * map_voxel_size, 
                                        (0.5 + position.y) * map_voxel_size, 
                                        (0.5 + position.z) * map_voxel_size};
            octree->setVoxelCenter(voxel_center);
            float quater_length = map_voxel_size / 4.0;
            octree->setQuaterLength(quater_length);
            octree->addTempPoints(point);
            octree->addNewPoints();
            
            voxel_map_cache_.emplace_front(position, octree);
            voxel_map_.insert({position, voxel_map_cache_.begin()});
        }
    }

    common::Utils::printColored(" Map size: " + std::to_string(voxel_map_cache_.size()), common::Color::kGreen, common::Style::kBold);
    
    if (voxel_map_cache_.size() >= static_cast<size_t>(voxel_map_config_.capacity)) {
        while (voxel_map_cache_.size() >= static_cast<size_t>(voxel_map_config_.capacity)) {
            voxel_map_.erase(voxel_map_cache_.back().first);
            delete voxel_map_cache_.back().second;
            voxel_map_cache_.pop_back();
        }
    }
    
    for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ++iter) {
        iter->second->second->initOctoTree();
    }

}

void MapManager::updateMap(const frontend::SystemState& system_state) {


    double map_voxel_size = voxel_map_config_.map_voxel_size;
    double plane_threshold = voxel_map_config_.plane_threshold;
    int octree_max_depth = voxel_map_config_.octree_max_depth;
    int node_max_points = voxel_map_config_.node_max_points;
    int node_min_points = voxel_map_config_.node_min_points;

    for (size_t i = 0; i < downsampled_global_cloud_->points.size(); i++) {
        points_with_covariances_list_[i].global_point << downsampled_global_cloud_->points[i].x,
                                    downsampled_global_cloud_->points[i].y,
                                    downsampled_global_cloud_->points[i].z;
        Eigen::Matrix3d point_crossmat = skew_symm_matrices_list_[i];
        Eigen::Matrix3d covariances = lidar_point_covariances_list_[i];
        covariances = system_state.getSystemRotation() * lidar_imu_rotation_ * 
                            covariances * 
                        (system_state.getSystemRotation() * lidar_imu_rotation_).transpose() + 
                        (-point_crossmat) * system_state.getStateCovariance().block<3, 3>(0, 0) * (-point_crossmat).transpose() + 
                        system_state.getStateCovariance().block<3, 3>(3, 3);
        points_with_covariances_list_[i].global_point_covariance = covariances;
    }

    int plsize = points_with_covariances_list_.size();
    for (int i = 0; i < plsize; i++) {

        const PointWithCovarience point = points_with_covariances_list_[i];
        float local_xyz[3];
        for (int j = 0; j < 3; j++) {
            local_xyz[j] = point.global_point[j] / map_voxel_size;
            if (local_xyz[j] < 0) {
                local_xyz[j] -= 1.0;
            }
        }
        VoxelLocation position((int64_t)local_xyz[0], (int64_t)local_xyz[1], (int64_t)local_xyz[2]);
        auto iter = voxel_map_.find(position);

        if (iter != voxel_map_.end()) {
            iter->second->second->updateOctoTree(point);
            voxel_map_cache_.splice(voxel_map_cache_.begin(), voxel_map_cache_, iter->second);
            iter->second = voxel_map_cache_.begin();
        } else {
            VoxelOctoTree* octree = new VoxelOctoTree(octree_max_depth, 0, node_min_points, node_max_points, plane_threshold);
            double voxel_center[3] = {(0.5 + position.x) * map_voxel_size,
                                    (0.5 + position.y) * map_voxel_size,
                                    (0.5 + position.z) * map_voxel_size};
            octree->setVoxelCenter(voxel_center);
            float quater_length = map_voxel_size / 4.0;
            octree->setQuaterLength(quater_length);
            octree->addTempPoints(point);
            octree->addNewPoints();
            octree->updateOctoTree(point);

            voxel_map_cache_.emplace_front(position, octree);
            voxel_map_.insert({position, voxel_map_cache_.begin()});
        }
    }

    if (voxel_map_cache_.size() >= static_cast<size_t>(voxel_map_config_.capacity)) {
        while (voxel_map_cache_.size() >= static_cast<size_t>(voxel_map_config_.capacity)) {
            voxel_map_.erase(voxel_map_cache_.back().first);
            delete voxel_map_cache_.back().second;
            voxel_map_cache_.pop_back();
        }
    }
    
}

void MapManager::loadVoxelMapConfig(const common::Config& config) {
    voxel_map_config_.octree_max_depth = config.getFrontendParam().lio_param.octree_max_depth;
    voxel_map_config_.node_max_points = config.getFrontendParam().lio_param.node_max_points;
    voxel_map_config_.node_min_points = config.getFrontendParam().lio_param.node_min_points;
    voxel_map_config_.frame_voxel_size = config.getFrontendParam().lio_param.frame_voxel_size;
    voxel_map_config_.map_voxel_size = config.getFrontendParam().lio_param.map_voxel_size;
    voxel_map_config_.plane_threshold = config.getFrontendParam().lio_param.plane_threshold;
    voxel_map_config_.ranging_noise = config.getLidarParam().ranging_noise;
    voxel_map_config_.bearing_noise = config.getLidarParam().bearing_noise;
    voxel_map_config_.sigma_num = config.getFrontendParam().lio_param.sigma_num;
    voxel_map_config_.capacity = config.getFrontendParam().lio_param.capacity;

    std::vector<double> lidar_imu_rotation = config.getLidarParam().lidar_imu_rotation;
    lidar_imu_rotation_ << lidar_imu_rotation[0], lidar_imu_rotation[1], lidar_imu_rotation[2], 
                            lidar_imu_rotation[3], lidar_imu_rotation[4], lidar_imu_rotation[5],
                            lidar_imu_rotation[6], lidar_imu_rotation[7], lidar_imu_rotation[8];
    std::vector<double> lidar_imu_translation = config.getLidarParam().lidar_imu_translation;
    lidar_imu_translation_ << lidar_imu_translation[0], lidar_imu_translation[1], lidar_imu_translation[2];

    maximum_iteration = config.getFrontendParam().max_iterations;
    sor.setLeafSize(voxel_map_config_.frame_voxel_size, voxel_map_config_.frame_voxel_size, voxel_map_config_.frame_voxel_size);
    visual_map_ptr_ = new VisualMap(config);
}

void MapManager::calculateLidarPointCovariance(Eigen::Vector3d& point, Eigen::Matrix3d& cov) {
  if (point[2] == 0) point[2] = 0.0001;
  float range = std::sqrt(point[0] * point[0] + point[1] * point[1] + point[2] * point[2]);
  float range_var = (float)voxel_map_config_.ranging_noise * (float)voxel_map_config_.ranging_noise;
  Eigen::Matrix2d direction_var;
  direction_var << std::pow(std::sin(DEG2RAD((float)voxel_map_config_.bearing_noise)), 2), 0, 0, std::pow(std::sin(DEG2RAD((float)voxel_map_config_.bearing_noise)), 2);
  Eigen::Vector3d direction(point);
  direction.normalize();
  Eigen::Matrix3d direction_hat;
  direction_hat << 0, -direction(2), direction(1), direction(2), 0, -direction(0), -direction(1), direction(0), 0;
  Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
  base_vector1.normalize();
  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
  base_vector2.normalize();
  Eigen::Matrix<double, 3, 2> N;
  N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);
  Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
  cov = direction * range_var * direction.transpose() + A * direction_var * A.transpose();
}

void MapManager::voxelGridFilter(const pcl::PointCloud<PointXYZIRT>::Ptr input_cloud) {
    
    /*
    pcl::PCLPointCloud2::Ptr common_cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    // transform to pcl::PCLPointCloud2
    pcl::toPCLPointCloud2(*input_cloud, *common_cloud);

    sor.setInputCloud(common_cloud);

    sor.filter(*cloud_filtered);

    // transform to pcl::PointCloud
    pcl::fromPCLPointCloud2(*cloud_filtered, *downsampled_lidar_cloud_);
    */
    *downsampled_lidar_cloud_ = *input_cloud;


    
}


std::list<std::pair<VoxelLocation, VoxelOctoTree*>>& MapManager::getVoxelMapCache() {
    return voxel_map_cache_;
}

std::unordered_map<VoxelLocation, std::list<std::pair<VoxelLocation, VoxelOctoTree*>>::iterator>& MapManager::getVoxelMap() {
    return voxel_map_;
}

VisualMap* MapManager::getVisualMapPtr() {
    return visual_map_ptr_;
}

pcl::PointCloud<PointXYZIRT>::Ptr MapManager::getDownsampledLidarCloud() {
    return downsampled_lidar_cloud_;
}

pcl::PointCloud<PointXYZIRT>::Ptr MapManager::getDownsampledGlobalCloud() {
    return downsampled_global_cloud_;
}


std::vector<Eigen::Matrix3d>& MapManager::getSkewSymmMatricesList() {
    return skew_symm_matrices_list_;
}

std::vector<Eigen::Matrix3d>& MapManager::getLidarPointCovariancesList() {
    return lidar_point_covariances_list_;
}

std::vector<PointWithCovarience>& MapManager::getPointsWithCovariancesList() {
    return points_with_covariances_list_;
}

std::vector<PointToPlane>& MapManager::getPointToPlaneList() {
    return point_to_plane_list_;
}

int MapManager::getMaximumIteration() {
    return maximum_iteration;
}

VoxelMapConfig& MapManager::getVoxelMapConfig() {
    return voxel_map_config_;
}


MapManager::~MapManager() {
    delete visual_map_ptr_;
}
}