/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef VOXEL_MAP_HPP
#define VOXEL_MAP_HPP

#include <unistd.h>
#include <unordered_map>
#include <Eigen/Eigen>


#define VOXELMAP_HASH_P 116101
#define VOXELMAP_MAX_N 10000000000
static int voxel_plane_id = 0;
class VoxelLocation {
    public:
    int64_t x, y, z;

    VoxelLocation(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

    bool operator==(const VoxelLocation &other) const { return (x == other.x && y == other.y && z == other.z); }
};

// Hash value
namespace std {
template <> struct hash<VoxelLocation>
{
  int64_t operator()(const VoxelLocation& s) const
  {
    using std::hash;
    using std::size_t;
    return ((((s.z) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.y)) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.x);
  }
};
} // namespace std

namespace map {

struct VoxelMapConfig {
  int octree_max_depth;
  double node_max_points;
  double node_min_points;
  double frame_voxel_size;
  double map_voxel_size;

  double plane_threshold;
  double ranging_noise;
  double bearing_noise;
  double sigma_num;
  int capacity;

};

struct PointWithCovarience {
  Eigen::Vector3d lidar_point;     // point in the lidar body frame
  Eigen::Vector3d global_point;     // point in the world frame
  Eigen::Matrix3d lidar_point_covariance;
  Eigen::Matrix3d global_point_covariance;
  Eigen::Vector3d normal;
  PointWithCovarience()
  {
    lidar_point_covariance = Eigen::Matrix3d::Zero();
    global_point_covariance = Eigen::Matrix3d::Zero();
    lidar_point = Eigen::Vector3d::Zero();
    global_point = Eigen::Vector3d::Zero();
    normal = Eigen::Vector3d::Zero();
  }
};

struct PointToPlane {
  Eigen::Vector3d lidar_point;
  Eigen::Vector3d global_point;
  Eigen::Vector3d plane_normal;
  Eigen::Vector3d plane_center;
  Eigen::Matrix<double, 6, 6> plane_covariance; // Contains the covariance of the normal vector and the center point
  Eigen::Matrix3d lidar_point_covariance;

  float distance_to_plane;
  double plane_intercept = 0;

  int layer;
};

struct VoxelPlane {
  Eigen::Vector3d plane_center;
  Eigen::Vector3d plane_normal;
  Eigen::Vector3d tangent_x;
  Eigen::Vector3d tangent_y;
  Eigen::Matrix3d points_covariance;
  Eigen::Matrix<double, 6, 6> plane_covariance;
  float radius = 0;
  float min_eigen_value = 1;
  float mid_eigen_value = 1;
  float max_eigen_value = 1;
  float plane_intercept = 0;
  int points_size = 0;
  bool is_plane = false;
  bool is_init = false;
  int plane_id = 0;
  bool is_update = false;
  VoxelPlane() {
    plane_covariance = Eigen::Matrix<double, 6, 6>::Zero();
    points_covariance = Eigen::Matrix3d::Zero();
    plane_center = Eigen::Vector3d::Zero();
    plane_normal = Eigen::Vector3d::Zero();
  }
};


class VoxelOctoTree {
public:
  VoxelOctoTree(int octree_max_depth, int layer, int node_min_points, int node_max_points, float planer_threshold);
  ~VoxelOctoTree();
  void initPlane(const std::vector<PointWithCovarience>& points, VoxelPlane* plane);
  void initOctoTree();
  void cutOctoTree();
  void updateOctoTree(const PointWithCovarience& pc);
  VoxelOctoTree* findCorrespondPoint(Eigen::Vector3d pw);
  VoxelOctoTree* insertPoint(const PointWithCovarience& pc);

  void setVoxelCenter(double voxel_center[3]);
  void setQuaterLength(float quater_length);
  void addTempPoints(PointWithCovarience temp_point);
  void resetTempPoints();
  void addNewPoints();
  void setNewPoints(int num_point);
  void setOctoState(int octo_state);
  void setUpdateEnable(bool update_enable);
  void setInitOcto(bool init_octo);

  const std::vector<PointWithCovarience>& getTempPoints() const;
  VoxelPlane* getPlanePtr() const;
  VoxelOctoTree* getLeaves(int index) const;
  const double* getVoxelCenter() const;
  float getQuaterLength() const;
  int getNodeMinPoints() const;
  int getNodeMaxPoints() const;


private:

  VoxelPlane* plane_ptr_;
  int octree_max_depth_;
  int layer_;
  int node_min_points_;
  int node_max_points_;
  float plane_threshold_;
  std::vector<PointWithCovarience> temp_points_;
  int octo_state_; // 0 is end of tree, 1 is not
  int new_points_;
  int update_size_threshold_;
  bool init_octo_;
  bool update_enable_;  
  VoxelOctoTree* leaves_[8];
  double voxel_center_[3]; // x, y, z
  float quater_length_;







};

} // namespace map
 



#endif // VOXEL_MAP_HPP