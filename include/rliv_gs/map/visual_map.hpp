/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef VISUAL_MAP_HPP
#define VISUAL_MAP_HPP


#include <vikit/pinhole_camera.h>
#include <vikit/vision.h>


#include "rliv_gs/common/config.hpp"
#include "voxel_map.hpp"
#include "rliv_gs/map/visual_elements/patch.hpp"
#include "rliv_gs/frontend/system_state.hpp"

#define SIZE_LARGE (500)
#define SIZE_SMALL (100)

namespace map {

struct SubSparseMap {
  std::vector<float> propa_errors;
  std::vector<float> errors;
  std::vector<std::vector<float>> warp_patch;
  std::vector<int> search_levels;
  std::vector<visual::Point *> visual_points;
  std::vector<double> inv_expo_list;
  std::vector<PointWithCovarience> add_from_voxel_map;

  SubSparseMap() {
    propa_errors.reserve(SIZE_LARGE);
    errors.reserve(SIZE_LARGE);
    warp_patch.reserve(SIZE_LARGE);
    search_levels.reserve(SIZE_LARGE);
    visual_points.reserve(SIZE_LARGE);
    inv_expo_list.reserve(SIZE_LARGE);
    add_from_voxel_map.reserve(SIZE_SMALL);
  };

  void reset() {
    propa_errors.clear();
    errors.clear();
    warp_patch.clear();
    search_levels.clear();
    visual_points.clear();
    inv_expo_list.clear();
    add_from_voxel_map.clear();
  }
};

struct Warp {
  Warp(int level, Eigen::Matrix2d warp_matrix)
      : search_level(level), A_cur_ref(warp_matrix) {}
  int search_level;
  Eigen::Matrix2d A_cur_ref;
};

struct VisualPoints {
  std::vector<visual::Point *> visual_points;
  int count;
  VisualPoints(int num) : count(num) {}
  ~VisualPoints() {
    for (visual::Point *vp : visual_points) {
      if (vp != nullptr) {
        delete vp;
        vp = nullptr;
      }
    }
  }
};

class VisualMap {
public:
  VisualMap(const common::Config& config);
  ~VisualMap();

  void retrieveFromVisualSparseMap(cv::Mat img, 
                                    std::vector<PointWithCovarience>& pc_list, 
                                    const std::unordered_map<VoxelLocation, 
                                    std::list<std::pair<VoxelLocation, 
                                    VoxelOctoTree*>>::iterator>& voxel_map,
                                    double system_inv_expo_time);
  void getWarpMatrixAffineHomography(const vk::AbstractCamera &cam,
                                      const Eigen::Vector2d &px_ref, 
                                      const Eigen::Vector3d &xyz_ref,
                                      const Eigen::Vector3d &normal_ref,
                                      const Sophus::SE3d &T_cur_ref, 
                                      const int level_ref,
                                      Eigen::Matrix2d &A_cur_ref);

  int getBestSearchLevel(const Eigen::Matrix2d &A_cur_ref, const int max_level);

  void warpAffine(const Eigen::Matrix2d &A_cur_ref, const cv::Mat &img_ref,
                const Eigen::Vector2d &px_ref, const int level_ref,
                const int search_level, const int pyramid_level,
                const int halfpatch_size, float *patch);
  
  void getImagePatch(cv::Mat img, Eigen::Vector2d pc, float *patch_tmp, int level);

  void updateFrameState(const frontend::SystemState& state);

  void setImuToLidarExtrinsic(const std::vector<double>& R, const std::vector<double>& P);

  void setLidarToCameraExtrinsic(const std::vector<double>& R, const std::vector<double>& P);

  void resetGrid();

  void generateVisualMapPoints(cv::Mat img, 
                                std::vector<PointWithCovarience> &pc_list,
                                double system_inv_expo_time);
  
  void insertPointIntoVisualMap(visual::Point *pt_new);

  void plotTrackedPoints(cv::Mat &img_cp);

  void updateVisualMap(cv::Mat img, double system_inv_expo_time);

  void updateReferencePatch(const std::unordered_map<VoxelLocation,
       typename std::list<std::pair<VoxelLocation, VoxelOctoTree *>>::iterator> &voxel_map);

  Eigen::Vector3f getInterpolatedPixel(cv::Mat img, Eigen::Vector2d pc);

  Eigen::Vector3d transGpointToCamera(const Eigen::Vector3d &global_point);

  Eigen::Vector2d transGpointToPixel(const Eigen::Vector3d &global_point);

  bool checkPixelInFrame(const Eigen::Vector2d &pixel, int level);

  SubSparseMap* getVisualSubmap();
  vk::AbstractCamera* getAbstractCamera();
  int getTotalPoints();
  int getPixelsPerPatch();
  int getPatchSize();
  int getHalfPatchSize();

  void resetNewFrame(const cv::Mat &img);



  Eigen::Matrix3d& getRci();
  Eigen::Vector3d& getPci();
  Eigen::Matrix3d& getRcw();
  Eigen::Vector3d& getPcw();
  Eigen::Matrix3d& getJdphidR();
  Eigen::Matrix3d& getJdpdt();
  Eigen::Matrix3d& getJdpdR();

private:
  SubSparseMap* visual_submap_;
  vk::AbstractCamera *cam_;
  visual::FramePtr new_frame_;
  
  int grid_size_;
  int patch_size_, half_patch_size_;
  int grids_along_width_, grids_along_height_;
  int total_grids_;
  int pixels_per_patch_;
  int patch_warp_size_;
  int patch_pyrimid_level_;
  int border_;
  double map_voxel_size_;
  int capacity_;
  int total_points_;

  double outlier_threshold_;

  Eigen::Matrix3d Rli_, Rci_, Rcl_, Rcw_, Jdphi_dR_, Jdp_dt_, Jdp_dR_;
  Eigen::Vector3d Pli_, Pci_, Pcl_, Pcw_;

  std::vector<int> grid_type_;
  std::vector<int> update_flag_;
  std::vector<float> map_point_dist_;
  std::vector<float> point_score_;
  std::vector<float> patch_pixels_;

  std::vector<visual::Point *> retrieve_visual_points_;
  std::vector<PointWithCovarience> append_voxel_points_;

  std::unordered_map<VoxelLocation, int> sub_map_flag_;
  std::unordered_map<VoxelLocation, typename std::list<std::pair<VoxelLocation, VisualPoints*>>::iterator> visualpoint_map_;
  std::list<std::pair<VoxelLocation, VisualPoints*>> visualpoint_map_cache_; // visual point data
  enum CellType { TYPE_MAP = 1, TYPE_POINTCLOUD, TYPE_UNKNOWN };

};

} // namespace map

#endif // VISUAL_MAP_HPP