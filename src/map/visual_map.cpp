/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "rliv_gs/map/visual_map.hpp"
#include <yaml_loader.h>
namespace map {

VisualMap::VisualMap(const common::Config& config) {
    visual_submap_ = new SubSparseMap;

    if (!vk::camera_loader::loadFromYaml(config.getCameraBaseConfig(), cam_)) {
        throw std::runtime_error("Camera model not correctly specified.");
    }

    patch_size_ = config.getFrontendParam().vio_param.patch_size;
    grid_size_ = config.getFrontendParam().vio_param.grid_size;

    if (grid_size_ > 10) {
        grids_along_width_ = std::ceil(static_cast<double>(cam_->width() / grid_size_));
        grids_along_height_ = std::ceil(static_cast<double>(cam_->height() / grid_size_));
    } else {
        grid_size_ = static_cast<int>(cam_->height() / 17);
        grids_along_height_ = std::ceil(static_cast<double>(cam_->height() / grid_size_));
        grids_along_width_ = std::ceil(static_cast<double>(cam_->width() / grid_size_));
    }
    total_grids_ = grids_along_width_ * grids_along_height_;

    grid_type_.resize(total_grids_);
    update_flag_.resize(total_grids_);
    map_point_dist_.resize(total_grids_);
    point_score_.resize(total_grids_);

    pixels_per_patch_ = patch_size_ * patch_size_;
    half_patch_size_ = patch_size_ / 2;
    patch_pixels_.resize(pixels_per_patch_);
    patch_pyrimid_level_ = config.getFrontendParam().vio_param.pyrimid_level;
    patch_warp_size_ = pixels_per_patch_ * patch_pyrimid_level_;
    border_ = (half_patch_size_ + 1) * (1 << patch_pyrimid_level_);

    map_voxel_size_ = config.getFrontendParam().lio_param.map_voxel_size;

    outlier_threshold_ = config.getFrontendParam().vio_param.outlier_threshold;
    capacity_ = config.getFrontendParam().lio_param.capacity;
    retrieve_visual_points_.reserve(total_grids_);
    append_voxel_points_.reserve(total_grids_);

    setImuToLidarExtrinsic(config.getLidarParam().lidar_imu_rotation, config.getLidarParam().lidar_imu_translation);
    setLidarToCameraExtrinsic(config.getLidarParam().lidar_camera_rotation, config.getLidarParam().lidar_camera_translation);
    Rci_ = Rcl_ * Rli_;
    Pci_ = Rcl_ * Pli_ + Pcl_;
    Eigen::Vector3d Pic;
    Eigen::Matrix3d tmp;
    Jdphi_dR_ = Rci_;
    Pic = -Rci_.transpose() * Pci_;
    tmp = common::Utils::getAntiSymmetricMatrix(Pic);
    Jdp_dR_ = -Rci_ * tmp;


    sub_map_flag_.clear();
}

void VisualMap::retrieveFromVisualSparseMap(cv::Mat img, 
        std::vector<PointWithCovarience>& pc_list, 
        const std::unordered_map<VoxelLocation, std::list<std::pair<VoxelLocation, VoxelOctoTree*>>::iterator>& voxel_map,
        double system_inv_expo_time) {
    
    if (visualpoint_map_.size() <= 0) return;
    visual_submap_->reset();

    sub_map_flag_.clear();

    cv::Mat depth_img = cv::Mat::zeros(cam_->height(), cam_->width(), CV_32FC1);
    float *it = (float *)depth_img.data;

    int local_xyz[3];
    for (size_t i = 0; i < pc_list.size(); i++) {
        Eigen::Vector3d global_point = pc_list[i].global_point;
        for (int j = 0; j < 3; j++) {
            local_xyz[j] = std::floor(global_point[j] / map_voxel_size_);
            if (local_xyz[j] < 0) {
                local_xyz[j] -= 1.0;
            }
        }
        VoxelLocation position(local_xyz[0], local_xyz[1], local_xyz[2]);

        auto iter = sub_map_flag_.find(position);
        if (iter == sub_map_flag_.end()) {
            sub_map_flag_[position] = 0;
        } else {
            iter->second = 0;
        }

        Eigen::Vector3d camera_point = new_frame_->transWorld2Cam(global_point);
        if (camera_point[2] > 0) {
            Eigen::Vector2d px;
            px = new_frame_->getAbstractCamera()->world2cam(camera_point); // projection to image plane
            // 记录深度
            if (new_frame_->getAbstractCamera()->isInFrame(px.cast<int>(), border_)) {
                float depth = camera_point[2];
                int col = int(px[0]);
                int row = int(px[1]);
                it[cam_->width() * row + col] = depth;
            }
        }
    }

    std::vector<VoxelLocation> DeleteKeyList;
    for (auto &iter : sub_map_flag_) {
        VoxelLocation position = iter.first;
        auto corre_visualpoint = visualpoint_map_.find(position);
        if (corre_visualpoint != visualpoint_map_.end()) {
            bool voxel_in_fov = false;
            std::vector<visual::Point *> &visual_points =
                corre_visualpoint->second->second->visual_points;
            int voxel_num = visual_points.size();

            for (int i = 0; i < voxel_num; i++) {
                visual::Point *pt = visual_points[i];
                if (pt == nullptr) continue;

                if (pt->getObservedPatchs().size() == 0) continue;

                Eigen::Vector3d norm_vec(new_frame_->getPoseFrameWorld().rotationMatrix() * pt->getNormal());
                Eigen::Vector3d dir(new_frame_->getPoseFrameWorld() * pt->getPosition());
                if (dir[2] < 0) continue;

                Eigen::Vector2d pc(new_frame_->transWorld2Pixel(pt->getPosition()));
                if (new_frame_->getAbstractCamera()->isInFrame(pc.cast<int>(), border_)) {
                    voxel_in_fov = true;
                    int index = static_cast<int>(pc[1] / grid_size_) * grids_along_width_ +
                                static_cast<int>(pc[0] / grid_size_);
                    grid_type_[index] = TYPE_MAP;
                    Eigen::Vector3d obs_vec(new_frame_->getPosition() - pt->getPosition());
                    float cur_dist = obs_vec.norm();
                    if (cur_dist <= map_point_dist_[index]) {
                        map_point_dist_[index] = cur_dist;
                        retrieve_visual_points_[index] = pt;
                    }
                }
            }
            if (!voxel_in_fov) {
                DeleteKeyList.push_back(position);
            }
        }
    }

    for (auto &key : DeleteKeyList) {
        sub_map_flag_.erase(key);
    }

    for (int i = 0; i < total_grids_; i++) {
        if (grid_type_[i] == TYPE_MAP) {
            visual::Point *pt = retrieve_visual_points_[i];
            Eigen::Vector2d pc(new_frame_->transWorld2Pixel(pt->getPosition()));
            Eigen::Vector3d pt_cam(new_frame_->transWorld2Cam(pt->getPosition()));
            
            bool depth_discontinous = false;
            for (int u = -half_patch_size_; u <= half_patch_size_; u++) {
                for (int v = -half_patch_size_; v <= half_patch_size_; v++) {
                    if (u == 0 && v == 0) continue;

                    float depth = it[cam_->width() * (v + int(pc[1])) + u + int(pc[0])];

                    if (depth == 0.0) continue;

                    double delta_dist = std::abs(pt_cam[2] - depth);

                    if (delta_dist > 0.5) {
                        depth_discontinous = true;
                        break;
                    }
                }
                if (depth_discontinous) break;
            }

            if (depth_discontinous) continue;

            visual::Patch *ref_patch;
            std::vector<float> patch_wrap(patch_warp_size_);

            int search_level;
            Eigen::Matrix2d A_cur_ref_zero;

            if (!pt->isNormalInitialized()) continue;
            // Normal enabled
            float phtometric_errors_min = std::numeric_limits<float>::max();
            if (pt->getObservedPatchs().size() == 1) {
                ref_patch = *pt->getObservedPatchs().begin();
                pt->setReferencePatch(ref_patch);
                pt->flagReferencePatch(true);
            } else if (!pt->hasReferencePatch()) {
                for (auto it = pt->getObservedPatchs().begin(), ite = pt->getObservedPatchs().end(); it != ite; ++it) {

                    visual::Patch *ref_patch_temp = *it;
                    float *patch_temp = ref_patch_temp->getPatchs();
                    float photometric_errors = 0.0;
                    int count = 0;
                    for (auto itm = pt->getObservedPatchs().begin(), itme = pt->getObservedPatchs().end(); itm != itme; ++itm) {
                        if ((*itm)->getId() == ref_patch_temp->getId()) continue;
                        float *patch_cache = (*itm)->getPatchs();

                        for (int ind = 0; ind < pixels_per_patch_; ind++) {
                            photometric_errors += (patch_temp[ind] - patch_cache[ind]) *
                                                (patch_temp[ind] - patch_cache[ind]);
                        }
                        count++;
                    }
                    photometric_errors = photometric_errors / count;
                    if (photometric_errors < phtometric_errors_min) {
                        phtometric_errors_min = photometric_errors;
                        ref_patch = ref_patch_temp;
                    }
                }
                pt->setReferencePatch(ref_patch);
                pt->flagReferencePatch(true);
            } else {
                ref_patch = pt->getReferencePatch();
            }

            Eigen::Vector3d norm_vec =
                (ref_patch->getPoseFrameWorld().rotationMatrix() * pt->getNormal()).normalized();
            Eigen::Vector3d pf(ref_patch->getPoseFrameWorld() * pt->getPosition());
            Sophus::SE3d T_cur_ref = new_frame_->getPoseFrameWorld() * ref_patch->getPoseFrameWorld().inverse();
            getWarpMatrixAffineHomography(*cam_, ref_patch->getPatchCenter(), pf, norm_vec,
                                        T_cur_ref, 0, A_cur_ref_zero);
            search_level = getBestSearchLevel(A_cur_ref_zero, 2);

            for (int pyramid_level = 0; pyramid_level <= patch_pyrimid_level_ - 1; pyramid_level++) {
                warpAffine(A_cur_ref_zero, ref_patch->getAssociatedImage(), ref_patch->getPatchCenter(), ref_patch->getExtractedPyramidLevel(),
                        search_level, pyramid_level, half_patch_size_,
                        patch_wrap.data());
            }
            getImagePatch(img, pc, patch_pixels_.data(), 0);
            
            float error = 0.0;
            for (int ind = 0; ind < pixels_per_patch_; ind++) {
                error += (ref_patch->getInvExpoTime() * patch_wrap[ind] -
                        system_inv_expo_time * patch_pixels_[ind]) *
                        (ref_patch->getInvExpoTime() * patch_wrap[ind] -
                        system_inv_expo_time * patch_pixels_[ind]);
            }

            if (error > outlier_threshold_ * pixels_per_patch_) continue;

            visual_submap_->visual_points.push_back(pt);
            visual_submap_->propa_errors.push_back(error);
            visual_submap_->search_levels.push_back(search_level);
            visual_submap_->errors.push_back(error);
            visual_submap_->warp_patch.push_back(patch_wrap);
            visual_submap_->inv_expo_list.push_back(ref_patch->getInvExpoTime());
        }
    }
    total_points_ = visual_submap_->visual_points.size();
    common::Utils::printColored("[ VIO ] Retrieve " + std::to_string(total_points_) + " points from visual sparse map", 
                                common::Color::kBlue, common::Style::kBold);
}

void VisualMap::getWarpMatrixAffineHomography(const vk::AbstractCamera &cam,
                                                const Eigen::Vector2d &px_ref, 
                                                const Eigen::Vector3d &xyz_ref,
                                                const Eigen::Vector3d &normal_ref,
                                                const Sophus::SE3d &T_cur_ref, 
                                                const int level_ref,
                                                Eigen::Matrix2d &A_cur_ref) {
    const Eigen::Vector3d t = T_cur_ref.inverse().translation();
    const Eigen::Matrix3d H_cur_ref =
        T_cur_ref.rotationMatrix() *
        (normal_ref.dot(xyz_ref) * Eigen::Matrix3d::Identity() -
        t * normal_ref.transpose());

    const int kHalfPatchSize = 4;
    Eigen::Vector3d f_du_ref(cam.cam2world(px_ref + Eigen::Vector2d(kHalfPatchSize, 0) *
                                            (1 << level_ref)));
    Eigen::Vector3d f_dv_ref(cam.cam2world(px_ref + Eigen::Vector2d(0, kHalfPatchSize) *
                                            (1 << level_ref)));

    const Eigen::Vector3d f_cur(H_cur_ref * xyz_ref);
    const Eigen::Vector3d f_du_cur = H_cur_ref * f_du_ref;
    const Eigen::Vector3d f_dv_cur = H_cur_ref * f_dv_ref;
    Eigen::Vector2d px_cur(cam.world2cam(f_cur));
    Eigen::Vector2d px_du_cur(cam.world2cam(f_du_cur));
    Eigen::Vector2d px_dv_cur(cam.world2cam(f_dv_cur));
    A_cur_ref.col(0) = (px_du_cur - px_cur) / kHalfPatchSize;
    A_cur_ref.col(1) = (px_dv_cur - px_cur) / kHalfPatchSize;
}

int VisualMap::getBestSearchLevel(const Eigen::Matrix2d &A_cur_ref, const int max_level) {
  int search_level = 0;
  double D = A_cur_ref.determinant();
  while (D > 3.0 && search_level < max_level) {
    search_level += 1;
    D *= 0.25;
  }
  return search_level;
}

void VisualMap::warpAffine(const Eigen::Matrix2d &A_cur_ref, const cv::Mat &img_ref,
                            const Eigen::Vector2d &px_ref, const int level_ref,
                            const int search_level, const int pyramid_level,
                            const int halfpatch_size, float *patch) {
    const int patch_size = halfpatch_size * 2;
    // 参考帧到当前帧的变换
    const Eigen::Matrix2f A_ref_cur = A_cur_ref.inverse().cast<float>();
    if (isnan(A_ref_cur(0, 0))) {
        printf("Affine warp is NaN, probably camera has no translation\n");  // TODO
        return;
    }

    float *patch_ptr = patch;
    for (int y = 0; y < patch_size; ++y) {
        for (int x = 0; x < patch_size; ++x) {
            Eigen::Vector2f px_patch(x - halfpatch_size, y - halfpatch_size);
            px_patch *= (1 << search_level);
            px_patch *= (1 << pyramid_level);
            const Eigen::Vector2f px(A_ref_cur * px_patch + px_ref.cast<float>());
            if (px[0] < 0 || px[1] < 0 || px[0] >= img_ref.cols - 1 ||
                px[1] >= img_ref.rows - 1)
                patch_ptr[pixels_per_patch_ * pyramid_level + y * patch_size + x] = 0;
            else
                patch_ptr[pixels_per_patch_ * pyramid_level + y * patch_size + x] =
                    (float)vk::interpolateMat_8u(img_ref, px[0], px[1]);
        }
    }
}

void VisualMap::getImagePatch(cv::Mat img, Eigen::Vector2d pc, float *patch_tmp, int level) {
  const float u_ref = pc[0];
  const float v_ref = pc[1];
  const int scale = (1 << level);
  const int u_ref_i = floorf(pc[0] / scale) * scale;
  const int v_ref_i = floorf(pc[1] / scale) * scale;
  const float subpix_u_ref = (u_ref - u_ref_i) / scale;
  const float subpix_v_ref = (v_ref - v_ref_i) / scale;
  const float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
  const float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref);
  const float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
  const float w_ref_br = subpix_u_ref * subpix_v_ref;
  for (int x = 0; x < patch_size_; x++) {
    uint8_t *img_ptr =
        (uint8_t *)img.data +
        (v_ref_i - half_patch_size_ * scale + x * scale) * cam_->width() +
        (u_ref_i - half_patch_size_ * scale);
    for (int y = 0; y < patch_size_; y++, img_ptr += scale) {
      patch_tmp[pixels_per_patch_ * level + x * patch_size_ + y] =
          w_ref_tl * img_ptr[0] + w_ref_tr * img_ptr[scale] +
          w_ref_bl * img_ptr[scale * cam_->width()] +
          w_ref_br * img_ptr[scale * cam_->width() + scale];
    }
  }
}

void VisualMap::updateFrameState(const frontend::SystemState& state) {
    Eigen::Matrix3d Rwi(state.getSystemRotation());
    Eigen::Vector3d Pwi(state.getSystemPosition());
    Rcw_ = Rci_ * Rwi.transpose();
    Pcw_ = -Rci_ * Rwi.transpose() * Pwi + Pci_;

    Sophus::SE3d Tcw(common::Utils::Mat3ToSO3(Rcw_), Pcw_);

    new_frame_->setPoseFrameWorld(Tcw);

}

void VisualMap::setImuToLidarExtrinsic(const std::vector<double>& R, const std::vector<double>& P) {
    Eigen::Matrix3d R_imu_lidar;
    Eigen::Vector3d P_imu_lidar;
    R_imu_lidar << MAT_FROM_ARRAY(R);
    P_imu_lidar << VEC_FROM_ARRAY(P);
    Pli_ = -R_imu_lidar.transpose() * P_imu_lidar;
    Rli_ = R_imu_lidar.transpose();
}

void VisualMap::setLidarToCameraExtrinsic(const std::vector<double>& R, const std::vector<double>& P) {
    Rcl_ << MAT_FROM_ARRAY(R);
    Pcl_ << VEC_FROM_ARRAY(P);
}

void VisualMap::resetGrid() {
    std::fill(grid_type_.begin(), grid_type_.end(), TYPE_UNKNOWN);
    std::fill(map_point_dist_.begin(), map_point_dist_.end(), 10000.0f);
    std::fill(update_flag_.begin(), update_flag_.end(), 0);
    std::fill(point_score_.begin(), point_score_.end(), 0.0f);

    retrieve_visual_points_.clear();
    retrieve_visual_points_.resize(total_grids_);

    append_voxel_points_.clear();
    append_voxel_points_.resize(total_grids_);

    total_points_ = 0;
}

void VisualMap::generateVisualMapPoints(cv::Mat img, 
                                        std::vector<PointWithCovarience> &pc_list,
                                        double system_inv_expo_time) {
  if (pc_list.size() <= 10) return;
  // Traverse the current frame point cloud data and add visual points to the pending list

  for (size_t i = 0; i < pc_list.size(); i++) {
    if (pc_list[i].normal == Eigen::Vector3d(0, 0, 0)) continue;
    Eigen::Vector3d pt = pc_list[i].global_point;
    Eigen::Vector2d pc(new_frame_->transWorld2Pixel(pt));

    if (new_frame_->getAbstractCamera()->isInFrame(pc.cast<int>(), border_)) {

      int index = static_cast<int>(pc[1] / grid_size_) * grids_along_width_ +
                  static_cast<int>(pc[0] / grid_size_);
      if (grid_type_[index] != TYPE_MAP) {
        // Shi-Tomasi 角点检测
        float cur_value = vk::shiTomasiScore(img, pc[0], pc[1]);
        // if (cur_value < 5) continue;
        if (cur_value > point_score_[index]) {
          point_score_[index] = cur_value;
          append_voxel_points_[index] = pc_list[i];
          grid_type_[index] = TYPE_POINTCLOUD;

        }
      }
    }
  }

  // Traverse the visual submap (which may include historical points, etc.) and add points to the pending list
  for (size_t j = 0; j < visual_submap_->add_from_voxel_map.size(); j++) {
    Eigen::Vector3d pt = visual_submap_->add_from_voxel_map[j].global_point;
    Eigen::Vector2d pc(new_frame_->transWorld2Pixel(pt));
    // 20px is the patch size in the matcher
    if (new_frame_->getAbstractCamera()->isInFrame(pc.cast<int>(), border_)) {
      int index = static_cast<int>(pc[1] / grid_size_) * grids_along_width_ +
                  static_cast<int>(pc[0] / grid_size_);

      if (grid_type_[index] != TYPE_MAP) {
        float cur_value = vk::shiTomasiScore(img, pc[0], pc[1]);
        if (cur_value > point_score_[index]) {
          point_score_[index] = cur_value;
          append_voxel_points_[index] = visual_submap_->add_from_voxel_map[j];
          grid_type_[index] = TYPE_POINTCLOUD;
        }
      }
    }
  }

  int add = 0;
  for (int i = 0; i < total_grids_; i++) {
    if (grid_type_[i] == TYPE_POINTCLOUD) {
      PointWithCovarience pt_cov = append_voxel_points_[i];
      Eigen::Vector3d pt = pt_cov.global_point;

      Eigen::Vector3d norm_vec(new_frame_->getPoseFrameWorld().rotationMatrix() * pt_cov.normal);
      Eigen::Vector3d dir(new_frame_->getPoseFrameWorld() * pt);
      dir.normalize();
      double cos_theta = dir.dot(norm_vec);
    
      Eigen::Vector2d pc(new_frame_->transWorld2Pixel(pt));

      float *patch = new float[pixels_per_patch_];
      getImagePatch(img, pc, patch, 0);

      visual::Point *pt_new = new visual::Point(pt);

      Eigen::Vector3d f = cam_->cam2world(pc);
      visual::Patch *patch_new =
          new visual::Patch(pt_new, patch, pc, f, new_frame_->getPoseFrameWorld(), 0);
      patch_new->setAssociatedImage(img);
      patch_new->setId(new_frame_->getId());
      patch_new->setInvExpoTime(system_inv_expo_time);

      pt_new->addPatch(patch_new);
      pt_new->setCovariance(pt_cov.global_point_covariance);
      pt_new->flagNormalInitialized(true);

      if (cos_theta < 0) {
        pt_new->setNormal(-pt_cov.normal);
      } else {
        pt_new->setNormal(pt_cov.normal);
      }

      pt_new->setPreviousNormal(pt_new->getNormal());

      insertPointIntoVisualMap(pt_new);
      add += 1;
    }
  }

  common::Utils::printColored("[ VIO ] Append " + std::to_string(add) + " new visual map points", 
                              common::Color::kBlue, common::Style::kBold);

}

void VisualMap::insertPointIntoVisualMap(visual::Point *pt_new) {
  Eigen::Vector3d pt_w(pt_new->getPosition()[0], pt_new->getPosition()[1], pt_new->getPosition()[2]);
  float loc_xyz[3];
  for (int j = 0; j < 3; j++) {
    loc_xyz[j] = pt_w[j] / map_voxel_size_;
    if (loc_xyz[j] < 0) {
      loc_xyz[j] -= 1.0;
    }
  }
  VoxelLocation position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                          (int64_t)loc_xyz[2]);
  auto iter = visualpoint_map_.find(position);
  if (iter != visualpoint_map_.end()) {

    iter->second->second->visual_points.push_back(pt_new);
    iter->second->second->count++;

    visualpoint_map_cache_.splice(visualpoint_map_cache_.begin(), visualpoint_map_cache_, iter->second);
    iter->second = visualpoint_map_cache_.begin();
  } else {
    VisualPoints *ot = new VisualPoints(0);
    ot->visual_points.push_back(pt_new);
    visualpoint_map_cache_.push_front({position, {ot}});
    visualpoint_map_.insert({position, visualpoint_map_cache_.begin()});

    if (visualpoint_map_cache_.size() >= static_cast<size_t>(capacity_)) {

      visualpoint_map_.erase(visualpoint_map_cache_.back().first);
      delete visualpoint_map_cache_.back().second;
      visualpoint_map_cache_.pop_back();
    }
  }

}

void VisualMap::plotTrackedPoints(cv::Mat &img_cp) {
    int total_points = visual_submap_->visual_points.size();
    if (total_points == 0) return;

    for (int i = 0; i < total_points; i++) {
        visual::Point *pt = visual_submap_->visual_points[i];
        Eigen::Vector2d pc(new_frame_->transWorld2Pixel(pt->getPosition()));

        if (visual_submap_->errors[i] <= visual_submap_->propa_errors[i]) {
            cv::circle(img_cp, cv::Point2f(pc[0], pc[1]), 7, cv::Scalar(0, 255, 0),
                        -1, 8);  // Green Sparse Align tracked
        } else {
            cv::circle(img_cp, cv::Point2f(pc[0], pc[1]), 7, cv::Scalar(0, 0, 255),
                        -1, 8);  // Red Sparse Align tracked
        }
    }
}

void VisualMap::updateVisualMap(cv::Mat img, double system_inv_expo_time) {
    if (total_points_ == 0) return;

    int update_num = 0;
    Sophus::SE3d pose_cur = new_frame_->getPoseFrameWorld();
    for (int i = 0; i < total_points_; i++) {
        visual::Point *pt = visual_submap_->visual_points[i];
        if (pt == nullptr) continue;
        if (pt->isConverged()) {
            pt->removeNonReferencePatch();
            continue;
        }

        Eigen::Vector2d pc(new_frame_->transWorld2Pixel(pt->getPosition()));
        bool add_flag = false;

        float *patch_temp = new float[pixels_per_patch_];
        getImagePatch(img, pc, patch_temp, 0);

        visual::Patch *last_patch = pt->getObservedPatchs().back();

        // Step 2: delta_pose
        Sophus::SE3d pose_ref = last_patch->getPoseFrameWorld();
        Sophus::SE3d delta_pose = pose_ref * pose_cur.inverse();
        double delta_p = delta_pose.translation().norm();
        double delta_theta =
            (delta_pose.rotationMatrix().trace() > 3.0 - 1e-6)
                ? 0.0
                : std::acos(0.5 * (delta_pose.rotationMatrix().trace() - 1));
        if (delta_p > 0.5 || delta_theta > 0.3) add_flag = true;  // 0.5 || 0.3

        // Step 3: pixel distance
        Eigen::Vector2d last_px = last_patch->getPatchCenter();
        double pixel_dist = (pc - last_px).norm();
        if (pixel_dist > 40) add_flag = true;

        // Maintain the size of 3D point observation features. 
        if (pt->getObservedPatchs().size() >= 30) {
            visual::Patch *ref_ftr;
            pt->findMinScorePatch(new_frame_->getPosition(), ref_ftr);
            pt->removeReferencePatch(ref_ftr);
        }
        if (add_flag) {
            update_num += 1;
            update_flag_[i] = 1;
            Eigen::Vector3d f = cam_->cam2world(pc);
            visual::Patch *new_patch = new visual::Patch(pt, patch_temp, pc, f, new_frame_->getPoseFrameWorld(),
                                            visual_submap_->search_levels[i]);
            new_patch->setAssociatedImage(img);
            new_patch->setId(new_frame_->getId());
            new_patch->setInvExpoTime(system_inv_expo_time);
            pt->addPatch(new_patch);
        }
    }
    common::Utils::printColored("[ VIO ] Update " + std::to_string(update_num) + " points in visual submap", 
                              common::Color::kBlue, common::Style::kBold);
}

void VisualMap::updateReferencePatch(const std::unordered_map<VoxelLocation,
   typename std::list<std::pair<VoxelLocation, VoxelOctoTree *>>::iterator> &voxel_map) {
    if (total_points_ == 0) return;

    for (size_t i = 0; i < visual_submap_->visual_points.size(); i++) {
        visual::Point *pt = visual_submap_->visual_points[i];
        if (!pt->isNormalInitialized()) continue;
        if (pt->isConverged()) continue;
        if (pt->getObservedPatchs().size() <= 5) continue;
        if (update_flag_[i] == 0) continue;
        const Eigen::Vector3d &p_w = pt->getPosition();
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = p_w[j] / map_voxel_size_;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }

        VoxelLocation position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                                (int64_t)loc_xyz[2]);
        auto iter = voxel_map.find(position);
        if (iter != voxel_map.end()) {
            VoxelOctoTree *current_octo;
            current_octo = iter->second->second->findCorrespondPoint(p_w);
            if (current_octo->getPlanePtr()->is_plane) {
                VoxelPlane &plane = *current_octo->getPlanePtr();
                float dis_to_plane = plane.plane_normal(0) * p_w(0) +
                                    plane.plane_normal(1) * p_w(1) +
                                    plane.plane_normal(2) * p_w(2) + plane.plane_intercept;
                float dis_to_plane_abs = std::fabs(dis_to_plane);
                float dis_to_center =
                    (plane.plane_center(0) - p_w(0)) * (plane.plane_center(0) - p_w(0)) +
                    (plane.plane_center(1) - p_w(1)) * (plane.plane_center(1) - p_w(1)) +
                    (plane.plane_center(2) - p_w(2)) * (plane.plane_center(2) - p_w(2));
                float range_dis = std::sqrt(dis_to_center - dis_to_plane * dis_to_plane);
                if (range_dis <= 3 * plane.radius) {
                    Eigen::Matrix<double, 1, 6> J_nq;
                    J_nq.block<1, 3>(0, 0) = p_w - plane.plane_center;
                    J_nq.block<1, 3>(0, 3) = -plane.plane_normal;
                    double sigma_l = J_nq * plane.plane_covariance * J_nq.transpose();
                    sigma_l +=
                        plane.plane_normal.transpose() * pt->getCovariance() * plane.plane_normal;
                    
                    if (dis_to_plane_abs < 3 * std::sqrt(sigma_l)) {

                        if (pt->getPreviousNormal().dot(plane.plane_normal) < 0) {
                            pt->setNormal(-plane.plane_normal);
                        } else {
                            pt->setNormal(plane.plane_normal);
                        }

                        double normal_update = (pt->getNormal() - pt->getPreviousNormal()).norm();

                        pt->setPreviousNormal(pt->getNormal());

                        if (normal_update < 0.0001 && pt->getObservedPatchs().size() > 10) {
                            pt->setConverged(true);
                        }
                    }
                }
            }
        }
        // Calculate NCC score (Formula 12) and select the optimal reference image patch
        float score_max = -1000.0f;
        for (auto it = pt->getObservedPatchs().begin(), ite = pt->getObservedPatchs().end(); it != ite; ++it) {
            visual::Patch *ref_patch_temp = *it;
            float *patch_temp = ref_patch_temp->getPatchs();
            float NCC_up = 0.0;
            float NCC_down1 = 0.0;
            float NCC_down2 = 0.0;
            float NCC = 0.0;
            float score = 0.0;
            int count = 0;
           
            Eigen::Vector3d pf = ref_patch_temp->getPoseFrameWorld() * pt->getPosition();
            Eigen::Vector3d norm_vec = ref_patch_temp->getPoseFrameWorld().rotationMatrix() * pt->getNormal();

            pf.normalize();
            double cos_angle = pf.dot(norm_vec);
            // if(fabs(cos_angle) < 0.86) continue; // 20 degree

            float ref_mean;
            if (std::abs(ref_patch_temp->getMeanIntensity()) < 1e-6) {
                float ref_sum =
                    std::accumulate(patch_temp, patch_temp + pixels_per_patch_, 0.0);
                ref_mean = ref_sum / pixels_per_patch_;
                ref_patch_temp->setMeanIntensity(ref_mean);
            }

            for (auto itm = pt->getObservedPatchs().begin(), itme = pt->getObservedPatchs().end(); itm != itme; ++itm) {

                if ((*itm)->getId() == ref_patch_temp->getId()) continue;

                float *patch_cache = (*itm)->getPatchs();

                float other_mean;
                if (std::abs((*itm)->getMeanIntensity()) < 1e-6) {
                float other_sum = std::accumulate(
                    patch_cache, patch_cache + pixels_per_patch_, 0.0);
                other_mean = other_sum / pixels_per_patch_;
                (*itm)->setMeanIntensity(other_mean);
                }

                for (int ind = 0; ind < pixels_per_patch_; ind++) {
                    NCC_up +=
                        (patch_temp[ind] - ref_mean) * (patch_cache[ind] - other_mean);
                    NCC_down1 +=
                        (patch_temp[ind] - ref_mean) * (patch_temp[ind] - ref_mean);
                    NCC_down2 +=
                        (patch_cache[ind] - other_mean) * (patch_cache[ind] - other_mean);
                }

                NCC += std::fabs(NCC_up / std::sqrt(NCC_down1 * NCC_down2));

                count++;
            }

            NCC = NCC / count;

            score = NCC + cos_angle;
            ref_patch_temp->setScore(score);

            if (score > score_max) {
                score_max = score;
                pt->setReferencePatch(ref_patch_temp);
                pt->flagReferencePatch(true);
            }

        }

    }
   
}

Eigen::Vector3f VisualMap::getInterpolatedPixel(cv::Mat img, Eigen::Vector2d pc) {
    const float u_ref = pc[0];
    const float v_ref = pc[1];
    const int u_ref_i = floorf(pc[0]);
    const int v_ref_i = floorf(pc[1]);
    const float subpix_u_ref = (u_ref - u_ref_i);
    const float subpix_v_ref = (v_ref - v_ref_i);
    const float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
    const float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref);
    const float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
    const float w_ref_br = subpix_u_ref * subpix_v_ref;
    uint8_t *img_ptr = (uint8_t *)img.data + ((v_ref_i)*cam_->width() + (u_ref_i)) * 3;
    float B = w_ref_tl * img_ptr[0] + w_ref_tr * img_ptr[0 + 3] +
                w_ref_bl * img_ptr[cam_->width() * 3] +
                w_ref_br * img_ptr[cam_->width() * 3 + 0 + 3];
    float G = w_ref_tl * img_ptr[1] + w_ref_tr * img_ptr[1 + 3] +
                w_ref_bl * img_ptr[1 + cam_->width() * 3] +
                w_ref_br * img_ptr[cam_->width() * 3 + 1 + 3];
    float R = w_ref_tl * img_ptr[2] + w_ref_tr * img_ptr[2 + 3] +
                w_ref_bl * img_ptr[2 + cam_->width() * 3] +
                w_ref_br * img_ptr[cam_->width() * 3 + 2 + 3];
    Eigen::Vector3f pixel(B, G, R);
    return pixel;
}

Eigen::Vector3d VisualMap::transGpointToCamera(const Eigen::Vector3d &global_point) {
    return new_frame_->transWorld2Cam(global_point);
}

Eigen::Vector2d VisualMap::transGpointToPixel(const Eigen::Vector3d &global_point) {
    return new_frame_->transWorld2Pixel(global_point);
}

bool VisualMap::checkPixelInFrame(const Eigen::Vector2d &pixel, int level) {
    return new_frame_->getAbstractCamera()->isInFrame(pixel.cast<int>(), level);
}

SubSparseMap* VisualMap::getVisualSubmap() {
    return visual_submap_;
}

vk::AbstractCamera* VisualMap::getAbstractCamera() {
    return cam_;
}

void VisualMap::resetNewFrame(const cv::Mat &img) {
    new_frame_.reset(new visual::Frame(cam_, img));
}

int VisualMap::getTotalPoints() {
    return total_points_;
}

int VisualMap::getPixelsPerPatch() {
    return pixels_per_patch_;
}

int VisualMap::getPatchSize() {
    return patch_size_;
}

int VisualMap::getHalfPatchSize() {
    return half_patch_size_;
}

Eigen::Matrix3d& VisualMap::getRci() {
    return Rci_;
}

Eigen::Vector3d& VisualMap::getPci() {
    return Pci_;
}

Eigen::Matrix3d& VisualMap::getRcw() {
    return Rcw_;
}

Eigen::Vector3d& VisualMap::getPcw() {
    return Pcw_;
}

Eigen::Matrix3d& VisualMap::getJdphidR() {
    return Jdphi_dR_;
}

Eigen::Matrix3d& VisualMap::getJdpdt() {
    return Jdp_dR_;
}

Eigen::Matrix3d& VisualMap::getJdpdR() {
    return Jdp_dR_;
}

VisualMap::~VisualMap() {
    delete visual_submap_;
    for (auto &pair : visualpoint_map_) delete pair.second->second;
    visualpoint_map_.clear();
    visualpoint_map_cache_.clear();
}

    
} // namespace map