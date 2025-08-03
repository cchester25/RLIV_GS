/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "rliv_gs/map/visual_elements/point.hpp"
#include "rliv_gs/map/visual_elements/patch.hpp"

namespace visual {

Point::Point(const Eigen::Vector3d &position) : global_position_(position) {
    normal_ = Eigen::Vector3d::Zero();
    previous_normal_ = Eigen::Vector3d::Zero();
    is_converged_ = false;
    is_normal_initialized_ = false;
    has_ref_patch_ = false;
}

void Point::findMinScorePatch(const Eigen::Vector3d &frame_position, Patch* &patch) const {
    auto min_it = observed_patchs_.begin();
    float min_score = std::numeric_limits<float>::max();

    for (auto it = observed_patchs_.begin(), ite = observed_patchs_.end(); it != ite; ++it) {
        if ((*it)->getScore() < min_score) {
            min_score = (*it)->getScore();
            min_it = it;
        }
    }
    patch = *min_it;
}

void Point::removeNonReferencePatch() {
  for (auto it = observed_patchs_.begin(); it != observed_patchs_.end();) {
    if (*it != ref_patch_) {
      delete *it;
      it = observed_patchs_.erase(it);
    } else {
      ++it;
    }
  }
}

void Point::removeReferencePatch(Patch* patch) {
  if (ref_patch_ == patch) {
    ref_patch_ = nullptr;
    has_ref_patch_ = false;
  }
  for (auto it = observed_patchs_.begin(), ite = observed_patchs_.end(); it != ite; ++it) {
    if ((*it) == patch) {
      delete((*it));
      observed_patchs_.erase(it);
      return;
    }
  }
}

void Point::addPatch(Patch* patch) {
    observed_patchs_.push_front(patch);
}

bool Point::getNearestPatch(const Eigen::Vector3d &frame_position, Patch* &patch,
                       const Eigen::Vector2d &currrent_pixel) const {
    if (observed_patchs_.size() <= 0) return false;

    Eigen::Vector3d obs_dir(frame_position - global_position_);
    obs_dir.normalize();
    auto min_it = observed_patchs_.begin();
    double min_cos_angle = 0;
    for (auto it = observed_patchs_.begin(), ite = observed_patchs_.end(); it != ite; ++it) {
      Eigen::Vector3d dir((*it)->getPosition() - global_position_);
      dir.normalize();
      double cos_angle = obs_dir.dot(dir);
      if (cos_angle > min_cos_angle) {
        min_cos_angle = cos_angle;
        min_it = it;
      }
    }
    patch = *min_it;

    if (min_cos_angle < 0.5) {
      return false;
    }

    return true;
}

std::list<Patch *>& Point::getObservedPatchs() {
    return observed_patchs_;
}

Eigen::Vector3d Point::getNormal() const {
    return normal_;
}

Eigen::Vector3d Point::getPosition() const {
    return global_position_;
}

bool Point::isNormalInitialized() const {
    return is_normal_initialized_;
}

void Point::flagNormalInitialized(bool is_normal_initialized) {
    is_normal_initialized_ = is_normal_initialized;
}

Eigen::Matrix3d Point::getCovariance() const {
    return covariance_;
}

Patch* Point::getReferencePatch() const {
    return ref_patch_;
}

void Point::setReferencePatch(Patch* patch) {
    ref_patch_ = patch;
}

void Point::setCovariance(const Eigen::Matrix3d &covariance) {
    covariance_ = covariance;
}

void Point::setNormal(const Eigen::Vector3d &normal) {
    normal_ = normal;
}

Eigen::Vector3d Point::getPreviousNormal() const {
    return previous_normal_;
}

void Point::setPreviousNormal(const Eigen::Vector3d &normal) {
    previous_normal_ = normal;
}

void Point::flagReferencePatch(bool has_ref_patch) {
    has_ref_patch_ = has_ref_patch;
}
bool Point::hasReferencePatch() const {
    return has_ref_patch_;
}

void Point::setConverged(bool is_converged) {
    is_converged_ = is_converged;
}

bool Point::isConverged() const {
    return is_converged_;
}

Point::~Point() {
  for (auto it = observed_patchs_.begin(), ite = observed_patchs_.end(); it != ite; ++it)
  {
    delete(*it);
  }
  observed_patchs_.clear();
  ref_patch_ = nullptr;
}



}