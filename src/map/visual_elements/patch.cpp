/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "rliv_gs/map/visual_elements/patch.hpp"

namespace visual {

Patch::Patch(Point* point, float* patchs, const Eigen::Vector2d& patch_center, const Eigen::Vector3d& unit_direction, 
    const Sophus::SE3d& pose_frame_world, int extracted_pyramid_level)
    : patch_center_(patch_center), unit_direction_(unit_direction), pose_frame_world_(pose_frame_world), 
    extracted_pyramid_level_(extracted_pyramid_level), point_(point), patchs_(patchs) {
    mean_intensity_ = 0.0f;
    score_ = 0.0f;
}

Eigen::Vector3d Patch::getPosition() const {
    return pose_frame_world_.inverse().translation();
}

int Patch::getId() const {
    return id_;
}

cv::Mat Patch::getAssociatedImage() const {
    return associated_image_;
}

Eigen::Vector2d Patch::getPatchCenter() const {
    return patch_center_;
}

Eigen::Vector3d Patch::getUnitDirection() const {
    return unit_direction_;
}

int Patch::getExtractedPyramidLevel() const {
    return extracted_pyramid_level_;
}

Point* Patch::getPoint() const {
    return point_;
}

Sophus::SE3d Patch::getPoseFrameWorld() const {
    return pose_frame_world_;
}

float* Patch::getPatchs() const {
    return patchs_;
}

float Patch::getScore() const {
    return score_;
}

float Patch::getMeanIntensity() const {
    return mean_intensity_;
}

double Patch::getInvExpoTime() const {
    return inv_expo_time_;
}

void Patch::setId(int id) {
    id_ = id;
}

void Patch::setAssociatedImage(const cv::Mat& associated_image) {
    associated_image_ = associated_image;
}

void Patch::setPatchCenter(const Eigen::Vector2d& patch_center) {
    patch_center_ = patch_center;
}

void Patch::setUnitDirection(const Eigen::Vector3d& unit_direction) {
    unit_direction_ = unit_direction;
}

void Patch::setExtractedPyramidLevel(int extracted_pyramid_level) {
    extracted_pyramid_level_ = extracted_pyramid_level;
}

void Patch::setPoint(Point* point) {
    point_ = point;
}

void Patch::setPoseFrameWorld(const Sophus::SE3d& pose_frame_world) {
    pose_frame_world_ = pose_frame_world;
}

void Patch::setPatchs(float* patchs) {
    patchs_ = patchs;
}

void Patch::setScore(float score) {
    score_ = score;
}

void Patch::setMeanIntensity(float mean_intensity) {
    mean_intensity_ = mean_intensity;
}

void Patch::setInvExpoTime(double inv_expo_time) {
    inv_expo_time_ = inv_expo_time;
}

Patch::~Patch() {
    delete[] patchs_;
}

}