/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/
#include "rliv_gs/map/visual_elements/patch.hpp"
#include "rliv_gs/map/visual_elements/frame.hpp"
#include "rliv_gs/map/visual_elements/point.hpp"

namespace visual {

int Frame::frame_counter_ = 0;

Frame::Frame(vk::AbstractCamera* cam, const cv::Mat& img) : id_(frame_counter_++), cam_(cam) {
    initFrame(img);
}

void Frame::initFrame(const cv::Mat &img) {
  if (img.empty()) { throw std::runtime_error("Frame: provided image is empty"); }

  if (img.cols != cam_->width() || img.rows != cam_->height())
  {
    common::Utils::printColored("Image size: " + std::to_string(img.cols) + "x" + std::to_string(img.rows), 
                                common::Color::kRed, common::Style::kBold);
    common::Utils::printColored("Camera model size: " + std::to_string(cam_->width()) + "x" + std::to_string(cam_->height()), 
                                common::Color::kRed, common::Style::kBold);
    throw std::runtime_error("Frame: provided image has not the same size as the camera model");
  }

  if (img.type() != CV_8UC1) { throw std::runtime_error("Frame: provided image is not grayscale"); }

  img_ = img;
}

int Frame::getId() const {
    return id_;
}

size_t Frame::getPatchesNum() const {
    return patchs_list_.size();
}

Eigen::Vector2d Frame::transWorld2Pixel(const Eigen::Vector3d &global_point) const {
    return cam_->world2cam(pose_frame_world_ * global_point); 
}

Eigen::Vector3d Frame::transWorld2Cam(const Eigen::Vector3d &global_point) const {
    return pose_frame_world_ * global_point;
}

Eigen::Vector3d Frame::transCam2World(const Eigen::Vector3d &camera_point) const {
    return pose_frame_world_.inverse() * camera_point;
}

Eigen::Vector3d Frame::getPosition() const {
    return pose_frame_world_.inverse().translation();
}

vk::AbstractCamera* Frame::getAbstractCamera() const {
    return cam_;
}

Sophus::SE3d Frame::getPoseFrameWorld() const {
    return pose_frame_world_;
}

void Frame::setPoseFrameWorld(const Sophus::SE3d &pose_frame_world) { 
    pose_frame_world_ = pose_frame_world;
}

Frame::~Frame() {
    std::for_each(patchs_list_.begin(), patchs_list_.end(), [&](Patch* i) { delete i; });
}

}