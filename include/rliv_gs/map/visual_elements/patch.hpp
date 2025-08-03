/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef PATCH_HPP
#define PATCH_HPP

#include <vikit/math_utils.h>


#include "point.hpp"


namespace visual {

class Patch {
public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   Patch(Point* point, float* patchs, const Eigen::Vector2d& patch_center, const Eigen::Vector3d& unit_direction, const Sophus::SE3d& pose_frame_world, int extracted_pyramid_level);
   ~Patch();

   Eigen::Vector3d getPosition() const;

   // Getters
   int getId() const;
   cv::Mat getAssociatedImage() const;
   Eigen::Vector2d getPatchCenter() const;
   Eigen::Vector3d getUnitDirection() const;
   int getExtractedPyramidLevel() const;
   Point* getPoint() const;
   Sophus::SE3d getPoseFrameWorld() const;
   float* getPatchs() const;
   float getScore() const;
   float getMeanIntensity() const;
   double getInvExpoTime() const;

   // Setters
   void setId(int id);
   void setAssociatedImage(const cv::Mat& associated_image);
   void setPatchCenter(const Eigen::Vector2d& patch_center);
   void setUnitDirection(const Eigen::Vector3d& unit_direction);
   void setExtractedPyramidLevel(int extracted_pyramid_level);
   void setPoint(Point* point);
   void setPoseFrameWorld(const Sophus::SE3d& pose_frame_world);
   void setPatchs(float* patchs);
   void setScore(float score);
   void setMeanIntensity(float mean_intensity);
   void setInvExpoTime(double inv_expo_time);

private:
    int id_;
    cv::Mat associated_image_; // the image associated with the patch
    Eigen::Vector2d patch_center_; // the center of the patch
    Eigen::Vector3d unit_direction_; // the unit direction of the patch
    Sophus::SE3d pose_frame_world_; // the global pose of the camera when the patch was extracted
    int extracted_pyramid_level_; // the pyramid level at which the patch was extracted
    Point* point_; // the point associated with the patch
    float* patchs_; // pointer to the image patchs
    float mean_intensity_; // the mean intensity of the patch
    float score_; // the score of the patch
    double inv_expo_time_; // the inverse exposure time of the image associated with the patch
};

} // namespace visual






#endif // PATCH_HPP