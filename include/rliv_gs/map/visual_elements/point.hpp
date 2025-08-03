/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef POINT_HPP
#define POINT_HPP

#include <boost/noncopyable.hpp>
#include <Eigen/Eigen>

#include "frame.hpp"  





namespace visual {


class Point : boost::noncopyable {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Point(const Eigen::Vector3d& position);
    ~Point();

    void findMinScorePatch(const Eigen::Vector3d &frame_position, Patch* &patch) const;
    void removeNonReferencePatch();
    void removeReferencePatch(Patch* patch);
    void addPatch(Patch* patch);

    bool getNearestPatch(const Eigen::Vector3d &frame_position, Patch* &patch,
                       const Eigen::Vector2d &currrent_pixel) const;
    std::list<Patch *>& getObservedPatchs();
    Eigen::Vector3d getPosition() const;
    Eigen::Vector3d getNormal() const;
    Eigen::Vector3d getPreviousNormal() const;
    Patch* getReferencePatch() const;
    Eigen::Matrix3d getCovariance() const;

    void setReferencePatch(Patch* patch);
    void setCovariance(const Eigen::Matrix3d &covariance);
    void setNormal(const Eigen::Vector3d &normal);
    void setPreviousNormal(const Eigen::Vector3d &normal);
    void setConverged(bool is_converged);


    void flagReferencePatch(bool has_ref_patch);
    bool hasReferencePatch() const;
    bool isNormalInitialized() const;
    bool isConverged() const;
    void flagNormalInitialized(bool is_normal_initialized);









private:
    Eigen::Vector3d global_position_; // position in world coordinate
    Eigen::Vector3d normal_; // normal in the plane
    Eigen::Vector3d previous_normal_; // last normal in the plane
    Eigen::Matrix3d inv_cov_normal_; // inverse covariance of the normal
    std::list<Patch *> observed_patchs_; // patches that observed this point
    Eigen::Matrix3d covariance_; // covariance of the point
    bool is_converged_; // whether the point is converged
    bool is_normal_initialized_; // whether the normal is initialized
    bool has_ref_patch_; // whether the point has reference patch
    Patch *ref_patch_; // reference patch
};

} // namespace visual




#endif // POINT_HPP