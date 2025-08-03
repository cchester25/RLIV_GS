/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef FRAME_HPP
#define FRAME_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <boost/noncopyable.hpp>

#include <vikit/abstract_camera.h>
#include <vikit/math_utils.h>

#include "rliv_gs/common/utils.hpp"


namespace visual {

class Point;
class Patch;
typedef std::list<Patch *> Patchs;

class Frame : boost::noncopyable {
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Frame(vk::AbstractCamera *cam, const cv::Mat &img);
    ~Frame();

    void initFrame(const cv::Mat &img);

    Eigen::Vector2d transWorld2Pixel(const Eigen::Vector3d &global_point) const; // transform world coordinate to image coordinate
    Eigen::Vector3d transWorld2Cam(const Eigen::Vector3d &global_point) const; // transform world coordinate to camera coordinate
    Eigen::Vector3d transCam2World(const Eigen::Vector3d &camera_point) const; // transform camera coordinate to world coordinate
    
    int getId() const; // get the id of the frame
    size_t getPatchesNum() const; // get the number of patches
    Eigen::Vector3d getPosition() const; // get the position of the frame
    vk::AbstractCamera* getAbstractCamera() const; // get the camera model
    Sophus::SE3d getPoseFrameWorld() const; // get the pose of the frame

    void setPoseFrameWorld(const Sophus::SE3d &pose_frame_world); // set the pose of the frame




private:
    int id_; // frame id
    static int frame_counter_; // frame counter
    vk::AbstractCamera* cam_; // camera model
    Sophus::SE3d pose_frame_world_;
    cv::Mat img_; // image of the frame
    Patchs patchs_list_; // patches of the frame


};

typedef std::unique_ptr<Frame> FramePtr;
} // namespace visual






#endif // FRAME_HPP