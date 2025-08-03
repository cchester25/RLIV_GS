/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "rliv_gs/map/voxel_map.hpp"

namespace map {
    VoxelOctoTree::VoxelOctoTree(int octree_max_depth, 
                                int layer, 
                                int node_min_points, 
                                int node_max_points, 
                                float planer_threshold)
                                  : octree_max_depth_(octree_max_depth), 
                                    layer_(layer), 
                                    node_min_points_(node_min_points), 
                                    node_max_points_(node_max_points),
                                    plane_threshold_(planer_threshold) {
        temp_points_.clear();
        octo_state_ = 0;
        new_points_ = 0;
        update_size_threshold_ = 5;
        init_octo_ = false;
        update_enable_ = true;
        for (int i = 0; i < 8; i++)
        {
            leaves_[i] = nullptr;
        }
        plane_ptr_ = new VoxelPlane;
    }

    void VoxelOctoTree::initPlane(const std::vector<PointWithCovarience>& points, VoxelPlane* plane) {
        plane->plane_covariance = Eigen::Matrix<double, 6, 6>::Zero();
        plane->points_covariance = Eigen::Matrix3d::Zero();
        plane->plane_center = Eigen::Vector3d::Zero();
        plane->plane_normal = Eigen::Vector3d::Zero();
        plane->points_size = points.size();
        plane->radius = 0;
        for (auto pv : points)
        {
            plane->points_covariance += pv.global_point * pv.global_point.transpose();
            plane->plane_center += pv.global_point;
        }
        plane->plane_center = plane->plane_center / plane->points_size;
        plane->points_covariance = plane->points_covariance / plane->points_size - 
                                    plane->plane_center * plane->plane_center.transpose();
        Eigen::EigenSolver<Eigen::Matrix3d> es(plane->points_covariance);
        Eigen::Matrix3cd evecs = es.eigenvectors();
        Eigen::Vector3cd evals = es.eigenvalues();
        Eigen::Vector3d evalsReal;
        evalsReal = evals.real();
        Eigen::Matrix3f::Index evalsMin, evalsMax;
        evalsReal.rowwise().sum().minCoeff(&evalsMin);
        evalsReal.rowwise().sum().maxCoeff(&evalsMax);
        int evalsMid = 3 - evalsMin - evalsMax;

        Eigen::Matrix3d J_Q;
        J_Q << 1.0 / plane->points_size, 0, 0, 0, 1.0 / plane->points_size, 0, 0, 0, 1.0 / plane->points_size;

        if (evalsReal(evalsMin) < plane_threshold_) {
            for (size_t i = 0; i < points.size(); i++) {
                Eigen::Matrix<double, 6, 3> J;
                Eigen::Matrix3d F;
                for (int m = 0; m < 3; m++) {
                    if (m != (int)evalsMin) {
                        Eigen::Matrix<double, 1, 3> F_m =
                            (points[i].global_point - plane->plane_center).transpose() / ((plane->points_size) * (evalsReal[evalsMin] - evalsReal[m])) *
                            (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() + evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
                        F.row(m) = F_m;
                    } else {
                        Eigen::Matrix<double, 1, 3> F_m;
                        F_m << 0, 0, 0;
                        F.row(m) = F_m;
                    }
                }
                J.block<3, 3>(0, 0) = evecs.real() * F;
                J.block<3, 3>(3, 0) = J_Q;
                plane->plane_covariance += J * points[i].global_point_covariance * J.transpose();
            }

            plane->plane_normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
            plane->tangent_y << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid), evecs.real()(2, evalsMid);
            plane->tangent_x << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax);
            plane->min_eigen_value = evalsReal(evalsMin);
            plane->mid_eigen_value = evalsReal(evalsMid);
            plane->max_eigen_value = evalsReal(evalsMax);
            plane->radius = std::sqrt(evalsReal(evalsMax));
            plane->plane_intercept = -(plane->plane_normal(0) * plane->plane_center(0) +
                        plane->plane_normal(1) * plane->plane_center(1) +
                        plane->plane_normal(2) * plane->plane_center(2));
            plane->is_plane = true;
            plane->is_update = true;
            if (!plane->is_init) {
                plane->plane_id = voxel_plane_id;
                voxel_plane_id++;
                plane->is_init = true;
            }
        } else {
            plane->is_update = true;
            plane->is_plane = false;
        }
    }

    void VoxelOctoTree::initOctoTree() {
        if (temp_points_.size() > static_cast<size_t>(node_min_points_)) {
            initPlane(temp_points_, plane_ptr_);
            if (plane_ptr_->is_plane == true) {
                octo_state_ = 0;
                // new added
                if (temp_points_.size() > static_cast<size_t>(node_max_points_)) {
                    update_enable_ = false;
                    std::vector<PointWithCovarience>().swap(temp_points_);
                    new_points_ = 0;
                }
            } else {
                octo_state_ = 1;
                cutOctoTree();
            }
            init_octo_ = true;
            new_points_ = 0;
        }
    }

    void VoxelOctoTree::cutOctoTree() {
        if (layer_ >= octree_max_depth_) {
            octo_state_ = 0;
            return;
        }
        for (size_t i = 0; i < temp_points_.size(); i++) {
            int xyz[3] = {0, 0, 0};
            if (temp_points_[i].global_point[0] > voxel_center_[0]) { xyz[0] = 1; }
            if (temp_points_[i].global_point[1] > voxel_center_[1]) { xyz[1] = 1; }
            if (temp_points_[i].global_point[2] > voxel_center_[2]) { xyz[2] = 1; }
            int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
            if (leaves_[leafnum] == nullptr) {
                leaves_[leafnum] = new VoxelOctoTree(octree_max_depth_, layer_ + 1, node_min_points_, node_max_points_, plane_threshold_);
                double voxel_center[3] = {voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_, 
                                            voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_, 
                                            voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_};
                leaves_[leafnum]->setVoxelCenter(voxel_center);
                float quater_length = quater_length_ / 2.0;
                leaves_[leafnum]->setQuaterLength(quater_length);
            }
            leaves_[leafnum]->addTempPoints(temp_points_[i]);
            leaves_[leafnum]->addNewPoints();
        }
        for (uint i = 0; i < 8; i++) {
            if (leaves_[i] != nullptr) {
                if (leaves_[i]->getTempPoints().size() > static_cast<size_t>(leaves_[i]->getNodeMinPoints())) {
                    initPlane(leaves_[i]->getTempPoints(), leaves_[i]->getPlanePtr());
                    if (leaves_[i]->getPlanePtr()->is_plane) {
                        leaves_[i]->setOctoState(0);
                        if (leaves_[i]->getTempPoints().size() > static_cast<size_t>(leaves_[i]->getNodeMinPoints())) {
                            leaves_[i]->setUpdateEnable(false);
                            leaves_[i]->resetTempPoints();
                            new_points_ = 0;
                        }
                    } else {
                        leaves_[i]->setOctoState(1);
                        leaves_[i]->cutOctoTree();
                    }
                    leaves_[i]->setInitOcto(true);
                    leaves_[i]->setNewPoints(0);
                }
            }
        }
    }

    void VoxelOctoTree::updateOctoTree(const PointWithCovarience& pc) {
        if (!init_octo_) {
            new_points_++;
            temp_points_.push_back(pc);
            if (temp_points_.size() > static_cast<size_t>(node_min_points_)) { 
                initOctoTree(); 
            }
        }
        else {
            if (plane_ptr_->is_plane) {
                if (update_enable_) {
                    new_points_++;
                    temp_points_.push_back(pc);
                    if (new_points_ > update_size_threshold_) {
                        initPlane(temp_points_, plane_ptr_);
                        new_points_ = 0;
                    }
                    if (temp_points_.size() >=  static_cast<size_t>(node_max_points_)) {
                        update_enable_ = false;
                        std::vector<PointWithCovarience>().swap(temp_points_);
                        new_points_ = 0;
                    }
                }
            }
            else {
                if (layer_ < octree_max_depth_) {
                    int xyz[3] = {0, 0, 0};
                    if (pc.global_point[0] > voxel_center_[0]) { xyz[0] = 1; }
                    if (pc.global_point[1] > voxel_center_[1]) { xyz[1] = 1; }
                    if (pc.global_point[2] > voxel_center_[2]) { xyz[2] = 1; }
                    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
                    if (leaves_[leafnum] != nullptr) { 
                        leaves_[leafnum]->updateOctoTree(pc); 
                    } else {
                        leaves_[leafnum] = new VoxelOctoTree(octree_max_depth_, layer_ + 1, node_min_points_, node_max_points_, plane_threshold_);
                        double voxel_center[3] = {voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_, 
                                                    voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_, 
                                                    voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_};
                        leaves_[leafnum]->setVoxelCenter(voxel_center);
                        float quater_length = quater_length_ / 2.0;
                        leaves_[leafnum]->setQuaterLength(quater_length);
                        leaves_[leafnum]->updateOctoTree(pc);
                    }
                } else {
                    if (update_enable_) {
                        new_points_++;
                        temp_points_.push_back(pc);
                        if (new_points_ > update_size_threshold_) {
                            initPlane(temp_points_, plane_ptr_);
                            new_points_ = 0;
                        }
                        if (temp_points_.size() >  static_cast<size_t>(node_max_points_)) {
                            update_enable_ = false;
                            std::vector<PointWithCovarience>().swap(temp_points_);
                            new_points_ = 0;
                        }
                    }
                }
            }
        }
    }

    VoxelOctoTree* VoxelOctoTree::findCorrespondPoint(Eigen::Vector3d pw) {
        if (!init_octo_ || plane_ptr_->is_plane || (layer_ >= octree_max_depth_)) return this;

        int xyz[3] = {0, 0, 0};
        xyz[0] = pw[0] > voxel_center_[0] ? 1 : 0;
        xyz[1] = pw[1] > voxel_center_[1] ? 1 : 0;
        xyz[2] = pw[2] > voxel_center_[2] ? 1 : 0;
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

        // printf("leafnum: %d. \n", leafnum);

        return (leaves_[leafnum] != nullptr) ? leaves_[leafnum]->findCorrespondPoint(pw) : this;
    }

    VoxelOctoTree* VoxelOctoTree::insertPoint(const PointWithCovarience& pc) {
        if ((!init_octo_) || 
            (init_octo_ && plane_ptr_->is_plane) || 
            (init_octo_ && (!plane_ptr_->is_plane) && (layer_ >= octree_max_depth_))) {
            new_points_++;
            temp_points_.push_back(pc);
            return this;
        }

        if (init_octo_ && (!plane_ptr_->is_plane) && (layer_ < octree_max_depth_)) {
            int xyz[3] = {0, 0, 0};
            xyz[0] = pc.global_point[0] > voxel_center_[0] ? 1 : 0;
            xyz[1] = pc.global_point[1] > voxel_center_[1] ? 1 : 0;
            xyz[2] = pc.global_point[2] > voxel_center_[2] ? 1 : 0;
            int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
            if (leaves_[leafnum] != nullptr) { return leaves_[leafnum]->insertPoint(pc); }
            else
            {
                leaves_[leafnum] = new VoxelOctoTree(octree_max_depth_, layer_ + 1, node_min_points_, node_max_points_, plane_threshold_);
                double voxel_center[3] = {voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_, 
                                            voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_, 
                                            voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_};
                leaves_[leafnum]->setVoxelCenter(voxel_center);
                float quater_length = quater_length_ / 2.0;
                leaves_[leafnum]->setQuaterLength(quater_length);
                return leaves_[leafnum]->insertPoint(pc);
            }
        }
        return nullptr;
    }

    void VoxelOctoTree::setVoxelCenter(double voxel_center[3]) {
        voxel_center_[0] = voxel_center[0];
        voxel_center_[1] = voxel_center[1];
        voxel_center_[2] = voxel_center[2];
    }

    void VoxelOctoTree::setQuaterLength(float quater_length) {
        quater_length_ = quater_length;
    }

    void VoxelOctoTree::addTempPoints(PointWithCovarience temp_point) {
        temp_points_.push_back(temp_point);
    }

    void VoxelOctoTree::resetTempPoints() {
        std::vector<PointWithCovarience>().swap(temp_points_);
    }

    void VoxelOctoTree::addNewPoints() {
        new_points_++;
    }
    void VoxelOctoTree::setNewPoints(int num_point) {
        new_points_ = num_point;
    }

    void VoxelOctoTree::setOctoState(int octo_state) {
        octo_state_ = octo_state;
    }

    void VoxelOctoTree::setUpdateEnable(bool update_enable) {
        update_enable_ = update_enable;
    }

    void VoxelOctoTree::setInitOcto(bool init_octo) {
        init_octo_ = init_octo;
    }

    const std::vector<PointWithCovarience>& VoxelOctoTree::getTempPoints() const {
        return temp_points_;
    }

    VoxelPlane* VoxelOctoTree::getPlanePtr() const {
        return plane_ptr_;
    }

    VoxelOctoTree* VoxelOctoTree::getLeaves(int index) const {
        return leaves_[index];
    }

    const double* VoxelOctoTree::getVoxelCenter() const {
        return voxel_center_;
    }

    float VoxelOctoTree::getQuaterLength() const {
        return quater_length_;
    }

    int VoxelOctoTree::getNodeMinPoints() const {
        return node_min_points_;
    }

    int VoxelOctoTree::getNodeMaxPoints() const {
        return node_max_points_;
    }

    VoxelOctoTree::~VoxelOctoTree() {
        for (int i = 0; i < 8; i++) {
            delete leaves_[i];
        }
        delete plane_ptr_;
    }
}