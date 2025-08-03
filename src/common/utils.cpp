/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "rliv_gs/common/utils.hpp"

namespace common {

    Utils::Utils() {
        // TODO
    }

    Utils::~Utils() {
        // TODO
    }

    void Utils::printColored(const std::string& text, Color color, Style style) {
        // Construct the ANSI escape code
        std::string ansi_code = "\033[" + std::to_string(static_cast<int>(style)) + ";" + std::to_string(static_cast<int>(color)) + "m";
        // Print the colored text
        std::cout << ansi_code << text << "\033[" + std::to_string(static_cast<int>(Style::kNormal)) + "m" << std::endl;
    }
    

    Eigen::Matrix3d Utils::getAntiSymmetricMatrix(const Eigen::Vector3d& vec) {
        Eigen::Matrix3d mat;
        mat << 0.0, -vec(2), vec(1),
            vec(2), 0.0, -vec(0),
            -vec(1), vec(0), 0.0;
        return mat;
    }

    Eigen::Matrix3d Utils::Exp(const Eigen::Vector3d& ang_vel) {
        double ang_norm = ang_vel.norm();
        Eigen::Matrix3d Eye3 = Eigen::Matrix3d::Identity();
        if (ang_norm > 0.0000001)
        {
            Eigen::Vector3d r_axis = ang_vel / ang_norm;
            Eigen::Matrix3d K;
            K << getAntiSymmetricMatrix(r_axis);
            /// Roderigous Tranformation
            return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
        }
        else { return Eye3; }
    }
    
    Eigen::Matrix3d Utils::Exp(const Eigen::Vector3d& ang_vel, const double& dt) {
        
        double ang_vel_norm = ang_vel.norm();
        Eigen::Matrix3d Eye3 = Eigen::Matrix3d::Identity();

        if (ang_vel_norm > 0.0000001)
        {
            Eigen::Vector3d r_axis = ang_vel / ang_vel_norm;
            Eigen::Matrix3d K;

            K = getAntiSymmetricMatrix(r_axis);

            double r_ang = ang_vel_norm * dt;

            /// Roderigous Tranformation
            return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
        }
        else { return Eye3; }
    }

    Eigen::Matrix3d Utils::Exp(const double& v1, const double& v2, const double& v3) {
        double &&norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
        Eigen::Matrix3d Eye3 = Eigen::Matrix3d::Identity();
        if (norm > 0.00001)
        {
            double r_ang[3] = {v1 / norm, v2 / norm, v3 / norm};
            Eigen::Matrix3d K;
            Eigen::Vector3d r_ang_vec(r_ang[0], r_ang[1], r_ang[2]);
            K << getAntiSymmetricMatrix(r_ang_vec);

            /// Roderigous Tranformation
            return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
        }
        else { return Eye3; }
    }

    Eigen::Vector3d Utils::Log(const Eigen::Matrix3d& R) {
        double theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
        Eigen::Vector3d K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
        return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
    }

    Eigen::Vector3d Utils::RotMtoEuler(const Eigen::Matrix3d& rot) {
        double sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
        bool singular = sy < 1e-6;
        double x, y, z;
        if (!singular)
        {
            x = atan2(rot(2, 1), rot(2, 2));
            y = atan2(-rot(2, 0), sy);
            z = atan2(rot(1, 0), rot(0, 0));
        }
        else
        {
            x = atan2(-rot(1, 2), rot(1, 1));
            y = atan2(-rot(2, 0), sy);
            z = 0;
        }
        Eigen::Vector3d ang(x, y, z);
        return ang;
    }

    Sophus::SO3d Utils::Mat3ToSO3(const Eigen::Matrix<double, 3, 3> &m) {
    /// 对R做归一化，防止sophus里的检查不过
        Eigen::Quaterniond q(m.template cast<double>());
        q.normalize();
        return Sophus::SO3d(q);
    }


    double common::Utils::first_lidar_header_time = 0.0;

    std::ofstream common::Utils::fout_imu(DEBUG_FILE_DIR("imu.txt"), std::ios::out);
    std::ofstream common::Utils::fout_pridicted_state(DEBUG_FILE_DIR("pridicted_state.txt"), std::ios::out);
    std::ofstream common::Utils::fout_updated_state(DEBUG_FILE_DIR("updated_state.txt"), std::ios::out);
    std::ofstream common::Utils::fout_dt(DEBUG_FILE_DIR("data2.txt"), std::ios::out);
}