/* 
This file is part of RLIV-GS: Robust Lidar-Inertial-Vision-based Gaussian Splatter.

Developer: Feng Pan <fengpan97618@163.com>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef UTILS_HPP
#define UTILS_HPP

#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <fstream>

#include <vikit/math_utils.h>

#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define DEBUG_FILE_DIR(name) \
  (std::string(std::string(ROOT_DIR) + "log/" + name))


namespace common {

// Color codes for terminal output
enum class Color {
    kReset = 0,     // Reset all styles
    kBlack = 30,    // Black text
    kRed = 31,      // Red text
    kGreen = 32,    // Green text
    kYellow = 33,   // Yellow text
    kBlue = 34,     // Blue text
    kMagenta = 35,  // Magenta text
    kCyan = 36,     // Cyan text
    kWhite = 37,    // White text
    
    // Bright color variants
    kBrightBlack = 90,
    kBrightRed = 91,
    kBrightGreen = 92,
    kBrightYellow = 93,
    kBrightBlue = 94,
    kBrightMagenta = 95,
    kBrightCyan = 96,
    kBrightWhite = 97
};

// ANSI style codes with kPrefix and UpperCamelCase
enum class Style {
    kNormal = 0,    // Normal text style
    kBold = 1,      // Bold text
    kUnderline = 4, // Underlined text
    kInvert = 7     // Inverted colors
};



class Utils {
    
public:
    Utils();
    ~Utils();
    static void printColored(const std::string& text, Color color, Style style);
    // Get an antisymmetric matrix
    static Eigen::Matrix3d getAntiSymmetricMatrix(const Eigen::Vector3d& vec);


    static Eigen::Matrix3d Exp(const Eigen::Vector3d& ang_vel);
    static Eigen::Matrix3d Exp(const double& v1, const double& v2, const double& v3);
    static Eigen::Matrix3d Exp(const Eigen::Vector3d& ang_vel, const double& dt);

    static Eigen::Vector3d Log(const Eigen::Matrix3d& R);

    static Eigen::Vector3d RotMtoEuler(const Eigen::Matrix3d& rot);

    static Sophus::SO3d Mat3ToSO3(const Eigen::Matrix<double, 3, 3> &m);
    
    // fout_imu is used to save the imu data

    static std::ofstream fout_imu;
    static std::ofstream fout_pridicted_state;
    static std::ofstream fout_updated_state;
    static std::ofstream fout_dt;


    static double first_lidar_header_time;
};

}

#endif // UTILS_HPP