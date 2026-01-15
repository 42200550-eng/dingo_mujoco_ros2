#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

namespace dingo_gait_controller_cpp {

inline double clamp(double x, double lo, double hi) { return std::min(std::max(x, lo), hi); }

inline Eigen::Matrix3d quat_to_rot_wxyz(const Eigen::Vector4d &q_wxyz) {
  double w = q_wxyz[0];
  double x = q_wxyz[1];
  double y = q_wxyz[2];
  double z = q_wxyz[3];
  double n = std::sqrt(w*w + x*x + y*y + z*z);
  if (n < 1e-9) {
    return Eigen::Matrix3d::Identity();
  }
  w /= n; x /= n; y /= n; z /= n;
  Eigen::Matrix3d R;
  R <<
    1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w),
    2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w),
    2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y);
  return R;
}

inline Eigen::Matrix3d rotz(double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  R(0,0) = c;  R(0,1) = -s;
  R(1,0) = s;  R(1,1) = c;
  return R;
}

}
