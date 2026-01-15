#pragma once

#include <array>
#include <chrono>

#include <Eigen/Dense>

#include "dingo_gait_controller_cpp/utils/math_utils.hpp"

namespace dingo_gait_controller_cpp {

class TrotGait {
public:
  TrotGait(
    double gait_period_s,
    double duty_factor,
    double swing_height_m,
    double step_length_max_m,
    double step_width_max_m,
    double max_vx_m_s,
    double max_vy_m_s,
    double max_wz_rad_s,
    double yaw_step_max_rad,
    std::array<double, 4> phase_offsets);

  Eigen::Vector3d foot_target_base(
    int leg_index,
    const Eigen::Vector3d &nominal_foot_base,
    double vx,
    double vy,
    double wz) const;

private:
  double phase() const;

  double gait_period_s_{0.8};
  double duty_{0.65};
  double swing_height_{0.035};
  double step_length_max_{0.08};
  double step_width_max_{0.05};
  double max_vx_{0.20};
  double max_vy_{0.12};
  double max_wz_{0.6};
  double yaw_step_max_{0.25};
  std::array<double, 4> phase_offsets_{};

  std::chrono::steady_clock::time_point t0_;
};

}
