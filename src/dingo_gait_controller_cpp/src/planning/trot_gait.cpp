#include "dingo_gait_controller_cpp/planning/trot_gait.hpp"

#include <cmath>

#include "dingo_gait_controller_cpp/utils/math_utils.hpp"

namespace dingo_gait_controller_cpp {

TrotGait::TrotGait(
  double gait_period_s,
  double duty_factor,
  double swing_height_m,
  double step_length_max_m,
  double step_width_max_m,
  double max_vx_m_s,
  double max_vy_m_s,
  double max_wz_rad_s,
  double yaw_step_max_rad,
  std::array<double, 4> phase_offsets)
: gait_period_s_(gait_period_s),
  duty_(duty_factor),
  swing_height_(swing_height_m),
  step_length_max_(step_length_max_m),
  step_width_max_(step_width_max_m),
  max_vx_(max_vx_m_s),
  max_vy_(max_vy_m_s),
  max_wz_(max_wz_rad_s),
  yaw_step_max_(yaw_step_max_rad),
  phase_offsets_(phase_offsets),
  t0_(std::chrono::steady_clock::now()) {}

double TrotGait::phase() const {
  const auto now = std::chrono::steady_clock::now();
  const double t = std::chrono::duration<double>(now - t0_).count();
  const double T = std::max(0.1, gait_period_s_);
  return std::fmod(t, T) / T;
}

Eigen::Vector3d TrotGait::foot_target_base(
  int leg_index,
  const Eigen::Vector3d &nominal_foot_base,
  double vx,
  double vy,
  double wz) const {

  Eigen::Vector3d p0 = nominal_foot_base;

  vx = clamp(vx, -max_vx_, max_vx_);
  vy = clamp(vy, -max_vy_, max_vy_);
  wz = clamp(wz, -max_wz_, max_wz_);

  const double ax = (max_vx_ <= 1e-6) ? 0.0 : vx / max_vx_;
  const double ay = (max_vy_ <= 1e-6) ? 0.0 : vy / max_vy_;
  const double aw = (max_wz_ <= 1e-6) ? 0.0 : wz / max_wz_;

  const double step_x = ax * step_length_max_;
  const double step_y = ay * step_width_max_;
  const double yaw_step = aw * yaw_step_max_;

  const Eigen::Vector3d d_rot = (rotz(yaw_step) * p0) - p0;
  const Eigen::Vector3d step_vec(step_x, step_y, 0.0);
  const Eigen::Vector3d step = step_vec + d_rot;

  const double ph = std::fmod(phase() + phase_offsets_.at(static_cast<size_t>(leg_index)), 1.0);
  const double beta = clamp(duty_, 0.2, 0.9);

  Eigen::Vector3d d(0, 0, 0);
  double dz = 0.0;
  if (ph < beta) {
    const double s = ph / beta;
    d = (+0.5 * step) - (step * s);
    dz = 0.0;
  } else {
    const double s = (ph - beta) / (1.0 - beta);
    d = (-0.5 * step) + (step * s);
    dz = swing_height_ * std::sin(M_PI * s);
  }

  Eigen::Vector3d p = p0 + d;
  p.z() += dz;
  return p;
}

}
