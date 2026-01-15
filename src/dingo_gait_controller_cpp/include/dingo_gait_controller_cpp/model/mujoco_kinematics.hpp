#pragma once

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include <mujoco/mujoco.h>

namespace dingo_gait_controller_cpp {

class MujocoKinematics {
public:
  MujocoKinematics(const std::string &model_xml_path, const std::vector<std::string> &joints);
  ~MujocoKinematics();

  MujocoKinematics(const MujocoKinematics &) = delete;
  MujocoKinematics &operator=(const MujocoKinematics &) = delete;

  bool has_freejoint() const { return has_freejoint_; }

  void set_q_from_vector(const std::vector<double> &q12);
  void forward();

  void sync_base_pose_into_mjdata(const Eigen::Vector3d &base_pos_w, const Eigen::Vector4d &base_quat_wxyz);

  std::pair<Eigen::Vector3d, Eigen::Matrix3d> base_pose_world(int base_body_id) const;

  int body_id_or_throw(const std::string &body_name) const;

  bool ik_leg_to_target(
    std::vector<double> &q12,
    const std::array<std::string, 3> &joint_names,
    const std::string &foot_body,
    const Eigen::Vector3d &target_world,
    bool have_odom,
    const Eigen::Vector3d &base_pos_w,
    const Eigen::Vector4d &base_quat_wxyz,
    int ik_iters,
    double ik_lambda,
    double ik_tol_m,
    double ik_max_err_m,
    double ik_dq_limit_rad);

  mjModel *model() const { return model_; }
  mjData *data() const { return data_; }

  const std::vector<std::string> &joints() const { return joints_; }

  int joint_qpos_adr(const std::string &jn) const;
  int joint_dof_adr(const std::string &jn) const;

private:
  mjModel *model_{nullptr};
  mjData *data_{nullptr};
  bool has_freejoint_{false};

  std::vector<std::string> joints_;
  std::unordered_map<std::string, int> joint_qpos_adr_;
  std::unordered_map<std::string, int> joint_dof_adr_;
  std::unordered_map<std::string, int> body_id_;
};

}
