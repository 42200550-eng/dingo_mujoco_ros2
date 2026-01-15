#include "dingo_gait_controller_cpp/model/mujoco_kinematics.hpp"

#include <algorithm>
#include <stdexcept>

#include "dingo_gait_controller_cpp/utils/math_utils.hpp"

namespace dingo_gait_controller_cpp {

MujocoKinematics::MujocoKinematics(const std::string &model_xml_path, const std::vector<std::string> &joints)
: joints_(joints) {
  char error[1024] = {0};
  model_ = mj_loadXML(model_xml_path.c_str(), nullptr, error, sizeof(error));
  if (!model_) {
    throw std::runtime_error(std::string("Failed to load MJCF: ") + error);
  }
  data_ = mj_makeData(model_);
  if (!data_) {
    throw std::runtime_error("Failed to allocate mjData");
  }

  has_freejoint_ = (model_->nq >= 7 && model_->nv >= 6);

  for (const auto &jn : joints_) {
    const int jid = mj_name2id(model_, mjOBJ_JOINT, jn.c_str());
    if (jid < 0) {
      throw std::runtime_error("Joint not found in MJCF: " + jn);
    }
    joint_qpos_adr_[jn] = model_->jnt_qposadr[jid];
    joint_dof_adr_[jn] = model_->jnt_dofadr[jid];
  }
}

MujocoKinematics::~MujocoKinematics() {
  if (data_) {
    mj_deleteData(data_);
    data_ = nullptr;
  }
  if (model_) {
    mj_deleteModel(model_);
    model_ = nullptr;
  }
}

void MujocoKinematics::set_q_from_vector(const std::vector<double> &q12) {
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto &jn = joints_[i];
    data_->qpos[joint_qpos_adr_.at(jn)] = q12[i];
  }
}

void MujocoKinematics::forward() { ::mj_forward(model_, data_); }

void MujocoKinematics::sync_base_pose_into_mjdata(const Eigen::Vector3d &base_pos_w, const Eigen::Vector4d &base_quat_wxyz) {
  if (!has_freejoint_) {
    return;
  }
  data_->qpos[0] = base_pos_w.x();
  data_->qpos[1] = base_pos_w.y();
  data_->qpos[2] = base_pos_w.z();
  data_->qpos[3] = base_quat_wxyz[0];
  data_->qpos[4] = base_quat_wxyz[1];
  data_->qpos[5] = base_quat_wxyz[2];
  data_->qpos[6] = base_quat_wxyz[3];
}

std::pair<Eigen::Vector3d, Eigen::Matrix3d> MujocoKinematics::base_pose_world(int base_body_id) const {
  Eigen::Vector3d base_p(
    data_->xpos[3 * base_body_id + 0],
    data_->xpos[3 * base_body_id + 1],
    data_->xpos[3 * base_body_id + 2]);

  Eigen::Matrix3d base_R;
  const mjtNum *m = &data_->xmat[9 * base_body_id];
  base_R << m[0], m[1], m[2],
            m[3], m[4], m[5],
            m[6], m[7], m[8];

  return {base_p, base_R};
}

int MujocoKinematics::body_id_or_throw(const std::string &body_name) const {
  const int bid = mj_name2id(model_, mjOBJ_BODY, body_name.c_str());
  if (bid < 0) {
    throw std::runtime_error("Body not found in MJCF: " + body_name);
  }
  return bid;
}

int MujocoKinematics::joint_qpos_adr(const std::string &jn) const { return joint_qpos_adr_.at(jn); }
int MujocoKinematics::joint_dof_adr(const std::string &jn) const { return joint_dof_adr_.at(jn); }

bool MujocoKinematics::ik_leg_to_target(
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
  double ik_dq_limit_rad) {

  std::array<int, 3> joint_idxs;
  std::array<int, 3> dof_adrs;
  for (int k = 0; k < 3; ++k) {
    const auto &jn = joint_names[k];
    auto it = std::find(joints_.begin(), joints_.end(), jn);
    if (it == joints_.end()) {
      return false;
    }
    joint_idxs[k] = static_cast<int>(std::distance(joints_.begin(), it));
    dof_adrs[k] = joint_dof_adr_.at(jn);
  }

  const int bid = body_id_or_throw(foot_body);

  set_q_from_vector(q12);

  const double lam = std::max(1e-6, ik_lambda);
  const int iters = std::max(1, ik_iters);

  std::vector<mjtNum> jacp(3 * model_->nv);
  std::vector<mjtNum> jacr(3 * model_->nv);

  for (int iter = 0; iter < iters; ++iter) {
    if (have_odom) {
      sync_base_pose_into_mjdata(base_pos_w, base_quat_wxyz);
    }
    forward();

    Eigen::Vector3d cur(
      data_->xpos[3 * bid + 0],
      data_->xpos[3 * bid + 1],
      data_->xpos[3 * bid + 2]);

    Eigen::Vector3d err = target_world - cur;
    if (err.norm() < ik_tol_m) {
      break;
    }

    mj_jacBody(model_, data_, jacp.data(), jacr.data(), bid);

    Eigen::Matrix3d J;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        const int dof = dof_adrs[c];
        J(r, c) = static_cast<double>(jacp[r * model_->nv + dof]);
      }
    }

    const Eigen::Matrix3d A = (J * J.transpose()) + (lam * lam) * Eigen::Matrix3d::Identity();
    const Eigen::Vector3d x = A.ldlt().solve(err);
    Eigen::Vector3d dq = J.transpose() * x;

    for (int k = 0; k < 3; ++k) {
      dq[k] = clamp(dq[k], -ik_dq_limit_rad, ik_dq_limit_rad);
    }

    for (int k = 0; k < 3; ++k) {
      q12[static_cast<size_t>(joint_idxs[k])] += dq[k];
    }

    set_q_from_vector(q12);
  }

  if (have_odom) {
    sync_base_pose_into_mjdata(base_pos_w, base_quat_wxyz);
  }
  forward();

  Eigen::Vector3d cur(
    data_->xpos[3 * bid + 0],
    data_->xpos[3 * bid + 1],
    data_->xpos[3 * bid + 2]);

  const double final_err = (target_world - cur).norm();
  return final_err <= ik_max_err_m;
}

}  // namespace dingo_gait_controller_cpp
