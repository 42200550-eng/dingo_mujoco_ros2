#include "dingo_gait_controller_cpp/node/controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "dingo_gait_controller_cpp/utils/math_utils.hpp"

namespace dingo_gait_controller_cpp {

static std::array<double, 4> get_phase_offsets_or_throw(rclcpp::Node *node) {
  const auto v = node->get_parameter("phase_offsets").as_double_array();
  if (v.size() != 4) {
    throw std::runtime_error("phase_offsets must have length 4");
  }
  return {v[0], v[1], v[2], v[3]};
}

static std::vector<LegDef> get_legs_or_throw(rclcpp::Node *node) {
  const std::array<std::string, 4> leg_names = {"FL", "FR", "RL", "RR"};
  std::vector<LegDef> legs;
  legs.reserve(4);

  for (const auto &ln : leg_names) {
    const std::string foot_key = "legs." + ln + ".foot_body";
    const std::string joints_key = "legs." + ln + ".joints";

    const std::string foot = node->get_parameter(foot_key).as_string();
    const auto joints = node->get_parameter(joints_key).as_string_array();
    if (joints.size() != 3) {
      throw std::runtime_error(joints_key + " must have length 3");
    }

    legs.push_back(LegDef{foot, {joints[0], joints[1], joints[2]}});
  }

  return legs;
}

DingoGaitControllerNode::DingoGaitControllerNode() : rclcpp::Node("dingo_gait_controller_cpp") {
  declare_parameter<std::string>("model_xml_path", "");
  declare_parameter<std::string>("cmd_topic", "/joint_group_position_controller/commands");
  declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  declare_parameter<std::string>("joint_states_topic", "/joint_states");
  declare_parameter<std::string>("odom_topic", "/odom");

  declare_parameter<double>("rate_hz", 100.0);

  // Robot config
  declare_parameter<std::string>("model_base_body", "base_link");

  declare_parameter<std::vector<std::string>>(
    "joints",
    {
      "FL_theta1", "FL_theta2", "FL_theta3",
      "FR_theta1", "FR_theta2", "FR_theta3",
      "RL_theta1", "RL_theta2", "RL_theta3",
      "RR_theta1", "RR_theta2", "RR_theta3",
    }
  );

  declare_parameter<std::vector<double>>(
    "stand_joint_positions",
    {
      0.000999, 0.719617, 0.000339,
      0.001002, 0.719615, 0.000339,
      0.000999, 0.719617, 0.000339,
      0.001002, 0.719615, 0.000339,
    }
  );

  declare_parameter<std::vector<double>>("phase_offsets", {0.0, 0.5, 0.5, 0.0});

  // Legs (from YAML: legs.FL.foot_body, legs.FL.joints, ...)
  declare_parameter<std::string>("legs.FL.foot_body", "FL_link4");
  declare_parameter<std::vector<std::string>>("legs.FL.joints", {"FL_theta1", "FL_theta2", "FL_theta3"});

  declare_parameter<std::string>("legs.FR.foot_body", "FR_link4");
  declare_parameter<std::vector<std::string>>("legs.FR.joints", {"FR_theta1", "FR_theta2", "FR_theta3"});

  declare_parameter<std::string>("legs.RL.foot_body", "RL_link4");
  declare_parameter<std::vector<std::string>>("legs.RL.joints", {"RL_theta1", "RL_theta2", "RL_theta3"});

  declare_parameter<std::string>("legs.RR.foot_body", "RR_link4");
  declare_parameter<std::vector<std::string>>("legs.RR.joints", {"RR_theta1", "RR_theta2", "RR_theta3"});

  // Gait
  declare_parameter<double>("gait_period_s", 0.8);
  declare_parameter<double>("duty_factor", 0.65);
  declare_parameter<double>("swing_height_m", 0.035);
  declare_parameter<double>("step_length_max_m", 0.08);
  declare_parameter<double>("step_width_max_m", 0.05);
  declare_parameter<double>("max_vx_m_s", 0.20);
  declare_parameter<double>("max_vy_m_s", 0.12);
  declare_parameter<double>("max_wz_rad_s", 0.6);
  declare_parameter<double>("yaw_step_max_rad", 0.25);

  // IK
  declare_parameter<int>("ik_iters", 15);
  declare_parameter<double>("ik_lambda", 0.02);
  declare_parameter<double>("ik_tol_m", 1e-4);
  declare_parameter<double>("ik_max_err_m", 0.015);
  declare_parameter<double>("ik_dq_limit_rad", 0.12);

  // Stand vs walk
  declare_parameter<double>("vel_deadband", 0.01);

  model_xml_path_ = get_parameter("model_xml_path").as_string();
  if (model_xml_path_.empty()) {
    throw std::runtime_error("model_xml_path is required");
  }

  cmd_topic_ = get_parameter("cmd_topic").as_string();
  cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
  joint_states_topic_ = get_parameter("joint_states_topic").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();

  model_base_body_ = get_parameter("model_base_body").as_string();

  rate_hz_ = get_parameter("rate_hz").as_double();
  dt_ = 1.0 / std::max(10.0, rate_hz_);

  vel_deadband_ = get_parameter("vel_deadband").as_double();

  joints_ = get_parameter("joints").as_string_array();
  if (joints_.size() != 12) {
    throw std::runtime_error("joints must have length 12");
  }

  const auto stand = get_parameter("stand_joint_positions").as_double_array();
  if (stand.size() != 12) {
    throw std::runtime_error("stand_joint_positions must have length 12");
  }
  stand_q_.resize(12);
  for (size_t i = 0; i < 12; ++i) {
    stand_q_[i] = stand[i];
  }
  last_q_cmd_ = stand_q_;

  phase_offsets_ = get_phase_offsets_or_throw(this);
  legs_ = get_legs_or_throw(this);

  const double gait_period_s = get_parameter("gait_period_s").as_double();
  const double duty = get_parameter("duty_factor").as_double();
  const double swing_height = get_parameter("swing_height_m").as_double();
  const double step_length_max = get_parameter("step_length_max_m").as_double();
  const double step_width_max = get_parameter("step_width_max_m").as_double();
  const double max_vx = get_parameter("max_vx_m_s").as_double();
  const double max_vy = get_parameter("max_vy_m_s").as_double();
  const double max_wz = get_parameter("max_wz_rad_s").as_double();
  const double yaw_step_max = get_parameter("yaw_step_max_rad").as_double();

  ik_iters_ = get_parameter("ik_iters").as_int();
  ik_lambda_ = get_parameter("ik_lambda").as_double();
  ik_tol_ = get_parameter("ik_tol_m").as_double();
  ik_max_err_ = get_parameter("ik_max_err_m").as_double();
  ik_dq_limit_ = get_parameter("ik_dq_limit_rad").as_double();

  kin_ = std::make_unique<MujocoKinematics>(model_xml_path_, joints_);
  base_body_id_ = kin_->body_id_or_throw(model_base_body_);

  gait_ = std::make_unique<TrotGait>(
    gait_period_s,
    duty,
    swing_height,
    step_length_max,
    step_width_max,
    max_vx,
    max_vy,
    max_wz,
    yaw_step_max,
    phase_offsets_);

  compute_nominal_feet_from_stand();

  cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(cmd_topic_, 10);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, qos, std::bind(&DingoGaitControllerNode::on_cmd_vel, this, std::placeholders::_1));
  js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_, qos, std::bind(&DingoGaitControllerNode::on_joint_states, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, qos, std::bind(&DingoGaitControllerNode::on_odom, this, std::placeholders::_1));

  timer_ = create_wall_timer(std::chrono::duration<double>(dt_), std::bind(&DingoGaitControllerNode::tick, this));

  RCLCPP_INFO(
    get_logger(),
    "dingo_gait_controller_cpp ready: rate=%.1fHz cmd_vel=%s -> %s base_body=%s",
    rate_hz_,
    cmd_vel_topic_.c_str(),
    cmd_topic_.c_str(),
    model_base_body_.c_str());
}

void DingoGaitControllerNode::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mu_);
  last_cmd_vel_ = *msg;
}

void DingoGaitControllerNode::on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mu_);
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    js_positions_[msg->name[i]] = msg->position[i];
  }
  have_joint_states_ = true;
}

void DingoGaitControllerNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mu_);
  base_pos_w_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  base_quat_wxyz_ = Eigen::Vector4d(
    msg->pose.pose.orientation.w,
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z);
  have_odom_ = true;
}

std::vector<double> DingoGaitControllerNode::get_q_vector_from_joint_states() {
  std::lock_guard<std::mutex> lk(mu_);
  if (!have_joint_states_) {
    return stand_q_;
  }
  auto out = stand_q_;
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto &jn = joints_[i];
    auto it = js_positions_.find(jn);
    if (it != js_positions_.end()) {
      out[i] = it->second;
    }
  }
  return out;
}

void DingoGaitControllerNode::compute_nominal_feet_from_stand() {
  kin_->set_q_from_vector(stand_q_);
  kin_->forward();

  const auto [base_p, base_R] = kin_->base_pose_world(base_body_id_);

  nominal_foot_world_.clear();
  nominal_foot_base_.clear();

  for (const auto &leg : legs_) {
    const int bid = kin_->body_id_or_throw(leg.foot_body);
    Eigen::Vector3d p(
      kin_->data()->xpos[3 * bid + 0],
      kin_->data()->xpos[3 * bid + 1],
      kin_->data()->xpos[3 * bid + 2]);
    nominal_foot_world_.push_back(p);
    nominal_foot_base_.push_back(base_R.transpose() * (p - base_p));
  }
}

Eigen::Vector3d DingoGaitControllerNode::foot_target_world(
  int leg_index,
  double vx,
  double vy,
  double wz,
  bool have_odom,
  const Eigen::Vector3d &base_pos,
  const Eigen::Vector4d &base_quat_wxyz) {

  const Eigen::Vector3d p_des_b = gait_->foot_target_base(
    leg_index,
    nominal_foot_base_.at(static_cast<size_t>(leg_index)),
    vx,
    vy,
    wz);

  if (have_odom) {
    const Eigen::Matrix3d R_wb = quat_to_rot_wxyz(base_quat_wxyz);
    return base_pos + (R_wb * p_des_b);
  }

  const Eigen::Vector3d delta = p_des_b - nominal_foot_base_.at(static_cast<size_t>(leg_index));
  Eigen::Vector3d p_w = nominal_foot_world_.at(static_cast<size_t>(leg_index));
  p_w += delta;
  return p_w;
}

void DingoGaitControllerNode::tick() {
  geometry_msgs::msg::Twist cmd_vel_local;
  bool have_odom_local = false;
  Eigen::Vector3d base_pos_local(0, 0, 0);
  Eigen::Vector4d base_quat_local(1, 0, 0, 0);

  {
    std::lock_guard<std::mutex> lk(mu_);
    cmd_vel_local = last_cmd_vel_;
    have_odom_local = have_odom_;
    base_pos_local = base_pos_w_;
    base_quat_local = base_quat_wxyz_;
  }

  const double vx = cmd_vel_local.linear.x;
  const double vy = cmd_vel_local.linear.y;
  const double wz = cmd_vel_local.angular.z;

  const double dead = std::max(0.0, vel_deadband_);

  std::vector<double> q_cmd;
  if (std::abs(vx) <= dead && std::abs(vy) <= dead && std::abs(wz) <= dead) {
    q_cmd = stand_q_;
  } else {
    q_cmd = get_q_vector_from_joint_states();

    for (size_t leg_i = 0; leg_i < legs_.size(); ++leg_i) {
      const Eigen::Vector3d target = foot_target_world(
        static_cast<int>(leg_i),
        vx,
        vy,
        wz,
        have_odom_local,
        base_pos_local,
        base_quat_local);

      const bool ok = kin_->ik_leg_to_target(
        q_cmd,
        legs_[leg_i].joint_names,
        legs_[leg_i].foot_body,
        target,
        have_odom_local,
        base_pos_local,
        base_quat_local,
        ik_iters_,
        ik_lambda_,
        ik_tol_,
        ik_max_err_,
        ik_dq_limit_);

      if (!ok) {
        q_cmd = last_q_cmd_;
        break;
      }
    }
  }

  last_q_cmd_ = q_cmd;

  std_msgs::msg::Float64MultiArray msg;
  msg.data = q_cmd;
  cmd_pub_->publish(msg);
}

}
