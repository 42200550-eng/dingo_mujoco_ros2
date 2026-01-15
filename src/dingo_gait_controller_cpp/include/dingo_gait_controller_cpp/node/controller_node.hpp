#pragma once

#include <array>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "dingo_gait_controller_cpp/model/mujoco_kinematics.hpp"
#include "dingo_gait_controller_cpp/model/types.hpp"
#include "dingo_gait_controller_cpp/planning/trot_gait.hpp"

namespace dingo_gait_controller_cpp {

class DingoGaitControllerNode : public rclcpp::Node {
public:
  DingoGaitControllerNode();

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg);
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);

  std::vector<double> get_q_vector_from_joint_states();
  void compute_nominal_feet_from_stand();

  Eigen::Vector3d foot_target_world(
    int leg_index,
    double vx,
    double vy,
    double wz,
    bool have_odom,
    const Eigen::Vector3d &base_pos,
    const Eigen::Vector4d &base_quat_wxyz);

  void tick();

private:
  // Params
  std::string model_xml_path_;
  std::string cmd_topic_;
  std::string cmd_vel_topic_;
  std::string joint_states_topic_;
  std::string odom_topic_;

  std::string model_base_body_{"base_link"};

  double rate_hz_{100.0};
  double dt_{0.01};

  int ik_iters_{15};
  double ik_lambda_{0.02};
  double ik_tol_{1e-4};
  double ik_max_err_{0.015};
  double ik_dq_limit_{0.12};

  double vel_deadband_{0.01};

  std::vector<std::string> joints_;
  std::vector<double> stand_q_;
  std::vector<double> last_q_cmd_;

  // Kinematics + gait
  std::unique_ptr<MujocoKinematics> kin_;
  std::unique_ptr<TrotGait> gait_;
  std::vector<LegDef> legs_;
  std::array<double, 4> phase_offsets_{};

  std::vector<Eigen::Vector3d> nominal_foot_world_;
  std::vector<Eigen::Vector3d> nominal_foot_base_;

  int base_body_id_{-1};

  // State
  std::mutex mu_;
  geometry_msgs::msg::Twist last_cmd_vel_;
  bool have_joint_states_{false};
  std::unordered_map<std::string, double> js_positions_;

  bool have_odom_{false};
  Eigen::Vector3d base_pos_w_{0, 0, 0};
  Eigen::Vector4d base_quat_wxyz_{1, 0, 0, 0};

  // ROS
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}
