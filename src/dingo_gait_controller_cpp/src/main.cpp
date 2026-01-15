#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "dingo_gait_controller_cpp/node/controller_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dingo_gait_controller_cpp::DingoGaitControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
