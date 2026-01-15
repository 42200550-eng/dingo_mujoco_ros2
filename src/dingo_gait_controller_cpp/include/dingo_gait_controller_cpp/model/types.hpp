#pragma once

#include <array>
#include <string>

namespace dingo_gait_controller_cpp {

struct LegDef {
  std::string foot_body;
  std::array<std::string, 3> joint_names;
};

}
