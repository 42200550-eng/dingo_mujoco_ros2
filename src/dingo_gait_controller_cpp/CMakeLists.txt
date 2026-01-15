cmake_minimum_required(VERSION 3.8)
project(dingo_gait_controller_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Locate MuJoCo headers/libs from the Python 'mujoco' package (pip/apt).
execute_process(
  COMMAND python3 -c "import mujoco, pathlib; print(pathlib.Path(mujoco.__file__).resolve().parent)"
  OUTPUT_VARIABLE MUJOCO_PY_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(MUJOCO_INCLUDE_DIR "${MUJOCO_PY_DIR}/include")
set(MUJOCO_LIB_DIR "${MUJOCO_PY_DIR}/lib")

if(NOT EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
  message(FATAL_ERROR "MuJoCo headers not found at ${MUJOCO_INCLUDE_DIR}. Install the 'mujoco' python package.")
endif()

# Prefer the typical shared library name; mujoco wheels may store it in different places.
set(MUJOCO_LIB "")
set(MUJOCO_LIB_DIR "")

set(_MUJOCO_LIB_SEARCH_DIRS
  "${MUJOCO_PY_DIR}/lib"
  "${MUJOCO_PY_DIR}"
  "${MUJOCO_PY_DIR}/../mujoco.libs"
)

foreach(_d IN LISTS _MUJOCO_LIB_SEARCH_DIRS)
  if(EXISTS "${_d}/libmujoco.so")
    set(MUJOCO_LIB "${_d}/libmujoco.so")
    set(MUJOCO_LIB_DIR "${_d}")
    break()
  endif()
endforeach()

if(MUJOCO_LIB STREQUAL "")
  foreach(_d IN LISTS _MUJOCO_LIB_SEARCH_DIRS)
    file(GLOB _cands "${_d}/libmujoco.so*")
    list(LENGTH _cands _len)
    if(_len GREATER 0)
      list(GET _cands 0 MUJOCO_LIB)
      set(MUJOCO_LIB_DIR "${_d}")
      break()
    endif()
  endforeach()
endif()

if(MUJOCO_LIB STREQUAL "")
  message(FATAL_ERROR "MuJoCo shared library not found. Searched: ${_MUJOCO_LIB_SEARCH_DIRS}")
endif()

add_executable(dingo_gait_controller_cpp
  src/main.cpp
  src/node/controller_node.cpp
  src/planning/trot_gait.cpp
  src/model/mujoco_kinematics.cpp
)

target_compile_features(dingo_gait_controller_cpp PUBLIC cxx_std_17)

target_include_directories(dingo_gait_controller_cpp PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${MUJOCO_INCLUDE_DIR}
)

# Use plain-signature here because ament_target_dependencies() uses plain-signature too.
target_link_libraries(dingo_gait_controller_cpp
  ${MUJOCO_LIB}
  Eigen3::Eigen
)

# Ensure runtime can find libmujoco from the python package directory.
set_target_properties(dingo_gait_controller_cpp PROPERTIES
  BUILD_RPATH "${MUJOCO_LIB_DIR}"
  INSTALL_RPATH "${MUJOCO_LIB_DIR}"
)

target_compile_definitions(dingo_gait_controller_cpp PRIVATE
  MUJOCO_PY_DIR="${MUJOCO_PY_DIR}"
)

ament_target_dependencies(dingo_gait_controller_cpp
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

install(TARGETS dingo_gait_controller_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
