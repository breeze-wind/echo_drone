cmake_minimum_required(VERSION 3.5)
project(sls_circle_controller_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sls_qsf_core REQUIRED)
find_package(std_msgs REQUIRED)

# 当前主控制器节点：接 MAVROS 位姿/状态，调用 sls_qsf_core，输出 attitude setpoint。
add_executable(sls_circle_controller_node_cpp
  src/sls_circle_controller_node.cpp
)

target_compile_options(sls_circle_controller_node_cpp PRIVATE -Wall -Wextra -Wpedantic)

ament_target_dependencies(sls_circle_controller_node_cpp
  # mavros_msgs 提供 State、AttitudeTarget 和 OFFBOARD/ARM 服务类型。
  geometry_msgs
  mavros_msgs
  rclcpp
  sls_qsf_core
  std_msgs
)

install(
  TARGETS sls_circle_controller_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
