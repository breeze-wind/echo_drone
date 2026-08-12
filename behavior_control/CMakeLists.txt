cmake_minimum_required(VERSION 3.8)
project(behavior_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

include_directories(
        include
)

add_library(mission_runtime
  src/mission_runtime.cpp
  src/mission_graph.cpp)
target_include_directories(mission_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(behavior_control_node src/behavior_control_node.cpp src/behavior_control.cpp)
target_link_libraries(behavior_control_node mission_runtime)
ament_target_dependencies(behavior_control_node
        rclcpp
        geometry_msgs
        std_msgs
        std_srvs
        tf2_ros
        tf2
        tf2_eigen
        tf2_geometry_msgs
        nav2_msgs
        robot_interfaces
)

install(TARGETS behavior_control_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_mission_runtime test/test_mission_runtime.cpp)
  if(TARGET test_mission_runtime)
    target_link_libraries(test_mission_runtime mission_runtime)
  endif()

  ament_add_gtest(test_mission_graph test/test_mission_graph.cpp)
  if(TARGET test_mission_graph)
    target_link_libraries(test_mission_graph mission_runtime)
  endif()
endif()

ament_package()
