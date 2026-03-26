cmake_minimum_required(VERSION 3.8)
project(ftc_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Library
add_library(ftc_control_lib
  src/models/auv_dynamics.cpp
  src/controllers/backstepping.cpp
  src/controllers/ftsmc_controller.cpp
  src/controllers/i_ftsmc_controller.cpp
  src/allocation/thruster_allocation.cpp
)

target_include_directories(ftc_control_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

# Nodes
add_executable(trajectory_node src/nodes/trajectory_node.cpp)
ament_target_dependencies(trajectory_node
  rclcpp std_msgs geometry_msgs nav_msgs
)

add_executable(state_estimator_node src/nodes/state_estimator_node.cpp)
ament_target_dependencies(state_estimator_node
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros
)

add_executable(thruster_allocator_node src/nodes/thruster_allocator_node.cpp)
target_link_libraries(thruster_allocator_node ftc_control_lib)
ament_target_dependencies(thruster_allocator_node
  rclcpp std_msgs std_srvs geometry_msgs
)

add_executable(data_logger_node src/nodes/data_logger_node.cpp)
ament_target_dependencies(data_logger_node
  rclcpp std_msgs nav_msgs tf2 geometry_msgs
)

add_executable(controller_node src/nodes/controller_node.cpp)
target_link_libraries(controller_node ftc_control_lib)
ament_target_dependencies(controller_node
  rclcpp std_msgs nav_msgs geometry_msgs
)

# Install
install(TARGETS
  trajectory_node
  state_estimator_node
  controller_node
  thruster_allocator_node
  data_logger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
