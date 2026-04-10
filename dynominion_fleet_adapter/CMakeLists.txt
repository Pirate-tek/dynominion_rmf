cmake_minimum_required(VERSION 3.8)
project(dynominion_fleet_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rmf_utils REQUIRED)
find_package(rmf_traffic REQUIRED)
find_package(rmf_traffic_ros2 REQUIRED)
find_package(rmf_fleet_adapter REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_executable(fleet_adapter_node
  src/FleetAdapter.cpp
  src/Nav2RobotHandle.cpp
)

target_link_libraries(fleet_adapter_node
  yaml-cpp
  Eigen3::Eigen
  rmf_fleet_adapter::rmf_fleet_adapter
  rmf_traffic::rmf_traffic
  rmf_traffic_ros2::rmf_traffic_ros2
  rmf_utils::rmf_utils
)

ament_target_dependencies(fleet_adapter_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  fleet_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
