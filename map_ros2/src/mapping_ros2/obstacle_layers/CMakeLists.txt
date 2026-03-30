cmake_minimum_required(VERSION 3.8)
project(obstacle_layers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(obstacle_layers_node src/obstacle_layers_node.cpp)

target_compile_features(obstacle_layers_node PUBLIC cxx_std_17)

ament_target_dependencies(
  obstacle_layers_node
  rclcpp
  grid_map_core
  grid_map_ros
  grid_map_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
)

install(TARGETS obstacle_layers_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
