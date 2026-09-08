cmake_minimum_required(VERSION 3.8)
project(fr3_dual_palletize)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_library(palletize_primitive
  src/palletize_primitive.cpp
)
target_include_directories(palletize_primitive
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(palletize_primitive
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  shape_msgs
  trajectory_msgs
  moveit_msgs
  moveit_ros_planning_interface
)

add_executable(task07_parallel_demo
  src/task07_parallel_demo.cpp
)
target_link_libraries(task07_parallel_demo palletize_primitive)
ament_target_dependencies(task07_parallel_demo
  rclcpp
  geometry_msgs
  shape_msgs
  moveit_msgs
  moveit_ros_planning_interface
)

install(
  TARGETS palletize_primitive task07_parallel_demo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  shape_msgs
  trajectory_msgs
  moveit_msgs
  moveit_ros_planning_interface
)

ament_package()
