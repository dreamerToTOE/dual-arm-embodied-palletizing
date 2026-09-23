cmake_minimum_required(VERSION 3.8)
project(fr3_moveit_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# Dependencies
# ============================================================

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


# ============================================================
# Task 00: MoveIt -> Isaac
# ============================================================

add_executable(
  moveit_to_isaac
  src/moveit_to_isaac.cpp
)

ament_target_dependencies(
  moveit_to_isaac
  rclcpp
  sensor_msgs
  trajectory_msgs
  moveit_core
  moveit_ros_planning_interface
)


# ============================================================
# Task 01: Single-arm Pick & Place
# ============================================================

add_executable(
  single_arm_pick_place
  src/single_arm_pick_place.cpp
)

ament_target_dependencies(
  single_arm_pick_place
  rclcpp
  sensor_msgs
  trajectory_msgs
  geometry_msgs
  shape_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
)


add_executable(
  placement_skill_demo
  src/placement_skill_demo.cpp
)

ament_target_dependencies(
  placement_skill_demo
  rclcpp
  sensor_msgs
  geometry_msgs
  shape_msgs
  trajectory_msgs
  moveit_msgs
  moveit_ros_planning_interface
)


add_executable(
  bac_placement_test
  src/bac_placement_test.cpp
)

ament_target_dependencies(
  bac_placement_test
  rclcpp
  sensor_msgs
  geometry_msgs
  shape_msgs
  trajectory_msgs
  moveit_msgs
  moveit_ros_planning_interface
)


add_executable(
  suction_three_cube_palletize
  src/suction_three_cube_palletize.cpp
)

ament_target_dependencies(
  suction_three_cube_palletize
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  shape_msgs
  trajectory_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_core
)


add_executable(
  suction_nine_cube_palletize
  src/suction_nine_cube_palletize.cpp
)

ament_target_dependencies(
  suction_nine_cube_palletize
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  shape_msgs
  trajectory_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_core
)


add_executable(
  task05_bac_suction_place
  src/task05_bac_suction_place.cpp
)

ament_target_dependencies(
  task05_bac_suction_place
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  shape_msgs
  trajectory_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_core
)


# ============================================================
# Install
# ============================================================

install(
  TARGETS
    moveit_to_isaac
    single_arm_pick_place
    placement_skill_demo
    bac_placement_test
    suction_three_cube_palletize
    suction_nine_cube_palletize
    task05_bac_suction_place
  DESTINATION lib/${PROJECT_NAME}
)


# ============================================================
# Testing
# ============================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
