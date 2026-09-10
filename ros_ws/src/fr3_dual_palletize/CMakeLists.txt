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
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_library(palletize_primitive
  src/palletize_primitive.cpp
  src/spatiotemporal_conflict_detector.cpp
  src/temporal_coordinator.cpp
  src/local_wait_coordinator.cpp
  src/coordinated_task_executor.cpp
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
  moveit_core
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

add_executable(task08_candidate_demo
  src/task08_candidate_demo.cpp
)
target_link_libraries(task08_candidate_demo palletize_primitive)
ament_target_dependencies(task08_candidate_demo
  rclcpp
  geometry_msgs
  shape_msgs
  trajectory_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
)

add_executable(task10_continuous_demo
  src/task10_continuous_demo.cpp
)
target_link_libraries(task10_continuous_demo palletize_primitive)
ament_target_dependencies(task10_continuous_demo
  rclcpp
  geometry_msgs
  shape_msgs
  trajectory_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
)

add_executable(task11_shared_box_grasp
  src/task11_shared_box_grasp.cpp
)
ament_target_dependencies(task11_shared_box_grasp
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  shape_msgs
  trajectory_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
)

add_executable(task12_shared_box_lift
  src/task11_shared_box_grasp.cpp
)
target_compile_definitions(task12_shared_box_lift PRIVATE TASK12_SHARED_LIFT=1)
ament_target_dependencies(task12_shared_box_lift
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  shape_msgs
  trajectory_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
)

add_executable(task12_shared_box_transport
  src/task11_shared_box_grasp.cpp
)
target_compile_definitions(
  task12_shared_box_transport
  PRIVATE TASK12_SHARED_LIFT=1 TASK12_SHARED_TRANSPORT=1
)
ament_target_dependencies(task12_shared_box_transport
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  shape_msgs
  trajectory_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
)

add_executable(task13_shared_box_place
  src/task11_shared_box_grasp.cpp
)
target_compile_definitions(
  task13_shared_box_place
  PRIVATE TASK12_SHARED_LIFT=1 TASK12_SHARED_TRANSPORT=1 TASK13_SHARED_PLACE=1
)
ament_target_dependencies(task13_shared_box_place
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  shape_msgs
  trajectory_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
)

add_executable(task14_shared_box_geometry_monitor
  src/task14_shared_box_geometry_monitor.cpp
)
ament_target_dependencies(task14_shared_box_geometry_monitor
  rclcpp
  std_msgs
  geometry_msgs
)

install(
  TARGETS
    palletize_primitive
    task07_parallel_demo
    task08_candidate_demo
    task10_continuous_demo
    task11_shared_box_grasp
    task12_shared_box_lift
    task12_shared_box_transport
    task13_shared_box_place
    task14_shared_box_geometry_monitor
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
  moveit_core
  moveit_ros_planning_interface
)

ament_package()
