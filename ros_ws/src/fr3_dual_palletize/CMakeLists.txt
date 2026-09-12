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
find_package(ompl REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Task18：Isaac Ground Truth 与未来感知模块共享的运行时输入接口。
# 自定义消息只表达观测结果；它不绑定某个 Task 的固定物体数量或坐标。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoxState.msg"
  "msg/BoxStateArray.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

# Task17：配置驱动的 BoxSpec / PlacementSpec 数据模型。它不依赖某个 Task 的
# 执行器，Task18 随机场景、Task19 放置规划和 Task21 路由可直接复用。
add_library(palletizing_job
  src/palletizing_job_loader.cpp
)
target_include_directories(palletizing_job
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(palletizing_job yaml-cpp)
ament_target_dependencies(palletizing_job
  geometry_msgs
  shape_msgs
  moveit_msgs
)

# Task18：把运行时 BoxState 转换为 Task17 BoxSpec，并以世界 pose 组合局部
# grasp candidate。Task19+ 的控制器可直接复用，而不再读取硬编码初始坐标。
add_library(runtime_box_state
  src/runtime_box_state.cpp
)
target_include_directories(runtime_box_state
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(runtime_box_state palletizing_job)
ament_target_dependencies(runtime_box_state
  geometry_msgs
)
rosidl_target_interfaces(runtime_box_state
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Task19：可解释的几何 / height-map placement planner。它只产生 PlacementSpec；
# Task16 RobustPlanner 仍独立决定机器人如何到达该目标。
add_library(placement_planner
  src/placement_planner.cpp
)
target_include_directories(placement_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(placement_planner palletizing_job)
ament_target_dependencies(placement_planner geometry_msgs)

add_library(palletize_primitive
  src/palletize_primitive.cpp
  src/robust_planner.cpp
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

# Task15 Phase A：以编译目标复用 Task11--Task13 的紧协调共同物体流程。
# 不复制状态机；宏只切换 Task15 的 ROS namespace / object id / node name。
add_executable(task15_tight_large_cube
  src/task11_shared_box_grasp.cpp
)
target_compile_definitions(
  task15_tight_large_cube
  PRIVATE
    TASK12_SHARED_LIFT=1
    TASK12_SHARED_TRANSPORT=1
    TASK13_SHARED_PLACE=1
    TASK15_TIGHT_PHASE=1
)
ament_target_dependencies(task15_tight_large_cube
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

# Task15 Phase B：两批次配置驱动的小 Cube 松协调控制器。
add_executable(task15_loose_layer_stack
  src/task15_loose_layer_stack.cpp
)
target_link_libraries(task15_loose_layer_stack palletize_primitive)
ament_target_dependencies(task15_loose_layer_stack
  rclcpp
  geometry_msgs
  shape_msgs
  trajectory_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
)

# Task16-D：只读规划 benchmark。它复用 Task15 下层任务几何、Task08 FCL 和
# Task09 LocalWait，但不连接 Isaac 的执行/吸盘话题。
add_executable(task16_planning_benchmark
  src/task16_planning_benchmark.cpp
)

add_executable(task17_job_validate
  src/task17_job_validate.cpp
)
target_link_libraries(task17_job_validate palletizing_job)
ament_target_dependencies(task17_job_validate
  rclcpp
  geometry_msgs
  shape_msgs
  moveit_msgs
)

add_executable(task18_runtime_input_demo
  src/task18_runtime_input_demo.cpp
)
target_link_libraries(task18_runtime_input_demo
  runtime_box_state
  palletizing_job
)
ament_target_dependencies(task18_runtime_input_demo
  rclcpp
  geometry_msgs
  shape_msgs
  moveit_msgs
)
rosidl_target_interfaces(task18_runtime_input_demo
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_executable(task19_placement_planner_demo
  src/task19_placement_planner_demo.cpp
)
target_link_libraries(task19_placement_planner_demo placement_planner palletizing_job)
ament_target_dependencies(task19_placement_planner_demo rclcpp geometry_msgs)
target_link_libraries(task16_planning_benchmark
  palletize_primitive
  ompl::ompl
)
ament_target_dependencies(task16_planning_benchmark
  rclcpp
  sensor_msgs
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
    palletizing_job
    runtime_box_state
    placement_planner
    task07_parallel_demo
    task08_candidate_demo
    task10_continuous_demo
    task11_shared_box_grasp
    task12_shared_box_lift
    task12_shared_box_transport
    task13_shared_box_place
    task15_tight_large_cube
    task15_loose_layer_stack
    task16_planning_benchmark
    task17_job_validate
    task18_runtime_input_demo
    task19_placement_planner_demo
    task14_shared_box_geometry_monitor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# 项目级协调 skill 是可安装的复用契约：控制器根据物体类型选择已验收的
# 松协调或紧协调流程，而不是为新任务复制一套抓放逻辑。
install(
  DIRECTORY skills/
  DESTINATION share/${PROJECT_NAME}/skills
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
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
  yaml-cpp
  rosidl_default_runtime
)

ament_package()
