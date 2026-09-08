cmake_minimum_required(VERSION 3.8)
project(fr3_dual_compact_suction_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf srdf config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/dual_joint_state_bridge.py
    scripts/task06_environment_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
