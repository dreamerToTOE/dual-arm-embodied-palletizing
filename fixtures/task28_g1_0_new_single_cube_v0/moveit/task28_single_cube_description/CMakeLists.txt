cmake_minimum_required(VERSION 3.8)
project(task28_single_cube_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes srdf urdf DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/render_robot_description.py
  scripts/task28_joint_state_bridge.py
  scripts/task28_environment_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
