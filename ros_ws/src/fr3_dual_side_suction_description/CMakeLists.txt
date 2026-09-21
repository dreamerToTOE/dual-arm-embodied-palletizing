cmake_minimum_required(VERSION 3.8)
project(fr3_dual_side_suction_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch srdf urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
