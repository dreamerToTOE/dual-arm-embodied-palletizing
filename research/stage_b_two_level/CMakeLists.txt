cmake_minimum_required(VERSION 3.8)
project(stage_b_two_level)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(srdfdom REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ompl REQUIRED)
find_package(fcl REQUIRED)
find_library(LMA_PLUGIN moveit_lma_kinematics_plugin PATHS /opt/ros/humble/lib REQUIRED)

set(GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${GENERATED_DIR})
execute_process(
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/generate_sources.py
          ${CMAKE_CURRENT_SOURCE_DIR}/../../offline/task28_stage_b_contact/precontact.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/../stage_b_yaw/yaw_planner.cpp
          ${GENERATED_DIR}
  RESULT_VARIABLE GENERATE_RESULT)
if(NOT GENERATE_RESULT EQUAL 0)
  message(FATAL_ERROR "failed to generate mirrored offline sources")
endif()

set(COMMON_INCLUDES ${OMPL_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR}/../../offline/task28_stage_b_contact
                    ${CMAKE_CURRENT_SOURCE_DIR}/../../ros_ws/src/fr3_dual_palletize/src)
set(COMMON_LIBS ${LMA_PLUGIN} ${OMPL_LIBRARIES} yaml-cpp fcl)

add_executable(precontact_left ${GENERATED_DIR}/precontact_left.cpp)
add_executable(precontact_right ${GENERATED_DIR}/precontact_right.cpp)
foreach(target precontact_left precontact_right)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(${target} PRIVATE ${COMMON_INCLUDES})
  target_link_libraries(${target} ${LMA_PLUGIN} ${OMPL_LIBRARIES} yaml-cpp)
  ament_target_dependencies(${target} rclcpp moveit_core moveit_msgs urdf srdfdom)
endforeach()

add_executable(yaw_planner_left ../stage_b_yaw/yaw_planner.cpp)
add_executable(yaw_planner_right ${GENERATED_DIR}/yaw_planner_right.cpp)
foreach(target yaw_planner_left yaw_planner_right)
  target_include_directories(${target} PRIVATE ${COMMON_INCLUDES})
  target_link_libraries(${target} ${COMMON_LIBS})
  ament_target_dependencies(${target} rclcpp moveit_core moveit_msgs urdf srdfdom)
endforeach()
