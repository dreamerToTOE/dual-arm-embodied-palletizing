#!/usr/bin/env bash
set -euo pipefail
isaac_root="${ISAAC_SIM_ROOT:-/home/ubuntu2004/isaacsim-4.5.0}"
ros_lib="$isaac_root/exts/isaacsim.ros2.bridge/humble/lib"
test -f "$ros_lib/librosidl_runtime_c.so"
# Do not inherit a system ROS overlay or a second ROS library implementation.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_PRELOAD
unset FASTRTPS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI PYTHONEXE
export ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="$ros_lib"
exec "$isaac_root/python.sh" "$@"
