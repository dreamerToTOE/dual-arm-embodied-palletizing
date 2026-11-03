#!/usr/bin/env bash
set -euo pipefail
repo=$(git rev-parse --show-toplevel)
test_dir="$repo/offline/task28_task27_integration/tests/fcl_replay"
tmp_dir=$(mktemp -d)
trap 'rm -rf "$tmp_dir"' EXIT
xacro "$repo/ros_ws/src/fr3_dual_side_suction_description/urdf/dual_fr3_side_suction.urdf.xacro" > "$tmp_dir/robot.urdf"
xacro "$repo/ros_ws/src/fr3_dual_side_suction_description/srdf/dual_fr3_side_suction.srdf.xacro" > "$tmp_dir/robot.srdf"
cmake -S "$test_dir" -B "$tmp_dir/build" >/dev/null
cmake --build "$tmp_dir/build" -j2 >/dev/null
"$tmp_dir/build/replay" "$tmp_dir/robot.urdf" "$tmp_dir/robot.srdf"
"$tmp_dir/build/side_contact"
