#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 1 ]]; then
  echo 'Usage: run.sh NEW_OUTPUT_DIRECTORY' >&2
  exit 2
fi
repo=$(git rev-parse --show-toplevel)
source_dir="$repo/offline/task28_stage_b_contact"
output_dir="$1"
mkdir "$output_dir"
tmp_dir=$(mktemp -d)
trap 'rm -rf "$tmp_dir"' EXIT
xacro "$repo/ros_ws/src/fr3_dual_side_suction_description/urdf/dual_fr3_side_suction.urdf.xacro" > "$tmp_dir/robot.urdf"
xacro "$repo/ros_ws/src/fr3_dual_side_suction_description/srdf/dual_fr3_side_suction.srdf.xacro" > "$tmp_dir/robot.srdf"
cmake -S "$source_dir" -B "$tmp_dir/build" >/dev/null
cmake --build "$tmp_dir/build" -j2 >/dev/null
"$tmp_dir/build/precontact" "$tmp_dir/robot.urdf" "$tmp_dir/robot.srdf" "$source_dir/fixture.json" "$output_dir/trajectory.csv" 2>&1 | tee "$output_dir/result.log"
for item in "robot.urdf:$tmp_dir/robot.urdf" "robot.srdf:$tmp_dir/robot.srdf" "fixture.json:$source_dir/fixture.json" "trajectory.csv:$output_dir/trajectory.csv"; do
  label=${item%%:*}; path=${item#*:}
  printf '%s  %s\n' "$(sha256sum "$path" | cut -d ' ' -f 1)" "$label"
done > "$output_dir/sha256.txt"
