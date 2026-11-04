# Final bounded Stage B study

This directory contains the final CPU-only parameterized feasibility study. The source generator reads the existing Task28 precontact and Stage B yaw planners, changes the compartment walls from fixture parameters, and keeps the fixed side suction geometry. `run_final.py` freezes the rule on the development split and evaluates the independent test split.

```bash
source /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws/install/setup.bash
python3 research/stage_b_final/generate_sources.py /tmp/stage_b_final_src
cmake -S research/stage_b_final -B /tmp/stage_b_final_build -DGENERATED_DIR=/tmp/stage_b_final_src
cmake --build /tmp/stage_b_final_build -j8
python3 research/stage_b_final/run_final.py \
  /tmp/stage_b_final_build \
  research/stage_b_two_level/evidence/reference/robot.urdf \
  research/stage_b_two_level/evidence/reference/robot.srdf \
  research/stage_b_final/evidence/final_study_20261104_final
python3 research/stage_b_final/check_final.py \
  research/stage_b_final/evidence/final_study_20261104_final
```

The final evidence has 36 raw scenes, 28 complete Level 2 scenes, 168 yaw candidates, and 120 passing candidates. Physical contact and recovery remain deferred.
