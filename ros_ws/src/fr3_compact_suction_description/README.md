# fr3_compact_suction_description

项目自定义 FR3 紧凑顶部吸盘 MoveIt 描述包。

## 几何基线

与 Isaac Sim Task04/Task05 完全一致：

- stem 半径：6 mm
- stem 长度：99 mm
- cup 半径：10 mm
- cup 长度：6 mm
- TCP：相对 `fr3_link8` 沿局部 `+Z` 105 mm

MoveIt planning group 当前仍以 `fr3_link8` 为 tip，以保持 Task04 基础码垛事件单元中的 0.105 m TCP 换算逻辑不变。

## 启动

```bash
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fr3_compact_suction_description --symlink-install
source install/setup.bash

ros2 launch fr3_compact_suction_description moveit_compact_suction.launch.py
```

该 launch 不加载官方 `franka_hand` / `cobot_pump`，也不创建额外 fake joint-state 源。Isaac ActionGraph 继续负责 `/joint_states` 和 `/joint_command` 联调。
