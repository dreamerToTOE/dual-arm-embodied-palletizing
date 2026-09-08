# fr3_dual_compact_suction_description

Task06-C 双 FR3 + 双紧凑吸盘 MoveIt2 描述包。

## 结构

```text
world
├── left_fr3_link0 ... left_fr3_link8
│   └── left_fr3_compact_suction
└── right_fr3_link0 ... right_fr3_link8
    └── right_fr3_compact_suction
```

基座位姿与 Isaac Task06-A 一致：

```text
left  = (0.55, -0.50, 0.00), yaw = 0
right = (0.55, +0.50, 0.00), yaw = 0
```

规划组：

```text
left_arm
right_arm
dual_arm
```

两端吸盘尺寸继续严格复用 Task04 / Task05：

- stem 半径 6 mm，长度 99 mm
- cup 半径 10 mm，长度 6 mm
- TCP 相对各自 link8 沿局部 +Z 105 mm

当前 planning tip 仍为：

```text
left_fr3_link8
right_fr3_link8
```

这样保留 Task04 基础码垛事件单元中的 0.105 m TCP 手工换算语义。

## JointState 适配

Isaac Task06-B 继续发布：

```text
/left/joint_states
/right/joint_states
```

`dual_joint_state_bridge.py` 将其合并并前缀化为 MoveIt / robot_state_publisher 使用的：

```text
/joint_states

left_fr3_joint1 ... left_fr3_joint7
right_fr3_joint1 ... right_fr3_joint7
```

因此标准 `/tf` 中左右 frame 名也保持唯一。

## MoveIt 环境

`task06_environment_publisher.py` 会在 `move_group` 启动后等待 `/apply_planning_scene`，然后把 Isaac Task06-A 的基准桌面写入 MoveIt Planning Scene。

环境参数统一放在：

```text
config/environment.yaml
```

当前桌面：

```text
id      = task06_table
frame   = world
center  = (0.55, 0.00, 0.025)
size    = (1.20, 0.80, 0.05)
```

因此 RViz 中显示的桌面不是装饰模型，而是会参与 MoveIt 碰撞检测的 `CollisionObject`。

## 编译

```bash
cd ~/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fr3_dual_compact_suction_description --symlink-install
source install/setup.bash
```

## 启动

先保证 Isaac Task06-A 场景与 Task06-B ROS Graph 正在 Play，再运行：

```bash
ros2 launch fr3_dual_compact_suction_description \
  moveit_dual_compact_suction.launch.py
```

启动后应同时具备：双臂模型、3 个 Planning Group、14 关节 `/joint_states`、唯一标准 `/tf`，以及 `task06_table` MoveIt 环境碰撞体。

Task06-C 当前只验证双臂描述、Planning Group、标准 TF 和碰撞模型；不在本阶段执行双臂抓取。
