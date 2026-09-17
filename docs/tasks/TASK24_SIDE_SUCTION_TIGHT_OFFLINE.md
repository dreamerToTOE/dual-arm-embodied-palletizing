# Task24：侧面吸盘紧协调离线码垛

状态：🟡 单 Cube 物理基线待验收（教师已确认 L 型阵列侧吸工具）

## 目标

验证两个 FR3 通过对应的左右侧面 L 型吸盘，共同搬运 `120 mm / 0.8 kg` Cube。
当前第一版是固定离线单件任务：不使用 Cube 选择器，`Cube_01` 的 source、entry
和 target 均已知。只有单件闭环通过后，才恢复多 Cube 紧协调任务。

每件 Cube 的流程为：

```text
RRTConnect 空载接近
-> 左右侧面对应 CONTACT
-> 双吸盘同步 CLOSED
-> 共同 Z 抬升
-> 共同 X 运输
-> 共同 Y 对齐
-> 共同 Z 下降到 entry
-> 共同 +X 20 mm 短推
-> 同步释放
-> 共同 Z 退出
```

吸住 Cube 后禁止 OMPL 任意绕行，负载段严格采用 `Z -> X -> Y -> Z -> X`。
空载阶段允许 RRTConnect；每个同步候选均由完整双臂 RobotState + FCL 采样验证。

## 垛型与场景

- 当前 Stage 只创建一个真实 Dynamic Rigid Body：`/World/Task24/Supply/Cube_01`。
- 当前目标是远侧 YZ 墙的第一个单元：`(0.820, -0.075, 0.110) m`。
- Cube 边长为 120 mm、质量为 0.8 kg；不使用灰色等待区或中转台。
- `/World/Task24` 可从空 Isaac Stage 重建；两个 FR3、桌面、L 型工具、物理 Cube 和 ROS bridge 均由 Task24 脚本配置。

多 Cube 阶段的前提是：Isaac 中每一个真实 Cube、bridge 发布的 PoseArray、MoveIt
Planning Scene 中的 CollisionObject 三者数量与顺序完全一致。单件阶段不会创建
`Cube_02` 至 `Cube_08` 的 placeholder、visual、Collider 或 RigidBody。

## 关键几何修正

早期短工具会迫使 FR3 wrist/flange 贴近 Cube 和桌面。教师确认后，Task24 只保留
下列 L 型侧吸阵列；此前的竖直顶吸外观对照版已从源码移除：

- 竖直段：80 mm
- 横向支臂：130 mm
- `side_suction_tcp` 相对 `fr3_link8` 的侧向偏置：155 mm

Isaac 工具、MoveIt URDF 与 bridge 的吸盘约束偏置使用同一尺寸。闭合前读取 Isaac Ground Truth，检查两个 TCP 到 Cube 的 X/Z、跨距、间隙与姿态；不满足阈值时不允许吸附。

## 当前验收边界

此前的单 Cube 记录不作为当前验收结论：当时 Isaac 场景仍有其它真实 Cube，执行器
却只向 MoveIt 加入了 `Cube_01`，两侧碰撞场景不一致。该问题已通过本次单件场景、
单件 bridge 和 `active_cube_count:=1` 约束修复。

本轮单件验收要求：双吸盘对应接触检查通过、`COMMON_LIFT -> COMMON_X_TRAVEL ->
COMMON_Y_ALIGN -> COMMON_DESCENT_TO_ENTRY -> COMMON_SIDE_SHORT_PUSH` 全部通过同步 FCL
门禁、两个吸盘稳定 CLOSED、最终 Cube Ground Truth 误差不超过 10 mm。通过前不得
开始多 Cube 连续搬运。

## 验收命令

先启动 Task24 场景和 bridge，再启动 Task24 专用 MoveIt：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1

ros2 launch fr3_dual_side_suction_description moveit_dual_side_suction.launch.py use_rviz:=true
```

另开终端执行单 Cube 紧协调任务：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1

ros2 run fr3_dual_palletize task24_side_suction_tight --ros-args \
  -p max_cubes:=1 \
  -p active_cube_count:=1 \
  -p execution_time_scale:=3.0
```

`execution_time_scale=1.0` 仅可用于无物理验收的快速检查；对 0.8 kg Cube，当前
Surface Gripper 物理模型在共同下降段不能保证稳态跟踪，正式验收固定使用 `3.0`。
