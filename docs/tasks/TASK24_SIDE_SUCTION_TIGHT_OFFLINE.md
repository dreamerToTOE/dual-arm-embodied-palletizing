# Task24：侧面吸盘紧协调离线码垛

状态：🟡 物理验证中（教师已确认 L 型阵列侧吸工具）

## 目标

验证两个 FR3 通过对应的左右侧面 L 型吸盘，共同搬运 `120 mm / 0.8 kg` Cube。
第一版是固定离线任务：不使用 Cube 选择器，source、entry 和 target 均已知。

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

- 先完成远侧 `x=0.820` 的 YZ 墙（2 x 2），再完成近侧 `x=0.640` 的 YZ 墙。
- 相邻单元的 Y/Z pitch 为 150 mm，Cube 边长为 120 mm。
- 8 个供料位均在桌面左侧开放区；不使用灰色等待区或中转台。
- `/World/Task24` 可从空 Isaac Stage 重建；两个 FR3、桌面、L 型工具、物理 Cube 和 ROS bridge 均由 Task24 脚本配置。

## 关键几何修正

早期短工具会迫使 FR3 wrist/flange 贴近 Cube 和桌面。教师确认后，Task24 只保留
下列 L 型侧吸阵列；此前的竖直顶吸外观对照版已从源码移除：

- 竖直段：80 mm
- 横向支臂：130 mm
- `side_suction_tcp` 相对 `fr3_link8` 的侧向偏置：155 mm

Isaac 工具、MoveIt URDF 与 bridge 的吸盘约束偏置使用同一尺寸。闭合前读取 Isaac Ground Truth，检查两个 TCP 到 Cube 的 X/Z、跨距、间隙与姿态；不满足阈值时不允许吸附。

## 已验证结果

单 Cube 隔离物理复验（`execution_time_scale=3.0`）通过：

```text
PRE_CLOSE  x=0.523 mm, z=0.349 mm, span=0.198 mm
POST_CLOSE x=0.498 mm, z=0.338 mm, span=0.190 mm
final placement error = 0.427 mm
```

这验证了加长 L 型工具和分轴接近可避免腕部/桌面净空问题。完整 8 Cube 连续验收尚未标记通过；恢复真实供料 Cube 后，已将 FCL 发现的工具扫掠冲突从供料布局中移除，正在进行连续复验。

## 验收命令

先启动 Task24 场景和 bridge，再启动 Task24 专用 MoveIt：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1

ros2 launch fr3_dual_side_suction_description moveit_dual_side_suction.launch.py use_rviz:=true
```

另开终端执行完整离线任务：

```bash
cd /home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1

ros2 run fr3_dual_palletize task24_side_suction_tight --ros-args \
  -p max_cubes:=8 \
  -p isolate_first_cube:=false \
  -p execution_time_scale:=3.0
```

`execution_time_scale=1.0` 仅可用于无物理验收的快速检查；对 0.8 kg Cube，当前
Surface Gripper 物理模型在共同下降段不能保证稳态跟踪，正式验收固定使用 `3.0`。
