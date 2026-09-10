# Task11：双吸盘共同物体基线

状态：✅ 已在 Isaac Sim 运行时验收（2026-09-10）：两个 Surface Gripper 同时 CLOSED，SharedBox 在未抬升阶段保持稳定。

## 目标

在不引入共同搬运轨迹、力分配或柔顺控制前，先验证两个 FR3 紧凑顶部吸盘可对**同一个**动态刚体分别建立 Surface Gripper 连接：

```text
LEFT contact  -> LEFT SUCTION CLOSED
RIGHT contact -> RIGHT SUCTION CLOSED
                         ↓
                 SharedBox 同时具有两个物理约束
```

Task11 只验收双吸附建立和状态/位姿观测；共同抬升、保持相对几何和共同运输属于 Task12。

## 场景参数

| 项目 | 值 |
| --- | --- |
| Prim | `/World/SharedBox` |
| 尺寸 | `0.220 × 0.320 × 0.080 m`（x × y × z） |
| 质量 | `1.000 kg` |
| 初始中心 | `(0.550, 0.000, 0.090) m` |
| 顶面高度 | `z = 0.130 m` |
| 左吸附参考点 | `(0.550, -0.100, 0.131) m` |
| 右吸附参考点 | `(0.550, +0.100, 0.131) m` |
| 两点间距 | `0.200 m` |

箱体底面位于 Task06 桌面顶面 `z=0.050 m`。两个参考点距各自 y 向边缘 60 mm，明显大于 10 mm 吸盘 cup 半径；因此它们是分离、对称的顶部吸附点。

## 复用与约束

- 复用 Task06 双 FR3、桌面、独立 `/left` / `/right` joint command 链路和 MoveIt 描述。
- 复用 Task09/10 已验收的 Surface Gripper 参数：`gripThreshold=0.003 m`、`forceLimit=torqueLimit=1e6`、`retryClose=True`。
- 两个 gripper 均以各自 `fr3_hand` 为 parent，但均可连接 `/World/SharedBox`。
- 不扩大 Allowed Collision Matrix；不关闭重力；不把共同箱体拆为两个虚拟物体。
- `execute:=false` 的预检不发布机器人关节或吸盘命令；`execute:=true` 只执行到双臂接触与双吸盘闭合，不实现共同抬升。这样可将“共同物体物理约束是否成立”与后续紧协调轨迹问题隔离。

## 新增接口

```text
isaac/scripts/task11_shared_box_scene.py
isaac/scripts/task11_shared_box_bridge.py

fr3_dual_palletize/task11_shared_box_grasp

/task11/left/suction_command   std_msgs/Bool
/task11/left/suction_state     std_msgs/Bool
/task11/right/suction_command  std_msgs/Bool
/task11/right/suction_state    std_msgs/Bool
/task11/shared_box_pose        geometry_msgs/PoseStamped
```

## 首次验收流程

在 Isaac Timeline Stop 时：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing-task11-shared-object-baseline/isaac/scripts/task11_shared_box_scene.py").read())
```

在 Play 后：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing-task11-shared-object-baseline/isaac/scripts/task11_shared_box_bridge.py").read())
```

Task11 控制节点在真实执行时，先将左右吸盘分别移动到表中参考点，再同步下探至 CONTACT 并向两个 command topic 发送 `true`。先做只读预检；`execute:=false` 不发布任何 joint 或 suction command：

```bash
cd ~/lmy/dual-arm-embodied-palletizing-task11-shared-object-baseline/ros_ws
source /opt/ros/humble/setup.bash
source ~/lmy/dual-arm-embodied-palletizing/ros_ws/install/setup.bash
source install/setup.bash

ros2 run fr3_dual_palletize task11_shared_box_grasp --ros-args \
  -p execute:=false
```

预检成功后才执行真实双吸附：

```bash
ros2 run fr3_dual_palletize task11_shared_box_grasp --ros-args \
  -p execute:=true \
  -p grasp_timeout_sec:=3.0
```

真实执行成功时确认：

```text
/task11/left/suction_state  = true
/task11/right/suction_state = true
```

保持双吸附期间读取 `/task11/shared_box_pose`，确认箱体没有掉落或被重力穿透。释放顺序将作为 Task13 的内容，不在本 Task11 基线中提前实现。

## 运行时验收记录（2026-09-10）

Bridge 重载后，先确认 Isaac Ground Truth 与初始状态：

```text
/task11/shared_box_pose
  position    = (0.550000, 0.000000, 0.090000) m
  orientation = (0.000000, 0.000000, 0.000000, 1.000000)
/task11/left/suction_state  = false
/task11/right/suction_state = false
```

随后执行 `execute:=false` 只读预检：左右 `PRE_CONTACT` 规划均成功，左右 `CONTACT` Cartesian fraction 均为 `1.0000`，节点输出：

```text
Task11 PRECHECK PASS：未发布任何 joint / suction 命令。
```

执行真实验收（`execute:=true`，`grasp_timeout_sec:=3.0`）后，左右机械臂均到达 `PRE_CONTACT`，同步下探的两个 Cartesian 段均为 `fraction=1.0000`；节点输出：

```text
Task11 PASS：双 Surface Gripper 同时 CLOSED；SharedBox=(0.5500, -0.0000, 0.0900)。
```

最终再次读取 Ground Truth：

| 项目 | 初始值 (m) | 双吸附闭合后 (m) |
| --- | --- | --- |
| SharedBox center | `(0.550000, 0.000000, 0.090000)` | `(0.550100, -0.000056, 0.089997)` |
| 水平位移 | — | `0.115 mm` |
| 三维位移 | — | `0.115 mm` |

同时 `/task11/left/suction_state=true` 且 `/task11/right/suction_state=true`。因此 Task11 的两个物理吸附约束已共同建立，且在不抬升的基线阶段未出现掉落、穿透或明显漂移。

## 后续边界

```text
Task11: 双吸附建立与 SharedBox Ground Truth
Task12: 共同抬升 / 共同运输 / 相对位姿保持
Task13: 共同下降 / 同步释放 / 双臂安全退出
```
