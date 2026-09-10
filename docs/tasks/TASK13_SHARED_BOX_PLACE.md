# Task13：双吸盘共同放置、释放与安全退出

状态：✅ 已在 Isaac Sim 运行时验收（2026-09-10）：共同下降、同步释放、Ground Truth 回写与共同安全退出均完成。

## 目标

完成紧协调共同搬运的最小闭环：两个 FR3 对同一 `SharedBox` 建立双吸附、共同抬升、共同运输、共同下降、同步释放并安全退出。

```text
CONTACT
  -> both CLOSED
  -> COMMON_LIFT (+50 mm)
  -> COMMON_TRANSPORT (+100 mm in X)
  -> COMMON_DESCENT
  -> both SUCTION OFF
  -> PhysX settle + MoveIt World write-back
  -> COMMON_RETREAT
```

## 放置时序

| 阶段 | SharedBox 状态 | MoveIt 表示 |
| --- | --- | --- |
| CONTACT 至 TRANSPORT | 双 Surface Gripper 物理保持 | 为有意接触临时移出 World；完整双臂 RobotState 仍做臂-臂碰撞检查 |
| DESCENT / RETREAT 规划 | 仍双吸附 | 箱体仍未作为 World CollisionObject，避免接触起点碰撞 |
| 双吸盘 OFF 后 | PhysX 自然落稳 | 读取 `/task11/shared_box_pose` 最新 Ground Truth |
| RETREAT 执行前 | 已释放 | 按 Ground Truth 将 `task11_shared_box` 加回 MoveIt World；不重新规划 RETREAT |

桌面顶面为 `z=0.050 m`，SharedBox 高度为 `0.080 m`，目标中心为 `(0.650, 0.000, 0.090) m`。下降目标保持既有 `CONTACT_CLEARANCE_Z=0.001 m`，因此释放前箱体底面距桌面 1 mm；同步 OFF 后由 PhysX 使其自然落稳。

不扩大 ACM、不关闭重力、不永久忽略 SharedBox。

## 节点与参数

```text
fr3_dual_palletize/task13_shared_box_place
```

节点复用 Task11 已验证的场景和 Bridge：

```text
isaac/scripts/task11_shared_box_scene.py
isaac/scripts/task11_shared_box_bridge.py
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `lift_height_m` | `0.050 m` | 共同抬升高度 |
| `transport_delta_x_m` | `0.100 m` | 共同运输 X 位移 |
| `transport_delta_y_m` | `0.000 m` | 共同运输 Y 位移 |
| `release_timeout_sec` | `3.0 s` | 两吸盘均 OPEN 的等待上限 |
| `settle_sec` | `1.0 s` | 释放后 PhysX 稳定等待 |
| `placement_tolerance_m` | `0.010 m` | 放置后 SharedBox 位置误差上限 |
| `box_orientation_tolerance_rad` | `0.035 rad` | 放置后姿态误差上限（约 2°） |

## 验收步骤

在 Isaac Timeline Stop 时运行：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing-task13-shared-place/isaac/scripts/task11_shared_box_scene.py").read())
```

在 Play 后运行：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing-task13-shared-place/isaac/scripts/task11_shared_box_bridge.py").read())
```

启动双臂 MoveIt：

```bash
source /opt/ros/humble/setup.bash
source ~/lmy/dual-arm-embodied-palletizing/ros_ws/install/setup.bash
ros2 launch fr3_dual_compact_suction_description moveit_dual_compact_suction.launch.py
```

先运行只读预检：

```bash
cd ~/lmy/dual-arm-embodied-palletizing-task13-shared-place/ros_ws
source /opt/ros/humble/setup.bash
source ~/lmy/dual-arm-embodied-palletizing/ros_ws/install/setup.bash
source install/setup.bash

ros2 run fr3_dual_palletize task13_shared_box_place --ros-args \
  -p execute:=false
```

预检要求左右 `CONTACT`、`COMMON_LIFT`、`COMMON_TRANSPORT`、`COMMON_DESCENT` 与 `COMMON_RETREAT` 的 Cartesian fraction 均为 `1.0000`，且打印 `Task13 PRECHECK PASS`。确认后执行真实闭环：

```bash
ros2 run fr3_dual_palletize task13_shared_box_place --ros-args \
  -p execute:=true \
  -p lift_height_m:=0.050 \
  -p transport_delta_x_m:=0.100 \
  -p transport_delta_y_m:=0.000
```

## 验收标准

```text
T13-01  execute:=false 不发布 joint 或 suction 命令
T13-02  CONTACT / LIFT / TRANSPORT / DESCENT / RETREAT 的所有 Cartesian 规划均成功
T13-03  双吸盘均 CLOSED 后才共同抬升
T13-04  共同下降结束后才同步发送 SUCTION OFF
T13-05  两个 suction_state 均为 false 后才读取 settle pose
T13-06  释放后的 Ground Truth SharedBox 已回写 MoveIt World
T13-07  RETREAT 在释放前规划、释放后执行，不从接触起点重新规划
T13-08  放置位置误差 <= 10 mm，姿态误差 <= 2°
T13-09  双臂共同 RETREAT 完成，无碰撞、无掉箱
```

## 编译验证（2026-09-10）

```bash
cd ~/lmy/dual-arm-embodied-palletizing-task13-shared-place/ros_ws
source /opt/ros/humble/setup.bash
source ~/lmy/dual-arm-embodied-palletizing/ros_ws/install/setup.bash
colcon build --packages-select fr3_dual_palletize --symlink-install
source install/setup.bash
ros2 pkg executables fr3_dual_palletize | rg 'task13_shared_box_place'
```

结果：`fr3_dual_palletize` 编译成功，`task13_shared_box_place` 已被 ament 索引发现。

## Isaac 运行时验收（2026-09-10）

真实闭环运行通过。左右 `CONTACT`、`COMMON_LIFT`、`COMMON_TRANSPORT`、`COMMON_DESCENT` 与 `COMMON_RETREAT` 的 Cartesian 规划均返回 `fraction=1.0000`。完整运行的 Ground Truth 指标：

```text
LIFT:
  expected_box_z          = 0.1396 m
  actual_box              = (0.5497, 0.0001, 0.1365) m
  box_error               = 3.105 mm
  relative_link8_error    = 1.022 mm
  orientation_error       = 0.026 deg

TRANSPORT:
  expected_box            = (0.6497, 0.0001, 0.1365) m
  actual_box              = (0.6496, -0.0001, 0.1363) m
  box_error               = 0.261 mm
  relative_link8_error    = 0.187 mm
  orientation_error       = 0.030 deg

PLACE:
  expected_box            = (0.6500, 0.0000, 0.0900) m
  actual_box              = (0.6498, -0.0009, 0.0900) m
  placement_error         = 0.947 mm
  orientation_error       = 0.026 deg
```

节点输出：

```text
Task13 PASS：共同下降、同步释放、SharedBox Ground Truth 回写与共同安全退出均完成。
```

所有误差均满足 T13-08；可视化确认两个吸盘已脱离，SharedBox 稳定留在目标桌面位置，两臂完成共同向上退出。
