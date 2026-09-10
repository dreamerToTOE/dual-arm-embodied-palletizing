# Task12：双吸盘共同抬升与运输

状态：🟡 Task12-A 共同抬升已在 Isaac Sim 运行时验收（2026-09-10）；Task12-B 共同水平运输已实现并编译，等待 Isaac 验收。

## 目标与边界

Task11 已确认左右 Surface Gripper 可以同时连接同一动态 `SharedBox`。Task12 的最小闭环是在此基础上验证：两个 FR3 能以相同起止时长共同竖直抬升箱体，并保持箱体姿态和两端末端的相对几何。

```text
PRE_CONTACT
  -> CONTACT
  -> LEFT / RIGHT SUCTION ON
  -> both CLOSED
  -> COMMON_LIFT (+50 mm)
  -> COMMON_TRANSPORT (+100 mm in X)
  -> Ground Truth relative-geometry checks
```

Task12-A 已完成共同抬升。Task12-B 在安全抬升高度执行共同水平运输；共同下降与释放仍留给 Task13。

## 实现

新增可执行节点：

```text
fr3_dual_palletize/task12_shared_box_lift
fr3_dual_palletize/task12_shared_box_transport
```

它直接复用 Task11 已验证的场景与 Bridge：

```text
isaac/scripts/task11_shared_box_scene.py
isaac/scripts/task11_shared_box_bridge.py
```

共同抬升以左右 `CONTACT` 末态组装**同一个全局 RobotState**，再分别计算两条竖直 Cartesian 轨迹。Task12-B 同样以共同抬升的末态组装全局 RobotState，计算同高度的共同水平轨迹。因此每次 Cartesian 计算时，另一台机械臂都已处于对应阶段的共同搬运姿态，MoveIt 会执行完整双臂模型的臂-臂碰撞检查。每个阶段的两条轨迹均按较长者重新定时，确保同时开始、同时结束。

由于一个 MoveIt `AttachedBody` 不能同时附着到左右两个末端，`SharedBox` 在有意接触、吸附和本 Task 的短距离抬升中临时不作为 MoveIt World 障碍物；它的双端保持由 Isaac 中两个已验收的 Surface Gripper 物理约束承担。没有扩大 ACM、关闭重力或永久忽略 SharedBox。

## 验收参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `lift_height_m` | `0.050 m` | 共同竖直抬升高度 |
| `grasp_timeout_sec` | `3.0 s` | 两吸盘 CLOSED 等待上限 |
| `settle_sec` | `1.0 s` | 抬升后 PhysX 稳定时间 |
| `box_lift_tolerance_m` | `0.010 m` | SharedBox 相对期望抬升终点的三维误差上限 |
| `relative_tcp_tolerance_m` | `0.003 m` | 左右 `link8` 相对向量变化上限 |
| `box_orientation_tolerance_rad` | `0.035 rad` | SharedBox 抬升前后姿态变化上限（约 2°） |
| `transport_delta_x_m` | `0.100 m` | Task12-B 共同运输的 X 位移 |
| `transport_delta_y_m` | `0.000 m` | Task12-B 共同运输的 Y 位移 |
| `box_transport_tolerance_m` | `0.010 m` | SharedBox 共同运输终点三维误差上限 |

节点输出以下实际测量量：

```text
expected_box_z
actual_box
box_error
relative_link8_error
orientation_error
```

只有三个误差均在阈值内才打印 `Task12 PASS`。

## 验收步骤

先在 Isaac Timeline Stop 执行 Task11 场景，再 Play 后执行 Task11 Bridge：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing-task12-shared-transport/isaac/scripts/task11_shared_box_scene.py").read())
```

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing-task12-shared-transport/isaac/scripts/task11_shared_box_bridge.py").read())
```

启动 Task06 双臂 MoveIt 后，先做无命令预检：

```bash
cd ~/lmy/dual-arm-embodied-palletizing-task12-shared-transport/ros_ws
source /opt/ros/humble/setup.bash
source ~/lmy/dual-arm-embodied-palletizing/ros_ws/install/setup.bash
source install/setup.bash

ros2 run fr3_dual_palletize task12_shared_box_transport --ros-args \
  -p execute:=false
```

仅当预检输出左右 `CONTACT`、左右 `COMMON_LIFT` 与左右 `COMMON_TRANSPORT` 的 Cartesian fraction 均为 `1.0000` 时，才运行真实共同搬运：

```bash
ros2 run fr3_dual_palletize task12_shared_box_transport --ros-args \
  -p execute:=true \
  -p lift_height_m:=0.050 \
  -p transport_delta_x_m:=0.100 \
  -p transport_delta_y_m:=0.000
```

真实运行后不要在同一场景内重复执行；先 Stop、重新运行场景脚本并重载 Bridge，恢复初始物理状态后才可再次测试。

## 编译验证（2026-09-10）

已在独立 Task12 工作区执行：

```bash
cd ~/lmy/dual-arm-embodied-palletizing-task12-shared-transport/ros_ws
source /opt/ros/humble/setup.bash
source ~/lmy/dual-arm-embodied-palletizing/ros_ws/install/setup.bash
colcon build --packages-select fr3_dual_palletize --symlink-install
source install/setup.bash
ros2 pkg executables fr3_dual_palletize | rg 'task1[12]_shared_box'
```

结果：`fr3_dual_palletize` 编译成功，`task11_shared_box_grasp`、`task12_shared_box_lift` 与 `task12_shared_box_transport` 均已被 ament 索引发现。Task12-B 尚未执行 Isaac 运行时测试。

## Isaac 运行时验收（2026-09-10）

真实执行 `lift_height_m:=0.050` 后，日志确认：

```text
Task12 left  COMMON_LIFT Cartesian fraction=1.0000, error=1
Task12 right COMMON_LIFT Cartesian fraction=1.0000, error=1

Task12 LIFT METRICS:
  expected_box_z          = 0.1400 m
  actual_box              = (0.5499, 0.0000, 0.1396) m
  box_error               = 0.446 mm
  relative_link8_error    = 0.252 mm
  orientation_error       = 0.052 deg

Task12 PASS：双 Surface Gripper 共同抬升 0.050 m；
SharedBox 与双臂相对几何均在阈值内。
```

所有值满足 T12-05 至 T12-07：箱体实际抬升误差远小于 10 mm，左右 `link8` 相对向量变化远小于 3 mm，箱体姿态变化远小于 2°。可视化也确认 SharedBox 由两端吸盘稳定共同保持。

## 验收标准

```text
T12-01  execute:=false 不发布 joint 或 suction 命令
T12-02  两个 PRE_CONTACT、CONTACT 与 COMMON_LIFT 均规划成功
T12-03  两个 Surface Gripper 均 CLOSED 后才允许抬升
T12-04  两条共同抬升轨迹同时开始、同时结束
T12-05  SharedBox 实际抬升误差 <= 10 mm
T12-06  左右 link8 相对向量变化 <= 3 mm
T12-07  SharedBox 姿态变化 <= 2°
T12-08  共同运输前，Task12-A 三项抬升验收必须通过
T12-09  两条共同水平轨迹同时开始、同时结束
T12-10  SharedBox 共同运输终点误差 <= 10 mm
T12-11  运输阶段左右 link8 相对向量变化 <= 3 mm，箱体姿态变化 <= 2°
```

## 后续边界

```text
Task12-A  共同抬升与相对几何保持（✅ 已完成）
Task12-B  共同水平运输与路径级相对约束（🟡 已实现，待 Isaac 验收）
Task13    共同下降、同步释放、两臂安全退出
```
