# Task14-A：双吸盘共同搬运连续几何观测

状态：🟡 已实现并编译，等待 Isaac Sim 运行时验收。

## 目的

Task12 / Task13 已证明各阶段终点的 SharedBox 误差处于毫米级。Task14-A 不修改既有控制器，而是在**双吸盘同时 CLOSED** 的整个物理窗口连续观测紧协调几何，建立后续是否需要闭环纠偏、柔顺或负载分配的定量基线。

```text
both CLOSED
  -> 连续采样 suction TCP / SharedBox Ground Truth
  -> LIFT / TRANSPORT / DESCENT
  -> 任一 suction OPEN
  -> 输出 max / RMS 连续误差报告
```

## 新接口

Task11 Bridge 新增：

```text
/task11/left/suction_tcp_pose   geometry_msgs/PoseStamped
/task11/right/suction_tcp_pose  geometry_msgs/PoseStamped
```

位姿来自 Isaac USD 的：

```text
/World/left_fr3/fr3_hand/suction_tool/suction_tcp
/World/right_fr3/fr3_hand/suction_tool/suction_tcp
```

新增只读 ROS 节点：

```text
fr3_dual_palletize/task14_shared_box_geometry_monitor
```

它不发布 joint、suction 或 Planning Scene 命令。

## 连续指标

开始采样时（首次左右状态均为 `CLOSED`）保存基准几何。此后每个 SharedBox Ground Truth 样本计算：

| 指标 | 定义 | 默认上限 |
| --- | --- | --- |
| `relative_tcp_error` | 左右 suction TCP 相对向量相对基准的变化 | 3 mm |
| `box_to_tcp_midpoint_error` | SharedBox 中心相对双 TCP 中点的偏移变化 | 5 mm |
| `box_orientation_error` | SharedBox 相对初始姿态的角度变化 | 2° |
| `sync_tolerance_sec` | 三路 Ground Truth 时间戳最大允许偏差 | 30 ms |

报告同时输出每项的 `max` 与 `RMS`。只有 SharedBox、left TCP 与 right TCP 三路时间戳相差不超过 30 ms 时才接受该样本；不同步样本会计数并丢弃。只在任一吸盘变为 `OPEN` 后结束并判定；因此预接近、接触前和释放后的自由落体都不污染共同搬运窗口。

## 验收步骤

在 Isaac Timeline Stop 时运行 Task11 场景：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing-task14-continuous-geometry/isaac/scripts/task11_shared_box_scene.py").read())
```

Play 后加载**更新后的** Bridge：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing-task14-continuous-geometry/isaac/scripts/task11_shared_box_bridge.py").read())
```

启动 Task06 MoveIt 后，先按 Task13 文档运行 `task13_shared_box_place --ros-args -p execute:=false`。预检通过后，另开两个终端。

终端 A：先启动只读监测器。

```bash
cd ~/lmy/dual-arm-embodied-palletizing-task14-continuous-geometry/ros_ws
source /opt/ros/humble/setup.bash
source ~/lmy/dual-arm-embodied-palletizing/ros_ws/install/setup.bash
source install/setup.bash

ros2 run fr3_dual_palletize task14_shared_box_geometry_monitor --ros-args \
  -p timeout_sec:=45.0
```

终端 B：立刻执行 Task13 完整闭环。

```bash
cd ~/lmy/dual-arm-embodied-palletizing-task14-continuous-geometry/ros_ws
source /opt/ros/humble/setup.bash
source ~/lmy/dual-arm-embodied-palletizing/ros_ws/install/setup.bash
source install/setup.bash

ros2 run fr3_dual_palletize task13_shared_box_place --ros-args \
  -p execute:=true \
  -p lift_height_m:=0.050 \
  -p transport_delta_x_m:=0.100 \
  -p transport_delta_y_m:=0.000
```

## 验收标准

```text
T14-01  更新后的 Bridge 发布左右 suction TCP Ground Truth
T14-02  监测器不发布任何控制命令
T14-03  监测窗口从 both CLOSED 开始，到任一 OPEN 结束
T14-04  输出有效样本数及三项 max / RMS 连续指标
T14-04a 输出并丢弃三路 Ground Truth 时间戳不一致的样本
T14-05  max relative_tcp_error <= 3 mm
T14-06  max box_to_tcp_midpoint_error <= 5 mm
T14-07  max box_orientation_error <= 2°
```

## 编译验证（2026-09-10）

```bash
cd ~/lmy/dual-arm-embodied-palletizing-task14-continuous-geometry/ros_ws
source /opt/ros/humble/setup.bash
source ~/lmy/dual-arm-embodied-palletizing/ros_ws/install/setup.bash
colcon build --packages-select fr3_dual_palletize --symlink-install
source install/setup.bash
ros2 pkg executables fr3_dual_palletize | rg 'task14_shared_box_geometry_monitor'
```

结果：`fr3_dual_palletize` 编译成功，`task14_shared_box_geometry_monitor` 已被 ament 索引发现；时间戳对齐修复后等待重新执行 Isaac 运行时验收。

## 时间戳对齐修复（2026-09-10）

首次运行曾出现 `box_to_tcp_midpoint max=25.284 mm`，但同时的 TCP 相对误差仍为 `2.896 mm`、箱体姿态误差仅 `0.403°`。这与 10 Hz Bridge 中“当前周期 SharedBox 回调先于当前周期 TCP 回调”造成的跨周期混采一致，不能据此推断共同搬运失稳。

监测器现以三路 ROS header 时间戳进行 30 ms 对齐，只对齐后采样，并报告 `timestamp_rejected` 数量。该修复不修改任何机器人、吸盘或 MoveIt 控制逻辑；需要重新执行 Task14-A 运行时验收。

## 共同轨迹同步起跑修复（2026-09-10）

时间戳对齐后，`box_to_tcp_midpoint` 已降至 `0.989 mm`、姿态误差为 `0.645°`，但 `relative_tcp max=3.667 mm`，仍超过 3 mm 阈值。这是有效的动态误差，不能通过放宽阈值处理。

根因是既有共同阶段在两个独立线程中分别调用 `steady_clock::now()`：即使轨迹拥有相同总时长，也会存在毫秒级起跑偏差。共同轨迹执行器现改为：

```text
CONTACT / LIFT / TRANSPORT / DESCENT / RETREAT
  -> 同一 future steady_clock start（提前 100 ms）
  -> 左右线程 sleep_until 同一时刻
  -> 各自按该共享时钟插值并发布 joint command
```

该改动不改变规划轨迹、Surface Gripper 参数、MoveIt 碰撞模型或验收阈值。已重新编译，等待完整 Task13 + Task14-A 运行时复验。

## 后续边界

Task14-A 只建立连续误差基线。若连续误差超限，Task14-B 再根据误差类型选择相对位姿反馈、柔顺、负载分配或局部重规划；不预设不必要的控制复杂度。
