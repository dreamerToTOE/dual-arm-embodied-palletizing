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

报告同时输出每项的 `max` 与 `RMS`。只在任一吸盘变为 `OPEN` 后结束并判定；因此预接近、接触前和释放后的自由落体都不污染共同搬运窗口。

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

结果：`fr3_dual_palletize` 编译成功，`task14_shared_box_geometry_monitor` 已被 ament 索引发现；尚未执行 Isaac 运行时测试。

## 后续边界

Task14-A 只建立连续误差基线。若连续误差超限，Task14-B 再根据误差类型选择相对位姿反馈、柔顺、负载分配或局部重规划；不预设不必要的控制复杂度。
