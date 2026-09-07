# Task 04 — 顶部吸盘单臂三 Cube 码垛基线

## 状态

🟡 **进行中 — 吸盘几何已在 Isaac 中建立，开始三 Cube 随机取料与依次码垛。**

## 1. 目标

当前阶段不再只做单 Cube Pick & Place，而是直接验证：

> 单台 FR3 使用固定顶部吸盘，从三个随机取料位置依次抓取三个 Cube，并在目标区域完成一层紧密码放。

暂不做双臂、不做紧协调控制、不提前绑定论文结构。

## 2. 固定顶部吸盘设计

后续左右 FR3 使用同一种固定顶部吸盘：

```text
小箱体：单臂顶部单吸附
大箱体：双臂在同一箱体上表面两个分离吸附点共同吊运
```

Isaac 中保留 `fr3_hand` / finger joint 的 articulation 结构，但隐藏原 Franka Hand/Finger 的视觉并关闭其物理碰撞；实际末端几何使用紧凑 `suction_tool`。

第一版 MoveIt 优先使用 Franka 官方 `cobot_pump` 末端，因此 Isaac 自定义吸盘的接触 TCP 也设置到 hand 局部 +Z `0.105 m`，使规划层和物理层的工具长度基本一致。

Isaac 4.5 的真实吸附采用 `isaacsim.robot.surface_gripper`，不模拟真空流体，只模拟吸附接触、保持与释放。

## 3. 三 Cube 随机场景

场景脚本：

```text
isaac/scripts/task04_three_cube_scene.py
```

功能：

```text
1. 清理 PickCube / BoxB / BoxC；
2. 保留 FR3 articulation；
3. 重建顶部 suction_tool；
4. 在取料区随机生成 Cube1 / Cube2 / Cube3；
5. 三个 Cube 均为 30 mm、Rigid Body + Collider、0.2 kg；
6. 自动保证随机初始位置之间存在安全距离；
7. 打印三个随机抓取位置和三个固定码放目标。
```

第一版码放目标中心：

```text
Cube1 -> (0.620, -0.150, 0.065)
Cube2 -> (0.652, -0.150, 0.065)
Cube3 -> (0.684, -0.150, 0.065)
```

中心距 32 mm，对 30 mm Cube 留约 2 mm 安全间隙。后续稳定后再继续缩小间隙。

## 4. Isaac 吸盘 ROS Bridge

桥接脚本：

```text
isaac/scripts/task04_suction_ros_bridge.py
```

运行条件：Isaac 已点击 Play。

ROS 接口：

```text
SUB /task04/suction_command   std_msgs/Bool
PUB /task04/suction_state     std_msgs/Bool
PUB /task04/cube_poses        geometry_msgs/PoseArray
```

`cube_poses` 顺序固定：

```text
Cube1, Cube2, Cube3
```

吸盘设置 `retryClose=True`，因此控制器可以按照当前设计：

```text
PRE_PICK
→ SUCTION ON
→ Cartesian 直线下降
→ 接近 Cube 后自动建立吸附
```

而不是先接触再开启吸盘。

## 5. ROS / MoveIt 控制节点

源码：

```text
ros_ws/src/fr3_moveit_test/src/suction_three_cube_palletize.cpp
```

每个 Cube 的执行链：

```text
RRTConnect -> PRE_PICK
→ SUCTION ON
→ MoveIt 临时移除目标 Cube（允许有意接触）
→ Cartesian PRE_PICK -> SUCTION_CONTACT
→ 等待 Isaac suction_state=CLOSED
→ MoveIt AttachedCollisionObject
→ Cartesian LIFT
→ RRTConnect -> PRE_PLACE
→ Cartesian PLACE
→ MoveIt DETACH + 临时 remove
→ SUCTION OFF
→ Cartesian RETREAT
→ 按最终目标重新加入 MoveIt World
```

三个 Cube 依次执行，最后返回 HOME。

## 6. MoveIt 末端模型原则

三 Cube 吸盘码垛不应继续使用原二指 Franka Hand 的碰撞模型，否则会再次产生 Task03 的虚假 finger 干涉。

当前 Franka ROS 2 / `franka_description` 支持 `ee_id:=cobot_pump`，因此优先使用官方吸盘式末端配置启动 MoveIt；若本机 `moveit.launch.py --show-args` 中存在 `load_gripper` / `ee_id`，使用：

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true \
  load_gripper:=true \
  ee_id:=cobot_pump
```

控制节点默认读取 MoveIt 当前配置给出的 end-effector link，不再硬编码 `fr3_hand_tcp`；必要时可通过 ROS 参数 `eef_link` 覆盖。

## 7. 已验证问题：`fr3_link7` 与 `fr3_cobot_pump` 安装处自碰撞

第一次运行三 Cube 控制时，`PRE_PICK` 的 RRTConnect 在约几十毫秒内直接 aborted。

排查结果：

```text
avoid_collisions = false 的 /compute_ik：SUCCESS
avoid_collisions = true  的 /compute_ik：NO_IK_SOLUTION
```

进一步使用 `/check_state_validity` 检查 HOME 和 PRE_PICK IK 解，两者均返回：

```text
valid = false
contact_body_1 / 2 = fr3_link7 <-> fr3_cobot_pump
penetration depth ≈ 0.0002867 m
```

因此根因不是随机 Cube 不可达，也不是桌面或环境障碍，而是 MoveIt 把官方 `cobot_pump` 与机器人相邻安装结构 `fr3_link7` 判定为自碰撞。因为 HOME 起点本身就是 invalid，后续 collision-aware RRTConnect 无法从合法起点开始规划，所以会立即失败。

临时工程修复：只在 Allowed Collision Matrix 中允许这一对内部安装结构碰撞：

```text
fr3_link7 <-> fr3_cobot_pump
```

注意：这不等于关闭吸盘与桌面、Cube 或邻近箱体之间的碰撞检测；环境碰撞仍保持启用。

用户实际验证后：

```text
HOME /check_state_validity
valid = true
contacts = []
```

因此该问题状态为：

```text
✅ 根因与临时修复均已验证
```

后续若继续长期使用 `cobot_pump`，应把这一相邻安装关系正式写入项目自己的 SRDF / Allowed Collision 配置，而不是每次手动注入。

## 8. 第一轮验收标准

```text
1. 三个 Cube 初始位置随机且互不重叠；
2. 控制节点自动收到三个 Cube 的 Isaac Ground Truth 位姿；
3. 每次都先到 PRE_PICK，再 SUCTION ON，再 Cartesian 下降；
4. suction_state=CLOSED 后才允许 LIFT；
5. Cube1 / Cube2 / Cube3 依次放到三个目标位置；
6. 搬运过程中不脱落；
7. 放置后 Cube 留在目标位置；
8. 三个 Cube 完成后 FR3 返回 HOME。
```

当前源码属于第一版待本机编译/运行验证；实际日志与问题在用户验证后继续记录。
