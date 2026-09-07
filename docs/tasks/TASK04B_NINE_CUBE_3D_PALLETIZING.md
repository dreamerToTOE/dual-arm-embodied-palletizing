# Task 04-B — 九 Cube 三维码垛与动态最高表面放置

## 状态

🟡 **待本机编译 / 仿真验证。**

Task04-A 已基本打通顶部吸盘三 Cube 单层码放。Task04-B 不继续做 0 mm 缝隙实验；保持当前 2 mm 水平安全间隙，优先验证更多 Cube、X/Y 平面展开、Z 方向叠层，以及动态放置高度计算。

> 若最后老师明确要求“零缝隙码垛”，再单独追加后续验证，不在 Task04-B 中提前引入该变量。

---

## 1. Task04-B 目标

使用单台 FR3 + 固定顶部吸盘，依次搬运 9 个随机初始位置的 Cube，完成一个同时覆盖 X / Y / Z 三个方向的码垛结构。

验证重点：

```text
X：三个不同横向码放位置
Y：两排码放位置
Z：第二层叠放
```

9 个 Cube 的目标结构：

```text
底层：3 x 2 = 6 个 Cube
第二层：在第一排三个位置继续叠 3 个 Cube
总计：9 个 Cube
```

水平相邻中心距仍保持：

```text
32 mm
```

Cube 边长：

```text
30 mm
```

因此 Task04-B 继续保留：

```text
2 mm 水平安全间隙
```

---

## 2. 关键变化：放置 Z 不再写死

Task04-A 第一版把放置高度直接写成第一层固定值：

```text
Cube center Z = 0.065 m
```

这只适用于直接放在桌面上的单层码放。

Task04-B 改为：

```text
给定目标 XY
↓
查询该 XY 当前位置的最高支撑表面
↓
根据最高表面计算本次 Cube 的释放高度
```

核心关系：

```text
support_surface_z
= max(
    table_top_z,
    已成功放置且位于该目标 XY 下方的 Cube top_z
  )
```

然后：

```text
planned_release_center_z
= support_surface_z
+ cube_height / 2
+ release_gap
```

当前：

```text
release_gap = 1 mm
```

这 1 mm 的作用不是码垛层间永久保留间隙，而是避免 MoveIt 在规划 PLACE 终点时把 Attached Cube 与支撑表面的有意接触直接判定为碰撞。

实际过程：

```text
MoveIt：规划到支撑表面上方 1 mm
↓
SUCTION OFF
↓
Isaac PhysX：Cube 下落约 1 mm
↓
Cube 落到真实最高表面
↓
Bridge 发布真实 settle pose
↓
MoveIt 按真实 settle pose 把该 Cube 重新加入 Planning Scene
```

因此最终物理堆叠仍由真实支撑面决定，而不是永久悬空 1 mm。

---

## 3. “最高表面”不是预先写死第二层 Z

Task04-B 不使用：

```text
Cube7~9 center Z = 0.095 m
```

这种固定第二层高度作为控制逻辑。

而是使用已经完成放置的 Cube 的最新 Isaac Ground Truth：

```text
/task04b/cube_poses
```

控制器只把：

```text
index < 当前 Cube
```

的已成功放置 Cube 视为候选支撑物。

在目标 XY 处查询最高 top surface，从而得到下一次放置高度。

这相当于第一版简化的局部高度图 / vertical-column height query：

```text
(x, y) -> current highest surface z
```

以后如果扩展到不同尺寸箱体、视觉高度图或更复杂码垛策略，可以在这个接口上继续升级。

---

## 4. 九 Cube 目标 XY

Task04-B 目标 XY：

```text
Cube1 -> (0.620, -0.134)
Cube2 -> (0.652, -0.134)
Cube3 -> (0.684, -0.134)

Cube4 -> (0.620, -0.166)
Cube5 -> (0.652, -0.166)
Cube6 -> (0.684, -0.166)

Cube7 -> (0.620, -0.134)
Cube8 -> (0.652, -0.134)
Cube9 -> (0.684, -0.134)
```

前六个先完成 3x2 底层。

Cube7~9 再次使用第一排三个 XY，因此此时控制器必须检测到对应下层 Cube 的最高表面，而不是继续使用桌面高度。

如果一个理论上应该有下层支撑的目标位置却没有检测到有效支撑，控制器直接停止，避免错误把第二层 Cube 放回桌面。

---

## 5. Isaac 场景

新脚本：

```text
isaac/scripts/task04b_nine_cube_scene.py
```

功能：

```text
1. 保留当前 FR3 + 顶部 suction_tool；
2. 创建 Cube1 ... Cube9；
3. 9 个 Cube 初始位置随机；
4. 初始 Cube 全部位于桌面第一层；
5. 取料区沿用 Task04-A 已验证范围；
6. 9 Cube 时随机最小中心距改为 45 mm；
7. 只打印目标 XY，不给目标 Z 写死常量。
```

---

## 6. Task04-B Isaac ROS Bridge

新脚本：

```text
isaac/scripts/task04b_suction_ros_bridge.py
```

ROS 接口：

```text
SUB /task04b/suction_command
PUB /task04b/suction_state
PUB /task04b/cube_poses
```

`PoseArray` 顺序固定：

```text
Cube1 ... Cube9
```

Bridge 保留 Task04-A 已实际验证有效的吸盘主动重试逻辑：

```text
PRE_PICK 时 SUCTION ON
↓
第一次 close() 可能因距离过远失败
↓
desired_closed=True 且尚未 CLOSED
↓
每 50 ms 主动重试 close()
↓
机械臂下降进入 gripThreshold 后吸附成功
```

---

## 7. ROS / MoveIt 控制节点

新源码：

```text
ros_ws/src/fr3_moveit_test/src/suction_nine_cube_palletize.cpp
```

与 Task04-A 相比，主要变化：

```text
3 Cube -> 9 Cube
固定 place Z -> 动态最高表面 Z
目标理论位置回填 -> Isaac 真实 settle pose 回填 MoveIt
固定第一层 PRE_PLACE -> 随堆叠高度动态抬高
```

同时明确保留：

```text
MoveIt planning tip = fr3_link8
cobot_pump TCP offset = 0.105 m
```

所有吸盘 TCP 世界 Z 都先转换为 `fr3_link8` 世界 Z 再交给 MoveIt。

Attached Cube 也按：

```text
link8 -> cobot_pump TCP -> Cube center
```

的真实几何关系建模，而不是把 Cube 错误附着在 flange 附近。

---

## 8. Task04-B 验收标准

第一轮只验证下面这些，不加入 0 mm 水平缝隙变量：

```text
1. Isaac 随机生成 9 个互不重叠 Cube；
2. Bridge 稳定发布 9 个 Ground Truth 位姿；
3. Cube1~6 完成 3x2 第一层；
4. 第一层 X / Y 两个方向位置均正确；
5. Cube7~9 放置前打印的 support_surface_z 明显高于 table_top_z；
6. Cube7~9 自动形成第二层，不使用固定第二层 Z；
7. 每次释放后使用 Isaac settle pose 重新加入 MoveIt Planning Scene；
8. 搬运过程中无脱落；
9. 不发生明显吸盘 / 已码放 Cube 非法碰撞；
10. 九个 Cube 完成后机械臂返回 HOME。
```

当前状态：

```text
🟡 代码已写入 GitHub，待本机 CMake 接入、编译和 Isaac 实际验证。
```
