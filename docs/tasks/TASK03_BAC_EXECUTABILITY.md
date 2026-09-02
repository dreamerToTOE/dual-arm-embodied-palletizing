# Task 03 — B-A-C 放置可执行性

## 状态

🟡 **进行中 — 构造“箱体几何可放，但夹爪不可插入”的 B-A-C 场景**

Task 02 已完成第一版 Placement Skill。Task 03 开始正式验证论文中的核心问题：

> **目标箱 A 的几何体能够放入目标空位，并不代表携带 A 的机械臂夹爪也能完成插入、释放和退出。**

---

## 1. 核心实验思想

保持 Task 02 的目标箱 A：

```text
A size   = 30 × 30 × 30 mm
A target = (0.65, -0.15, 0.065) m
Yaw      = 0
```

在 A 目标位置两侧沿世界 Y 方向放置邻箱 B、C。

第一版使用：

```text
B size   = 30 × 30 × 30 mm
B center = (0.65, -0.115, 0.065) m

C size   = 30 × 30 × 30 mm
C center = (0.65, -0.185, 0.065) m
```

B、C 内侧表面的净间距：

```text
d_BC = 40 mm
```

A 沿 Y 方向宽度：

```text
w_A = 30 mm
```

因此单看箱体几何：

```text
d_BC > w_A
40 mm > 30 mm
```

A 可以放入，左右各有约 5 mm 几何间隙。

但真实放置时，携带 A 的 Franka 两指夹爪占用空间大于 A 本体。预期在垂直插入过程中，finger / hand 的扫掠体会与 B 或 C 发生碰撞，使 Placement Skill 的：

```text
C_reach = PASS
C_insert = FAIL
RESULT = INSERT_FAILED
```

---

## 2. 与论文条件的对应

简化几何条件：

```text
d_BC >= w_A
```

只能说明箱体 A 本体在几何上有空间。

机器人实际可执行条件应进一步考虑夹爪厚度和安全间隙：

```text
d_BC >= w_A + 2 t_g + 2 delta
```

其中：

```text
t_g   = 单侧夹爪 / 手指额外占用厚度
delta = 安全间隙
```

Task 03 第一版先使用 MoveIt 的真实 FR3 碰撞模型直接判定，不急于人为给出 t_g 的精确常数。

后续可以从实验结果或模型包络进一步提取等效夹爪宽度。

---

## 3. 实验结构

```text
抓取 A
↓
LIFT
↓
携带 A 到 B/C 上方 PRE_PLACE
↓
C_reach
↓
保持 x/y/姿态，Cartesian 垂直下降
↓
MoveIt 检测 FR3 finger / hand / attached A 与 B/C 的碰撞
↓
C_insert
↓
PASS 或 FAIL
```

第一轮目标不是强行让动作成功，而是验证 Placement Skill 能否正确识别“几何上有空位、机器人却插不进去”。

---

## 4. Isaac Sim 场景要求

继续使用 Task 02 场景，并新增：

```text
/World/BoxB
/World/BoxC
```

第一版 B、C 作为静态环境障碍物：

```text
Collider    = ON
Rigid Body  = OFF
```

这样即使真实机械臂发生接触，B、C 也不会被撞走，便于稳定复现实验。

A 仍然是当前 `/World/PickCube`，保留：

```text
Rigid Body + Collider
Mass = 0.2 kg
```

---

## 5. MoveIt Planning Scene 要求

仅在 Isaac 中加入 B/C 不够。

Task 03 节点必须同时把 B、C 作为 `CollisionObject` 加入 MoveIt Planning Scene，否则 MoveIt 不会在规划阶段知道邻箱存在。

规划层场景应包含：

```text
table
pick_cube / attached A
box_b
box_c
```

B、C 的位置和尺寸必须与 Isaac 完全一致。

---

## 6. 源码策略

不修改已经验证成功的：

```text
single_arm_pick_place.cpp
placement_skill_demo.cpp
```

Task 03 新建独立测试节点：

```text
ros_ws/src/fr3_moveit_test/src/bac_placement_test.cpp
```

它复用 Task 02 的 Placement Skill 流程，只增加 B/C 场景建模和 Task 03 日志。

这样 Task 01、Task 02 始终保留为稳定回归基线。

---

## 7. 第一轮验收标准

第一轮 B/C 间距设置为 40 mm。

预期：

```text
A 本体 30 mm，可以几何放入
C_reach = PASS
C_insert < 0.999
RESULT = INSERT_FAILED
```

这将成为项目中第一个直接证明：

```text
几何可放置
≠
机器人可执行放置
```

的仿真案例。

如果 40 mm 间距仍然可以完整插入，则不改变整体方法，只进一步减小 B/C 间距，同时始终保持：

```text
d_BC > 30 mm
```

直到得到“箱体可放、夹爪不可插”的临界场景，并记录临界范围。

---

## 8. 与 Task 04 的衔接

Task 03 只负责发现：

```text
当前 B-A-C 顺序导致 A 无法执行插入
```

Task 04 再处理：

```text
如果先放 B/C 导致 A 后续不可插入
↓
调整码垛顺序
↓
例如优先放 A，再放 B/C
```

因此 Task 03 是 Task 04“放置顺序调整”的直接前置实验。
