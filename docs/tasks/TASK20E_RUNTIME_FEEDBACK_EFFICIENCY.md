# Task20-E：运行时反馈闭环与效率基准

状态：🟡 已实现并通过静态构建；等待 Isaac 物理验收。

## Task20-D 未通过记录

Task20-D 在固定 replay `seed=20260922` 中完成了两件物体：

```text
task 1: TIGHT_SHARED_OBJECT  PASS
task 2: LOOSE_*              PASS
task 3: LOOSE_RIGHT          FAIL
```

第三件 `task18_box_05` 的 release Ground Truth 为：

```text
target = (0.6897, -0.0100, 0.1700)
actual = (0.5993, -0.0291, 0.1701)
xy error = 92.411 mm
z  error = 0.123 mm
```

所以该次绝不是“规划通过即可视为物理通过”。原执行器在名义轨迹时间到达时直接发送
`SUCTION_OFF`，没有确认 Isaac suction TCP 已实际到达 PLACE。根据该日志，尚不能
断言偏差来自关节跟踪滞后还是释放后的物理滑移/碰撞；两种原因必须由真实 TCP 日志
区分。Task20-D 保持未通过，不把前两件成功外推为连续调度成功。

## Task20-E 改动

### 1. 释放前真实 TCP 闸门

松协调 primitive 在 `SUCTION_OFF` 前订阅对应：

```text
/task20/left/suction_tcp_pose
/task20/right/suction_tcp_pose
```

并等待真实 TCP 到达候选的 planned release point（位置误差 `<= 6 mm`，最长 `3 s`）。
到位后打印：

```text
RELEASE TCP READY: expected=..., actual=..., error=... mm
```

未到位则打印 `RELEASE TCP NOT READY`、拒绝按名义时间发送 `SUCTION_OFF`，并保持
`SUCTION ON` 以保留失败现场（随后人工重置场景）。这既是安全闸门，也是根因定位：
若 TCP 已到位但 Box 仍显著偏移，才应检查吸盘/物理碰撞；若 TCP 未到位，则优先处理
实际执行跟踪。

### 2. 中等尺寸、非规则可复现源布局

新场景 `task20e_runtime_feedback_scene.py` 复用 Task18 的自包含 Task06 基线，但替换
Task11 的 `0.220 x 0.320 x 0.080 m` 大件为：

```text
shared box size = (0.160, 0.240, 0.060) m
mass            = 0.70 kg
dual grasp y    = -0.075 / +0.075 m
```

它与两个 `0.040 x 0.040 x 0.080 m` tall box 组成 3 件 episode。源位姿从
`seed=20260924` 的分离可达区域采样，因而不规则但可复现；控制端仍仅消费
`/task18/box_states`，不读取固定源坐标。放置区也调整为不以旧 `(0.650, 0.000)`
为中心的非对称区域。

### 3. 性能剖析

每件成功后输出：

```text
Task20-E PROFILE task=...
  placement=<s>
  candidates=<n>
  route_calls=<n>
  route=<s>
  execution=<s>
  task_wall=<s>
```

episode 结束还输出总 wall time。先以实际数据区分 Placement、OMPL/Router/FCL 与
Isaac 执行的耗时，再决定缓存或 planner 调整的收益；不以直觉更换为 RRT*。

## 当前优化路线（DRL 暂缓）

```text
正确性：真实 TCP release gate + 物理误差记录
    ↓
测量：逐件 profile
    ↓
经典加速：IK/primitive/轨迹缓存、快速 arm 可达性筛选、
            简单段优先 Cartesian/Pilz，复杂段 RRTConnect fallback
    ↓
离线全局任务排序 + 在线 Ground Truth 校验/局部重规划
    ↓
DRL（暂缓）：仅作为物体顺序、机械臂分配、placement candidate 的高层提议器，
              始终经 MoveIt/FCL 安全闸门验证
```

DRL 不用于替代底层关节避障或物理安全验证；在没有经典缓存与可比较耗时基线之前，
直接训练策略只会把在线规划成本转移为更高的离线数据和调参成本。

## Task20-E 通过判据

1. 每个 loose release 前出现 `RELEASE TCP READY`，误差不大于 `6 mm`；
2. 每件都输出 `PROFILE`；
3. 三件依次 `WORLD COMMITTED`；
4. 最终输出：

```text
Task20-E CONTINUOUS EXECUTION PASS: completed=3/3 tight=1 loose=2
Task20-E PROFILE episode_wall=...
Task20 runtime EXECUTION PASS
```

任一 `RELEASE TCP NOT READY`、放置误差超限或 FCL 拒绝均为未通过，不扩大 ACM、
不增大吸附阈值、不手动移动物体绕过。
