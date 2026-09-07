# Task 04-B.2 — 持物状态判定、掉落恢复与码垛纠偏

## 状态

🟡 设计确认，待实现与仿真验证。

Task04-B 已完成 9 Cube 的 X/Y/Z 三维码垛与动态最高表面放置。随后加入仅依赖 `/task04b/suction_state` 的 GRASP_LOST 检测，但连续 6 次测试均出现失败，其中存在 Isaac 中 Cube 仍实际保持在吸盘下方、控制器却误判掉落的情况。因此该 Bool 不能继续作为唯一掉落判据。

---

## 1. 设计结论

后续不再使用：

```text
suction_state == false
=> 立即判定 Cube 已掉落
```

改为“物体实际位姿相对吸盘 TCP 的几何一致性”作为主要持物判据，`suction_state` 只保留为辅助状态。

Isaac Sim 当前直接提供 Cube Ground Truth，因此本阶段暂不加入相机。以后接入 RGB-D 时，只替换物体位姿来源，不改变恢复状态机。

---

## 2. 持物状态判定

Bridge 新增发布吸盘 TCP 世界位姿，例如：

```text
/task04b/suction_tcp_pose
```

抓取成功后记录：

```text
T_tcp_cube_ref
```

运输期间持续比较：

```text
当前 T_tcp_cube
vs
抓取成功时 T_tcp_cube_ref
```

主要检测量：

```text
位置误差 ||p_tcp_cube - p_tcp_cube_ref||
姿态误差 angle(R_ref^-1 R_now)
```

只有当相对位姿误差持续超过阈值一段时间，才确认 `GRASP_LOST`。

`/task04b/suction_state` 不再单独触发停机。

---

## 3. 掉落后的自动恢复

确认真实掉落后：

```text
GRASP_LOST
↓
停止当前轨迹
↓
机械臂保持 / 安全抬升
↓
等待 Cube 在 PhysX 中落稳
↓
读取该 Cube 最新 Isaac Ground Truth
↓
将真实掉落位姿重新加入 MoveIt Planning Scene
↓
重新规划 PRE_PICK
↓
重新抓取
↓
重新执行原目标的放置任务
```

第一版设置有限重试次数，建议：

```text
MAX_RECOVERY_ATTEMPTS = 2
```

超过次数则任务失败并停止，避免无限循环。

如果 Cube 落稳后明显翻倒、顶面不再近似水平，则第一版不自动恢复，直接报错停止。后续再扩展任意姿态抓取。

---

## 4. 已码垛 Cube 的质量检查与重新码垛

每次释放并等待 settle 后，读取实际位姿：

```text
p_actual
```

与目标位姿比较：

```text
XY 偏差
Z 偏差
yaw 偏差
```

若超过允许误差，则标记：

```text
MISALIGNED
```

随后执行：

```text
读取该 Cube 当前真实位姿
↓
重新抓取
↓
重新计算目标 XY 的最高支撑表面
  （高度查询时排除该 Cube 自身）
↓
重新放置
↓
再次检查 settle pose
```

为避免来回抖动，纠偏同样限制最大尝试次数。

重要约束：支撑 Cube 的纠偏应在其上方尚未放置更高层 Cube 时完成；一旦其已经成为上层 Cube 的支撑，不直接抽走底层 Cube。

---

## 5. Task04-B.2 验收标准

```text
1. Cube 实际仍随吸盘运动时，不因 suction_state 瞬时异常误报 GRASP_LOST；
2. 真正掉落时，基于 Cube/TCP 相对位姿能够确认掉落；
3. 掉落后读取 Isaac 最新位姿并自动重新规划抓取；
4. 恢复抓取后继续完成原目标放置，而不是直接进入下一 Cube；
5. 已放置 Cube 若偏差超阈值，可在上层建立前自动重新码垛；
6. 恢复 / 纠偏均有最大重试次数，不无限循环；
7. 最终状态与 MoveIt Planning Scene 使用 Isaac 真实 settle pose 同步。
```

---

## 6. 与后续双臂系统的关系

Task04-B.2 完成后，再进入第二台 FR3 和双臂松协调。

后续每个机械臂都应具备独立执行状态，例如：

```text
IDLE
PICKING
CARRYING
PLACING
GRASP_LOST
RECOVERING
REWORKING
FAILED
```

这样双臂调度层不会建立在“抓住后永远不会掉、放下后永远不会偏”的理想假设上。
