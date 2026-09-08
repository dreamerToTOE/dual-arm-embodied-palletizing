# Task 04-B.2 — 基于实际/期望位姿偏差的恢复与重新码垛

## 状态

🟡 设计确认，待实现与仿真验证。

Task04-B 已完成 9 Cube 的 X/Y/Z 三维码垛与动态最高表面放置。随后仅依赖 `/task04b/suction_state` 的 GRASP_LOST 检测在连续测试中出现明显误判，因此后续不再把 Surface Gripper Bool 作为唯一掉落依据。

---

## 1. 统一设计思想

后续把“中途掉落”和“最终码垛偏差”统一成同一类问题：

```text
实际 Cube 位姿
vs
当前阶段期望 Cube 位姿
↓
计算位姿偏差
↓
偏差超过阈值并持续成立
↓
触发恢复 / 重新抓取
```

关键点：运输过程中不能拿 Cube 当前实际位置直接与最终码垛目标比较，因为此时 Cube 本来就在运动途中。

因此“预定位姿”必须随执行阶段变化：

```text
CARRYING：期望位姿 = 由当前吸盘 TCP 位姿 + 抓取时相对变换推算出的 Cube 当前应在位姿
PLACED / QUALITY_CHECK：期望位姿 = 最终码垛目标位姿
```

这样中途掉落和最终位置偏差可以共用一个“pose residual”框架。

Isaac Sim 当前直接提供 Cube Ground Truth，因此本阶段暂不加入相机。以后接入 RGB-D 时，只替换实际 Cube 位姿来源，不改变恢复逻辑。

---

## 2. 运输阶段：实际位姿 vs 当前期望位姿

抓取确认后记录：

```text
T_tcp_cube_ref
```

运输过程中由当前吸盘 TCP 位姿计算 Cube 当前期望位姿：

```text
T_world_cube_expected
=
T_world_tcp_current
*
T_tcp_cube_ref
```

同时从 Isaac Ground Truth 得到：

```text
T_world_cube_actual
```

计算：

```text
位置偏差 e_p
姿态偏差 e_R
```

若偏差持续超过阈值，则确认：

```text
GRASP_LOST / CARRY_ERROR
```

`/task04b/suction_state` 只作为辅助信息，不单独触发停机。

---

## 3. 最终码垛阶段：实际位姿 vs 目标位姿

每次释放并等待 PhysX 落稳后，读取：

```text
T_world_cube_actual
```

与该 Cube 的目标位姿比较：

```text
T_world_cube_target
```

重点检查：

```text
XY 偏差
Z 偏差
yaw 偏差
```

若超过阈值，则标记：

```text
MISALIGNED
```

并进入重新码垛流程。

---

## 4. 掉落 / 偏差后的统一恢复流程

无论是运输中掉落，还是最终码垛偏差过大，都尽量复用同一个恢复入口：

```text
RECOVERY / REWORK
↓
停止当前动作
↓
获取该 Cube 最新 Isaac Ground Truth
↓
同步 MoveIt Planning Scene
↓
重新规划 PRE_PICK
↓
重新抓取
↓
确认抓取
↓
重新执行该 Cube 原目标任务
↓
再次质量检查
```

第一版限制恢复次数，例如：

```text
MAX_RECOVERY_ATTEMPTS = 2
MAX_REWORK_ATTEMPTS = 2
```

超过次数则标记 FAILED，避免无限循环。

如果 Cube 已明显翻倒、顶面不再近似水平，第一版直接报错停止，不自动做任意姿态抓取。

---

## 5. 动态最高表面与重新码垛

对已经放置但位置偏差过大的 Cube 重新抓取时，重新计算目标 XY 的最高支撑表面。

必须排除当前正在返工的 Cube 自身，否则高度查询会把它错误当成支撑物，导致重新放置高度多抬一层。

正确逻辑：

```text
computeSupportSurfaceZ(target_x, target_y, exclude_cube_id)
```

另外，支撑 Cube 的纠偏应在其上方尚未形成更高层之前完成；一旦已经成为上层 Cube 的支撑，不直接抽走底层 Cube。

---

## 6. 第一版建议判据

不预先把阈值写死为论文结论，先记录正常运输与正常落位数据，再根据实测分布设置。

开发阶段可先采用分阶段阈值：

```text
CARRYING：位置偏差超过较大阈值并持续一段时间才判掉落，优先避免误报
QUALITY_CHECK：XY / Z / yaw 使用更严格阈值判断码垛质量
```

具体数值通过 Isaac 实际数据确定，而不是继续由 `/task04b/suction_state` 推断。

---

## 7. Task04-B.2 验收标准

```text
1. Cube 实际仍跟随吸盘运动时，不误判掉落；
2. 真正掉落时，实际/期望位姿偏差能够稳定检测；
3. 掉落后读取最新 Isaac 位姿并自动重新抓取；
4. 恢复后继续完成原 Cube 目标，而不是跳到下一 Cube；
5. 最终放置偏差超阈值时自动进入重新码垛；
6. 重新码垛时动态最高表面计算排除该 Cube 自身；
7. 恢复 / 纠偏均有限次重试；
8. 最终 MoveIt Planning Scene 与 Isaac 实际 settle pose 保持一致。
```

---

## 8. 与后续双臂系统的关系

Task04-B.2 完成后，再进入第二台 FR3 和双臂松协调。

后续每个机械臂独立维护：

```text
IDLE
PICKING
VERIFY_GRASP
CARRYING
PLACING
QUALITY_CHECK
RECOVERING
REWORKING
FAILED
```

双臂调度层只使用已经过执行状态验证的结果，而不是假设“抓住后永不掉落、放置后永远准确”。
