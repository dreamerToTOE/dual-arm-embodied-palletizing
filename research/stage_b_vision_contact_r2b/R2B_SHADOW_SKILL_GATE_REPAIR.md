# R2-B shadow skill gate 修复

旧 selector 只看 E1 状态，physical07 产生 364 个 `CONTINUE_PUSH`，其中 172 个
位于 `single_arm_insertion` 之外。修复后 selector 要求：

- `CONTINUE_PUSH`：current executor skill 必须为 `single_arm_insertion`，controller
  owner、camera/q/dq/TCP/command 必须 PRESENT，左右 attachment confidence 必须
  已独立提供且不低于门限。
- `REQUEST_DEEP_WALL_CHECK`：只允许 single-arm insertion 或 deep-wall arrival，且
  owner、观测和 command 有效。
- `SIDE_APPROACH_READY`：只允许 auxiliary side compaction，并同时要求完整观测和
  明确通过的 side preflight。
- contradictory skill receipt、UNKNOWN/STALE、缺失 owner 或不确定 attachment
  一律 `HOLD_UNKNOWN`。

在相同 physical07 重放中，修复后 1,550 帧全部为 `HOLD_UNKNOWN`；非插入阶段
`CONTINUE_PUSH` 为 0。插入阶段也没有继续推进，因为该 trace 没有独立
attachment confidence。这是安全拒绝，不是接触成功结论。selector 仍只读，未写入
controller、运动或 suction。
