# R2-B Stage B 运行时传感器接入

本次将 physical07 的逐物理步 runtime receipt 接入 observer。视觉帧只选择
`physics_step` 和 `time_s` 均不晚于相机帧的 latest-past 行；未来行会被拒绝，
超出 0.25 s 的行标成 STALE。Cube/PhysX 真值没有进入 estimator payload。

## 覆盖与来源

- 运行时 trace：15,479 行，step 1–15,479；每行 q/dq、左右 TCP、Cube 速度、
  suction command/state 和 control owner/mode。
- 相机：1,550 帧全部成功 join；未来样本 0；runtime age 中位数和最大值均为
  0 s。相机 writer wall latency 中位数 0.166 s、最大 0.731 s，单独保留为诊断。
- controller command：1,347 个视觉帧有 receipt，203 个无 receipt；无 receipt
  的 command source health 为 UNKNOWN。原始 command 只有 wall timestamp，不能
  声称与同 physics step 严格曝光级因果。
- attachment confidence：原始 trace 没有独立测量，左右均保持 UNKNOWN；吸盘
  closed 不被解释为刚性附着。

原始文件在 sensor-pipeline repair worktree 中保存，哈希见
`evidence/r2b_rear_wall_physical07/runtime_integration/runtime_integration_summary.json`：
runtime `3b6781b0…780fcee`，contact `ed972498…0860df9`。

## 语义门

E0/E1 在视觉有效但任一 TCP 缺失或过期时返回 `UNKNOWN`，不会把视觉存在误判为
FREE。q/dq/TCP/FK 与 suction 通过 source stamp 进入 causal frame；command 和
attachment 缺失仍显式保留未知。selector 仍为 read-only，未发送运动、吸盘或
controller ownership 写入。
