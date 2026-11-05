# R2-B Calibration / Phase / Skill-Handoff 最终门

## 决策

**`R2B_CAL_PHASE_PARTIAL`**

## 已建立

- 新隔离分支从 `72af1f2735926e43c9674a9f824d22aeb58a70a7` 创建。
- C0 已将阶段标签改为 executor 收据，并分离 runner、manipulation、camera 三种结果。
- 新 physical episode 数：1（rear_11，约 10 Hz 请求/实测中位间隔 100 ms）。
- 16/16 insertion slices 和 deep-wall seat 实际完成；placement/release、regrasp、insertion、deep-wall 不再错误标成未执行，但多数没有相机样本。
- 独立静态 train/holdout + physical05 holdout 校准已完成，结果仍为厘米级残差，不能 PASS。
- rear_11 选为当前最佳已有动态证据位置；rear_17 仍是静态候选，不是已验证的动态替代。
- side-compaction transition 根因明确为 t=0 FCL 初始重叠，physical05 `.444 mm`、physical04 `.571 mm`；安全 handoff 仍 BLOCKED。
- P0/P1 在真实 blind interval 上有可计算但不适合 3 mm 的误差；P2 因附着证据不足保持 UNKNOWN。

## 未通过的门

相机全任务阶段覆盖不足；多视角校准仍未独立验证完整 K/T_world_camera/marker-to-Cube；相机柱、支架、线缆和全 FR3 link 的 FCL swept certificate 未建立；side-compaction 没有安全可执行路径；E2 尚未进入研究评价。

## 下一研究门

先补充非共面多 marker 的 held-out 标定，随后在不修改碰撞和安全阈值的前提下，用显式 handoff primitive 重新验证 side-compaction 初始状态；只有获得安全 preflight 和阶段同步相机样本后，才进入后续 E2 估计研究。RL、GNN、物理 jam recovery 和自动 handoff 继续暂停。
