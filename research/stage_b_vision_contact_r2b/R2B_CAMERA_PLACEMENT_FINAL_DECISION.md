# R2-B 后墙相机最终决策

## 决策

**`REAR_WALL_CAMERA_PARTIAL`**

## 支持理由

1. 18 个固定后墙候选完成了 RGB、深度、ArUco、视场和几何清距审计；z≥.80 的候选全部得到 marker `PRESENT`，而 z=.40 候选存在 UNKNOWN，且 rear_10/13/16 的深层墙射线被判定为离线遮挡。
2. 新的 physical04 在同一固定相机下实际观测到了 cube supply、dual-arm approach、attachment 和 coordinated transport，并保存了同步 RGB/深度/ArUco/poses/q/dq/TCP/commands/suction/GT/phase/contact trace。
3. authored 投影的系统垂直偏差可由离线 `cy≈341` 显著降低，但仍有残差；所以相机对部分任务可用，尚不能宣称完整任务可靠。
4. physical04 在 SIDE_COMPACTION preflight 因双侧吸盘为 OPEN 安全中止，后五个阶段均为 NOT_EXECUTED。该结果不支持物理 recovery 或 jamming 成功结论。

## 推荐配置

后续物理验证优先使用安全外部参考 `rear_11=[1.70,-.35,.80]` 或 `rear_17=[1.70,.35,.80]`，两者箱体清距均 `.380 m`；rear_02 的 `.130 m` 清距只保留为本次观测对照。先完成多视角内参/外参/视口一致性校准和未参与拟合的验证帧，再考虑在线使用校准参数。

## 边界与停止条件

本分支没有启动 RL、E2 优化、GNN、自动 handoff 或 jam recovery，也没有改碰撞约束、原实验阈值或主开发分支。下一项物理工作应等待主开发会话确认接触观测和可执行 recovery skill；若缺少该 primitive，结论保持 `INCONCLUSIVE`，而不是构造失败样本。

证据与代码均留在 `research/stage-b-rear-wall-camera-validation`；关联问题：[Issue #4](https://github.com/dreamerToTOE/dual-arm-embodied-palletizing/issues/4)。
