# R2-B 执行阶段证据校正

本报告纠正上一版 physical04 的几何阈值阶段标签。原始 `visual_qualification.json` 保留；校正结果单独写入每个运行目录的 `execution_phase_receipts.json`。阶段来源是 `task27_executor.log` 的事件收据，不使用 Cube GT 的 x 坐标决定在线阶段。

## 新 episode

`r2b_rear_wall_physical05` 使用 rear_11、每 6 个物理步请求一次 RGB-D。executor exit 为 `1`，因此 runner 的 `status=COMPLETED` 只表示 runner 收尾，不能表示 manipulation PASS。实际结果是：16/16 PUSH slices 完成、deep-wall seat 完成，随后 SIDE_COMPACTION preflight 被阻塞。

| 技能 | executor 收据 | 物理步区间 | 相机样本 | 校正分类 |
|---|---|---:|---:|---|
| cube supply | batch→selected candidate | 536–606 | 0 | EXECUTED_WITHOUT_CAMERA_SAMPLES |
| dual-arm approach | selected candidate→COMMON_LIFT | 607–2913 | 0 | EXECUTED_WITHOUT_CAMERA_SAMPLES |
| dual-arm attachment | COMMON_LIFT→LIFT GT | 2914–3406 | 0 | EXECUTED_WITHOUT_CAMERA_SAMPLES |
| coordinated transport | COMMON_X_TRAVEL→Y_ALIGN GT | 3410–4123 | 0 | EXECUTED_WITHOUT_CAMERA_SAMPLES |
| placement/release | DROP_TO_TABLE→release receipt | 4844–5024 | 0 | EXECUTED_WITHOUT_CAMERA_SAMPLES |
| release retreat | retreat→双臂退出 | 5031–6061 | 2 | EXECUTED_WITH_CAMERA_SAMPLES |
| single-arm recontact | REGRASP→after REGRASP | 6232–7095 | 11 | EXECUTED_WITH_CAMERA_SAMPLES |
| single-arm insertion | PUSH 1/16→PUSH complete | 7221–8976 | 0 | EXECUTED_WITHOUT_CAMERA_SAMPLES |
| deep-wall arrival | PUSH complete→support seat | 8977–9037 | 0 | EXECUTED_WITHOUT_CAMERA_SAMPLES |
| auxiliary side compaction | SIDE_HIGH_APPROACH→preflight error | 9072–9081 | 0 | PREFLIGHT_BLOCKED |

各阶段使用自己的 executor 时间区间和分母；相机帧不跨阶段求和。`q_actual`、`dq_actual`、TCP、Cube、吸盘和 command receipt 均来自 physical trace；Cube/PhysX contact 只作为离线评价通道。

## 原始物理结果

physical04 的日志同样显示 COMMON_DROP_TO_TABLE、REGRASP、16/16 PUSH 和深墙座入；因此之前把这些阶段全部写成 `NOT_EXECUTED` 是错误的。physical04 的 side-compaction 碰撞深度为 `0.571 mm`，physical05 重复为 `0.444 mm`。两次都是 t=0 FCL pair `right_fr3_side_suction <-> task26_cube_1`。
