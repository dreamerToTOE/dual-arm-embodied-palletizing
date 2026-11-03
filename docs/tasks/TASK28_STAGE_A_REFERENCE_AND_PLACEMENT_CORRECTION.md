# Task28 Stage A：双臂参考一致性与精密放置修正

基线为 `b8e3c39`，分支为 `side-suction-palletizing`。本轮只运行 Stage-A-only 的 Cube_01 抓取、抬升、运输和落桌前下降；没有进入 REGRASP/PUSH、Stage B 或五 Cube 续跑。

## 结果

双臂参考冲突已经修正。原始完整周期在 physics step 5432、`t=90.533333 s` 的冻结 FK 参考闭合误差峰值为 **84.4635 mm**；对象中心适配器使用一次共同的 `T_WO_des` 和捕获时冻结的 `T_O_tcp` 后，v6 的期望闭合误差最大为 **0.00018 mm**，超过 0.150 mm 的样本数为 **0**。实际双臂相对 TCP 闭合在 v6 的最大平移误差为 **0.0699 mm**。

精密放置仍未通过。v6 的真实 Cube 在 `COMMON_DROP_TO_TABLE` 结束后为 `(0.773632, 0.059585, 0.273442) m`，目标为 `(0.770, 0.060, 0.260) m`；XY 误差 **3.655 mm**，Z 误差 **13.442 mm**，Table 接触力为 0，支撑门禁保持 false，因此没有释放吸盘。v6 的 QP 全部为 `7353 QP_OPTIMAL`，执行器以 `TASK28_PLACEMENT_NOT_CONVERGED` 退出。这个结果保留了真实失败，没有把未支撑的 Cube 写成精密放置成功。

## 实现改动

- `--consistent-object-reference` 只允许 Stage-A-only、integrated、live-QP 入口。
- `RigidObjectReference` 由规划 TCP 反推出单一对象位姿，再用捕获时冻结的左右抓取变换生成两侧目标；有限差分保存对象速度/加速度，并以连续 IK 投影保持关节参考。
- `alignCartesianProgress` 按笛卡尔进度重新标定左右轨迹时间，避免两套独立 MoveIt 时间参数化在附着段产生不同对象进度。
- 增加 Cube/Table 的 PhysX 支撑接触回读和落桌前门禁；接触证据仍标记为形状接触回调，不冒充 D6 独立力传感器。
- 末段 QP 使用有限的 `common_weight=3e4` 强化。v5 的 `1e6` 版本会使负载段发散并触发 `EMPTY_ACCELERATION_OR_TORQUE_BOX`，已保留 v5 失败证据并回退到稳定配置。

## 数值对照

| 运行 | 期望闭合峰值 | 实际相对 TCP 峰值 | Cube 跟踪峰值 | 最终 XYZ 误差 (mm) | 支撑/释放 | QP |
|---|---:|---:|---:|---|---|---|
| 原始 `task28_g3_full_cycle_ff_v1` | 84.4635 mm | — | — | 原周期释放后约 10.13 mm 落差 | 已释放 | 旧周期 |
| v4，旧末段权重 | 3.5e-10 mm | 0.0646 mm | 15.10 mm | `(2.436, -0.550, 9.936)` | 否/未释放 | 6945 optimal |
| v5，`1e6` 末段权重 | 4.6e-10 mm | 0.1578 mm | 94.49 mm | `(-12.110, -6.519, 93.439)` | 否/未释放 | 7036 optimal + 1 infeasible |
| v6，`3e4` 末段权重 | 3.5e-10 mm | 0.0699 mm | 16.04 mm | `(3.632, -0.415, 13.442)` | 否/未释放 | 7353 optimal |

v6 末段 1 秒窗口的 Cube 跟踪误差均值为 13.03 mm，实际相对 TCP 平移误差均值为 0.0143 mm；内部力候选均值为 1.283 N，目标力矩残差均值为 0.563 N·m。该内部力仍是模型候选估计，不是独立 D6 测量。

## 证据

- 原始冲突诊断：[baseline_diagnosis.json](../../offline/task28_g3_stage_a/evidence/reference_placement/baseline_diagnosis.json)。
- v4 对照：[metrics](../../offline/task28_task27_integration/evidence/task28_reference_placement_v4/reference_placement_metrics.json)、[原始 trace](../../offline/task28_task27_integration/evidence/task28_reference_placement_v4/task27_torque_bridge.jsonl.xz)。
- v5 发散失败：[metrics](../../offline/task28_task27_integration/evidence/task28_reference_placement_v5/reference_placement_metrics.json)、[summary](../../offline/task28_task27_integration/evidence/task28_reference_placement_v5/summary.json)、[executor log](../../offline/task28_task27_integration/evidence/task28_reference_placement_v5/task27_executor.log)。
- v6 稳定但拒绝释放：[summary](../../offline/task28_task27_integration/evidence/task28_reference_placement_v6/summary.json)、[reference consistency](../../offline/task28_task27_integration/evidence/task28_reference_placement_v6/reference_consistency.json)、[metrics](../../offline/task28_task27_integration/evidence/task28_reference_placement_v6/reference_placement_metrics.json)、[对比图](../../offline/task28_task27_integration/evidence/task28_reference_placement_v6/before_after_reference_placement.png)、[时序 CSV](../../offline/task28_task27_integration/evidence/task28_reference_placement_v6/reference_placement_timeseries.csv)、[GUI 场景截图](../../offline/task28_task27_integration/evidence/task28_reference_placement_v6/scene_loaded.png)、[原始 trace](../../offline/task28_task27_integration/evidence/task28_reference_placement_v6/task27_torque_bridge.jsonl.xz)。

## 复现与检查

```bash
DISPLAY=:1 XDG_SESSION_TYPE=x11 OMNI_KIT_ACCEPT_EULA=YES ROS_DOMAIN_ID=93 \
/home/ubuntu2004/isaacsim-4.5.0/python.sh isaac/scripts/task28_task27_one_pass.py \
  --mode integrated --live-qp --max-batches 1 --stage-a-only \
  --consistent-object-reference --assume-internal-contact-load \
  --internal-force-target-n 1 --record-joint-reactions \
  --internal-force-weight 1000 --internal-force-kp 0.5 \
  --local-sweep-limit-m 2.0 --execution-time-scale 16 \
  --timeout-s 1200 --run-id task28_reference_placement_v6

PYTHONPATH=offline/task28_g1_1:. \
python3 offline/task28_g3_stage_a/analyze_reference_placement.py \
  offline/task28_task27_integration/evidence/task28_reference_placement_v6 \
  --before offline/task28_task27_integration/evidence/task28_reference_placement_v4

PYTHONPATH=offline/task28_g1_1:. \
python3 offline/task28_g3_stage_a/diagnose_reference.py \
  offline/task28_task27_integration/evidence/task28_reference_placement_v6 \
  --output offline/task28_task27_integration/evidence/task28_reference_placement_v6/reference_consistency.json \
  --assert-consistent
```

针对本轮新增模块和分析器的回归为 **44 passed**；`py_compile` 通过。任务在 Stage A 停止，精密放置门禁仍是下一步需要解决的唯一未通过项。
