# Stage B G0-R / G1-A research evidence

本目录基于 G0 提交 `513cd7875fca52ac9e7a200ff67bf3ba2fcf3b91` 建立，分支为 `research/stage-b-contact-switch-g1a`。主分支 `side-suction-palletizing`、selector 分支和历史 G0 evidence 未修改。

## 运行类型

- **G0-R**：Isaac Sim 4.5 最小 Cube/Table/DeepWall 物理场景；修正 active external-force impulse ledger，并重复加载/撤回。
- **G0-R fuller diagnostic**：Isaac Sim 原始 Task27 fuller scene 预检；不进入 FR3 控制。
- **G1-A**：Isaac Sim 几何/运动状态触发的 Cube 物理 proxy；C0/C1/C2 都是实际 PhysX 力命令，不是离线命令生成。没有 FR3 关节执行。

## 复现命令

纯测试：

```bash
python3 -m pytest -q research/stage_b_contact_switch_g1a/tests
```

G0-R 单次：

```bash
OMNI_KIT_ACCEPT_EULA=YES timeout 240s bash isaac/scripts/task28_launch_bundled_humble.sh \
  research/stage_b_contact_switch_g1a/scripts/g0r_c2_wall.py \
  --run-id g0r_repro \
  --output-root research/stage_b_contact_switch_g1a/evidence \
  --config research/stage_b_contact_switch_g1a/configs/g0r_c2_protocol.json
```

G1-A 单次：

```bash
OMNI_KIT_ACCEPT_EULA=YES timeout 180s bash isaac/scripts/task28_launch_bundled_humble.sh \
  research/stage_b_contact_switch_g1a/scripts/g1a_geometry_switch.py \
  --run-id g1a_C2_repro --controller C2 \
  --output-root research/stage_b_contact_switch_g1a/evidence \
  --config research/stage_b_contact_switch_g1a/configs/g1a_proxy_protocol.json
```

聚合分析和图表：

```bash
python3 research/stage_b_contact_switch_g1a/scripts/analyze_g1a.py
python3 research/stage_b_contact_switch_g1a/scripts/analyze_full_scene.py
```

## 证据与结果

- [`G0_R_FORCE_OBSERVABILITY_REPORT.md`](G0_R_FORCE_OBSERVABILITY_REPORT.md)：G0-R ledger、三次卸载重复、fuller-scene UNKNOWN 诊断。
- [`G1A_GEOMETRY_CONTROL_SWITCH_REPORT.md`](G1A_GEOMETRY_CONTROL_SWITCH_REPORT.md)：C0/C1/C2 三次重复和 proxy 限制。
- `evidence/`：每个 run 的 JSONL、summary、原始 Isaac 日志和失败记录。
- `g0r_summary.json`、`g0r_full_scene_diagnostic.json`、`g1a_comparison.json`：机器可读聚合结果。
- `figures/g0r_wall_ledger.png`、`figures/g1a_proxy_comparison.png`：诊断图。

本任务在 `G1A_PROXY_PASS` 后停止。没有执行 G1-B 力反馈、G2、G3、G4、RL 或真实硬件。
