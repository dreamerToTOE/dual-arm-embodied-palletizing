# Stage B G0 contact-switch research evidence

本目录保存 G0 接触可观测性物理证据和最小纯函数测试。研究分支为 `research/stage-b-contact-switch-g0`，基于 `side-suction-palletizing@91bcf85cd32df91298a6ec428ada1d7563174260`；没有修改主分支或既有 selector 研究分支。

## 复现

在仓库根目录、已安装 Isaac Sim 4.5 的环境中运行单个 bounded probe（每次只启动一个实例）：

```bash
OMNI_KIT_ACCEPT_EULA=YES timeout 240s bash isaac/scripts/task28_launch_bundled_humble.sh \
  research/stage_b_contact_switch_g0/scripts/g0_c1_table.py \
  --run-id g0_c1_table_repro \
  --output-root research/stage_b_contact_switch_g0/evidence
```

C2/C3/C4 只需将脚本替换为 `g0_c2_wall.py`、`g0_c3_tool.py`、`g0_c4_full_scene.py`；每个 run-id 必须唯一。已有确切运行命令、状态、样本数和非零退出记录见 [`run_manifest.json`](run_manifest.json)。

纯函数/既有接触分析回归：

```bash
python3 -m pytest -q \
  research/stage_b_contact_switch_g0/tests/test_observer_math.py \
  offline/task28_stage_b_contact/test_c2_control.py \
  offline/task28_stage_b_contact/test_force_probe.py
```

生成诊断图：

```bash
python3 research/stage_b_contact_switch_g0/scripts/plot_g0.py
```

C1 额外分析可运行：

```bash
python3 offline/task28_stage_b_contact/analyze_d1.py \
  research/stage_b_contact_switch_g0/evidence/g0_c1_table_04
python3 offline/task28_stage_b_contact/analyze_force_probe.py \
  research/stage_b_contact_switch_g0/evidence/g0_c3_tool_01
```

## 证据布局

- `G0_CONTACT_OBSERVABILITY_REPORT.md`：中文结果、通道状态、失败与限制。
- `G0_G4_RESEARCH_ROADMAP.md`：G0–G4 规划，区分已完成和未执行内容。
- `protocol_manifest.json`：质量门槛和过程偏差说明。
- `observability_audit.json`：信号、来源、同步、验证状态和用途。
- `evidence/<run-id>/`：原始 JSONL、summary、分析文本；同级 `.process.log` 保存 Isaac Sim 日志。
- `figures/`：由原始 JSONL 可重复生成的三张诊断图。

本研究只做 Isaac Sim 物理观测验证。没有启动 G1–G4、RL、真实硬件或物理恢复实验；隔离刚性工具 proxy 结果不等于吸盘状态验证。
