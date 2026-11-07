# R3 P0/P1 复现实验

范围：只做数据/因果审计和桌面约束视觉 A/B。没有控制器写入、FK fusion、P2/RL。
Python 3.10、OpenCV 4.5.4（含 aruco）、NumPy 1.24.4、SciPy 1.8.0 已安装；无需新依赖。
实际版本以 `python3 -c 'import cv2,numpy,scipy; print(cv2.__version__,numpy.__version__,scipy.__version__)'` 为准。

从仓库根目录运行：

```bash
PYTHONPATH=. python3 -m pytest -q research/stage_b_multisensor_r3/tests research/stage_b_vision_contact_r2b/tests
PYTHONPATH=. python3 research/stage_b_multisensor_r3/scripts/evaluate_planar_pose.py \
  --dataset research/stage_b_multisensor_r3/evidence/physical06_holdout \
  --dataset research/stage_b_multisensor_r3/evidence/static_pose_dataset \
  --out /tmp/r3_p1_holdout_replay.json
```

该重放只用 committed corners/K/T/config 和独立 oracle，不需要大 RGB-D 文件。
预测先保存，然后 oracle 才被打开；两组相同帧、同 K/T 和 marker 模型。不要把 R2
`runtime_adapter` 的 GT velocity 或 `visual_detections.phase` 填进 R3 输入。

## 数据重建

`r3_p0_sensor_manifest.json` 记录本地 raw_root 与输入哈希。若大文件可用，重新提取：

```bash
PYTHONPATH=. python3 research/stage_b_multisensor_r3/scripts/build_pose_dataset.py \
  --name physical06_holdout \
  --compact research/stage_b_vision_contact_r2b/evidence/r2b_rear_wall_physical06 \
  --raw RAW_ROOT_FROM_P0_MANIFEST --stride 3 --out /tmp/r3_physical06_rebuild
```

使用新输出目录；脚本拒绝覆盖已有数据。physical05/07 可用同入口，stride=1；已有
cached corners 会直接重用，仅取像素字段。`audit_projection.py` 从原始 RGB 抽查旧
corners，结果为 10/10 一致。原始文件路径存在于 observations 中只作 lineage；不在
推断中读取 Cube GT 或重新选择 marker 位置。

P0 可重建入口：`scripts/audit_r3_p0.py --run NAME COMPACT_ROOT RAW_ROOT ... --out PATH`。
它记录 runtime/contact/corners/timeline 哈希、availability、时序和抽样 RGB/depth 检查。
`analyze_timing.py --dataset DIR ... --out PATH` 只在评分侧分析 render-wait 位移。

## 唯一新增 Isaac 静态运行

```bash
DISPLAY=:1 OMNI_KIT_ACCEPT_EULA=YES /home/ubuntu2004/isaacsim-4.5.0/python.sh \
  research/stage_b_multisensor_r3/scripts/capture_static_holdout.py --run-id NEW_RUN_ID
```

它复用 Task26 原场景并选 Task27 布局、legacy marker 和 rear_11。16 个预定义姿态、
0 动态 episode。内参与相机读回、image hash、actual visible-face corners、原场景
源码 hash、冻结配置 hash、GT transform 都在 manifest。新的 holdout 应在冻结模型
后运行，不能重用其 GT 调参再称独立测试。

主结果：`evidence/p1_holdout_results.json`；开发数据：`p1_development_results.json`；
代表失败图和 oracle 记录都保留。SDK K 修复前的失败在 `diagnostic_authored_k/`。
原始 R2 文件、控制器、Task26/27 判据没有修改。

## P2A：Stage-B 时间与视觉缺失

结果入口：`R3_P2A_FINAL_DECISION.md`。P2A停止于PARTIAL，不自动开始P2B。
P1模型/config未改；本轮不做离桌/侧翻/Stage-A研究。主验证是dynamic07；05/06仅诊断。

CPU检查与完整本地重放（从仓库根运行）：

```bash
python3 -m pytest -q research/stage_b_multisensor_r3/tests \
  research/stage_b_vision_contact_r2b/tests \
  offline/task28_g1_1/tests/test_independent_fk.py
python3 research/stage_b_multisensor_r3/scripts/evaluate_p2a_alignment.py \
  --run research/stage_b_multisensor_r3/evidence/r3_p2a_dynamic07 \
  --out /tmp/r3_p2a_replay
```

主验证07的30组RGB/压缩depth、history和oracle均提交，重放不依赖physical07大文件。
输出含逐帧CSV、误差/时间诊断和无oracle的P2B接口；所有测量时间仍UNCERTAIN，
`measurement_time_s=null`，禁止当作已批准的在线融合输入。

历史107帧审计（原始大文件需本地可用，路径在P0 manifest）：

```bash
python3 research/stage_b_multisensor_r3/scripts/audit_p2a_occlusion.py \
  --raw PHYSICAL07_RAW_ROOT --out /tmp/r3_p2a_occlusion_replay
```

`config/p2a_visual_review.json`是逐格图像审查标签，不是从detection failure推断遮挡
的自动规则。图像hash、全部107帧联系表和代表原图已提交；19帧UNKNOWN保留。

最终有界Isaac复采：

```bash
DISPLAY=:1 OMNI_KIT_ACCEPT_EULA=YES /home/ubuntu2004/isaacsim-4.5.0/python.sh \
  research/stage_b_multisensor_r3/scripts/capture_p2a_dynamic.py --run-id NEW_P2A_RUN
python3 -c "import json; from pathlib import Path; p=Path('research/stage_b_multisensor_r3/evidence/NEW_P2A_RUN/manifest.json'); m=json.loads(p.read_text()); assert m['frames']==30 and not m['errors']"
```

不要覆盖已有run目录。`--blocking-render`是另一采集模式，不是已通过的修复。
不同模式/阶段的条件由`P2A_EXPERIMENT_MANIFEST.json`登记。旧01/04几何失败和
05/06诊断raw仍在本地，Git保留日志、source snapshot、history/observations/oracle、
联系表和代表图；其原始RGB/depth未全量提交。复现旧失败源码时，先复制对应snapshot
到本目录的`scripts/`层级，再使用新run-id。02/03的启动失败同样保留。

Isaac关闭可能掩盖Python异常，所以退出码0不能替代完成manifest；检查failure.json、
callback errors及图像几何。scene只施加受控kinematic桌面刺激与小幅腕部目标，
不是Task27任务成功或真实推入实验。host arrival/writer时间不能当render时间。

## P2A-R1：独立图像时钟

结论 **PARTIAL**，报告 `R3_P2A_R1_TIME_BINDING.md`；入口
`R1_EXPERIMENT_MANIFEST.json`。SDK/Fabric 映射一致，但图像 epoch 时差在 0/33.333 ms
间变化；先冻结、后独立留出的区间覆盖 136/136 帧，宽 50 ms、最坏半宽 25 ms。
没有授权在线融合，旧 P2A 的 30 个 UNCERTAIN 时间戳保持原样。

只用 Git 中的原始时钟 ROI 重放，不启动 Isaac：

```bash
python3 research/stage_b_multisensor_r3/scripts/evaluate_r1_time_binding.py \
  --run research/stage_b_multisensor_r3/evidence/r1_calibration04 \
  --binding research/stage_b_multisensor_r3/config/r1_time_binding_frozen.json \
  --out /tmp/r1_calibration_replay
python3 research/stage_b_multisensor_r3/scripts/evaluate_r1_time_binding.py \
  --run research/stage_b_multisensor_r3/evidence/r1_holdout01 \
  --binding research/stage_b_multisensor_r3/config/r1_time_binding_frozen.json \
  --out /tmp/r1_holdout_replay
```

如需新的有界 Isaac 重采，先绑定安装包实际 C++ RationalTime。这里系统已有 g++、
`/usr/include/pybind11` 和 Python 3.10 headers，不新增 Python 包；换 SDK/ABI 必须
重新核查，不拷贝本机二进制到别处。

```bash
python3 research/stage_b_multisensor_r3/scripts/build_r1_fabric.py \
  --isaac-root /home/ubuntu2004/isaacsim-4.5.0
DISPLAY=:1 OMNI_KIT_ACCEPT_EULA=YES /home/ubuntu2004/isaacsim-4.5.0/python.sh \
  research/stage_b_multisensor_r3/scripts/capture_r1_time_binding.py \
  --run-id NEW_R1_CALIBRATION --split calibration
python3 research/stage_b_multisensor_r3/scripts/evaluate_r1_time_binding.py \
  --run research/stage_b_multisensor_r3/evidence/NEW_R1_CALIBRATION \
  --out /tmp/new_r1_calibration_analysis --freeze /tmp/new_r1_binding.json
DISPLAY=:1 OMNI_KIT_ACCEPT_EULA=YES /home/ubuntu2004/isaacsim-4.5.0/python.sh \
  research/stage_b_multisensor_r3/scripts/capture_r1_time_binding.py \
  --run-id NEW_R1_HOLDOUT --split holdout --binding /tmp/new_r1_binding.json
python3 research/stage_b_multisensor_r3/scripts/evaluate_r1_time_binding.py \
  --run research/stage_b_multisensor_r3/evidence/NEW_R1_HOLDOUT \
  --out /tmp/new_r1_holdout_analysis --binding /tmp/new_r1_binding.json
```

run、out、freeze 路径必须不存在；不得覆盖已提交的冻结文件。脚本最多 180 次 dynamic
updates，另有固定 warmup/pattern control；检查 completion manifest、errors 和图像码。
`--updates N` 只供启动诊断，短于完整 phase coverage 的运行不能替代完整结论。
原始时钟用 scene cells 在 RGB 和 depth 中独立编码，不是 writer 叠字；默认 near=1 m
导致早期码不可见，最终显式设 near=.01 m，未移动相机。

API/裁剪失败和 calibration03 无效完整数据都保留，不能拿其没有有效时间码的图像
拟合时间偏移。每个有效 run 保存 capture/config/helper snapshot；精确复现历史
source hash 时，把对应 snapshot 放回原模块路径的隔离 checkout 后运行新的 run-id。
producer `code_bits()` 未变，当前接口只增加区间输出与拒绝不确定帧；
`measurement_time_s=null`、`online_fusion_eligible=false`。

## P2A-R2：同帧 Cube 场景状态绑定

报告：`R3_P2A_R2_SCENE_TIME_BINDING.md`，入口：`R2_EXPERIMENT_MANIFEST.json`。
判定 PARTIAL：独立留出 180 个唯一帧，176 个候选集合输出；42 个可独立定时的运动帧
支持一 tick 最坏界，138 个仍不可独立定时。没有导出单点或中点时间。
V0/V1/V2 用相同图像和相同 RGB-D Cube 估计；V1 因 fingerprint 不同仅为冻结数值参考。

已提交 raw ROI 同时包含原渲染时间码和 Cube。离线重放：

```bash
python3 research/stage_b_multisensor_r3/scripts/evaluate_r2_scene_binding.py \
  --run research/stage_b_multisensor_r3/evidence/r2_calibration02 \
  --binding research/stage_b_multisensor_r3/config/r2_scene_binding_frozen.json \
  --out /tmp/r2_calibration_replay
python3 research/stage_b_multisensor_r3/scripts/evaluate_r2_scene_binding.py \
  --run research/stage_b_multisensor_r3/evidence/r2_holdout01 \
  --binding research/stage_b_multisensor_r3/config/r2_scene_binding_frozen.json \
  --out /tmp/r2_holdout_replay
```

新的独立重采流程（本轮已用完三次预算；这里只记录复现命令，不自动续跑）：

```bash
python3 research/stage_b_multisensor_r3/scripts/build_r1_fabric.py \
  --isaac-root /home/ubuntu2004/isaacsim-4.5.0
DISPLAY=:1 OMNI_KIT_ACCEPT_EULA=YES /home/ubuntu2004/isaacsim-4.5.0/python.sh \
  research/stage_b_multisensor_r3/scripts/capture_r2_scene_binding.py \
  --run-id NEW_R2_CALIBRATION --split calibration
python3 research/stage_b_multisensor_r3/scripts/evaluate_r2_scene_binding.py \
  --run research/stage_b_multisensor_r3/evidence/NEW_R2_CALIBRATION \
  --out /tmp/new_r2_calibration_analysis --freeze /tmp/new_r2_scene_binding.json
DISPLAY=:1 OMNI_KIT_ACCEPT_EULA=YES /home/ubuntu2004/isaacsim-4.5.0/python.sh \
  research/stage_b_multisensor_r3/scripts/capture_r2_scene_binding.py \
  --run-id NEW_R2_HOLDOUT --split holdout --binding /tmp/new_r2_scene_binding.json
python3 research/stage_b_multisensor_r3/scripts/evaluate_r2_scene_binding.py \
  --run research/stage_b_multisensor_r3/evidence/NEW_R2_HOLDOUT \
  --out /tmp/new_r2_holdout_analysis --binding /tmp/new_r2_scene_binding.json
```

不覆盖已有 run/out/frozen 文件。当前 capture 的 probes、较大 ROI writer 改变了观测
条件，不能把其零延迟回填 R1。Capture 只写受控桌面 Cube；没有机器人控制器命令。
机器接口在 `time_binding_interface.jsonl`，完整历史候选含 physics_step/time_s，未确认时
唯一时间/步为空，旧 SDK 0/−2 tick 假设另列。它依赖时间码和受控刺激，不是通用
在线时间修复。更换 pixel helper/config 后须重新校准，不可改源码再套用旧 binding。

P2B 仅具备时间候选/视觉离线回放基础；FK/vision 融合仍需配对 q/dq/FK。原任务
runtime shadow 和 online fusion 均未准入。R2 到此停止。

R2 正式候选保留完整因果历史中的重复 pose 别名，最大跨度 7150 ms；
`clock_conditioned_candidates` 的相邻两步只是局部先验。局部窗口产生的 74 帧结论
保留为诊断，不能代替最终全历史 42 帧证据。

R2 图表可用 `python3 research/stage_b_multisensor_r3/scripts/plot_r2_results.py`
从已提交结果重建；该命令不启动 Isaac。
