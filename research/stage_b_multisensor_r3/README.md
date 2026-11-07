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
