# R2-B 相机精度校准实现

日期：2026-11-06  
分支：`research/stage-b-r2b-camera-precision`  
实现提交：`915ddd4`

## 变更范围

实现位于 [`r2b_camera_precision_calibration.py`](./scripts/r2b_camera_precision_calibration.py)，标记码位于 [`precision_marker_grids.npz`](./config/precision_marker_grids.npz)，单元检查位于 [`test_aruco.py`](./tests/test_aruco.py)。固定使用 `/World/R2B_RearWallCamera`，眼点 `[1.70,-0.35,0.80]`，目标点 `[0.82,0,0.30]`；没有修改机器人、FCL、力矩、吸附或模式切换约束。

## 标定板

- marker IDs：10、11、12、13；每个为合法 `DICT_4X4_50` 6×6 raster。
- marker 边长：0.060 m；中心偏移为 `y=±0.10 m`、`z=±0.10 m`，板面覆盖约 0.20 m。
- 可见 code cell 沿 `+X` 抬高；评估角点使用 `MARKER_FACE_X_M=0.008 m`。
- 每个 marker 有白色 backing、显式 camera-facing quiet mesh 和四条白色 quiet strips，避免黑边与黑视口相连。

## 计算流程

1. Isaac Sim 在 12 个固定板姿态上捕获 RGB，8 个训练视图、4 个 holdout 视图。
2. OpenCV ArUco 检测所有 ID，并按已知板中心投影关联检测结果；每视图保留 2–4 个 marker。
3. 使用所有训练角点拟合 pinhole `K`，固定主点 `(640,360)`、零切向畸变并固定五个径向系数为零；不使用 Cube 真值修正每帧位姿。
4. 对每个视图用 `solvePnP` 求相机位姿，报告相机中心误差、旋转误差和角点重投影误差。

捕获需通过 Isaac Sim 4.5 的 `/home/ubuntu2004/isaacsim-4.5.0/python.sh` 运行；已有捕获可在普通 Python 中使用 `--analyze-only` 重算。最终证据在 [`evidence/r2b_camera_precision_cal24`](./evidence/r2b_camera_precision_cal24)。

## 可复现命令

```bash
DISPLAY=:1 XDG_SESSION_TYPE=x11 OMNI_KIT_ACCEPT_EULA=YES \
  /home/ubuntu2004/isaacsim-4.5.0/python.sh \
  research/stage_b_vision_contact_r2b/scripts/r2b_camera_precision_calibration.py \
  --run-id r2b_camera_precision_cal24

python3 research/stage_b_vision_contact_r2b/scripts/r2b_camera_precision_calibration.py \
  --analyze-only research/stage_b_vision_contact_r2b/evidence/r2b_camera_precision_cal24
```

分析输出写入 `calibration_result.json`，其 `policy` 明确记录了“rendered multi-marker target; no Cube ground truth or per-frame correction”。
