# Task28 G2-P5-QP-5

该目录是隔离的双 FR3 显式 effort 参考跟踪实验。`collector.py` 只加载冻结 FR3 USD，在同一 Isaac physics scene 中创建左右两个独立 articulation；没有 Cube、吸盘、D6、桌面、ROS 或生产 bridge。`preregistration.json` 在正式物理运行前冻结。

运行前置：

```bash
export OMNI_KIT_ACCEPT_EULA=YES PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONPATH=.:offline
PR=$(sha256sum offline/task28_p5_qp_5/preregistration.json | cut -d' ' -f1)
/home/ubuntu2004/isaacsim-4.5.0/python.sh offline/task28_p5_qp_5/collector.py \
  --asset /absolute/path/to/fr3.usd --output /fresh/output --prereg-sha256 "$PR"
```

第一阶段 240 个 physics step 静态门禁通过后才进入五段、0.1 s 的参考游标和最终保持。失败运行的原始 JSON 保存在 `evidence/failed_runs/`；正式运行保存在 `evidence/formal_run_v1/`。结果仅用于隔离虚拟几何研究。
