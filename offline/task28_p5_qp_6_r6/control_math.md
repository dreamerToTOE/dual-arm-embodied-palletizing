# R6 诊断计算范围

每个 arm 保留原场景的九维状态 `q=[q_arm7,q_finger2]`。诊断 effort 为

```text
tau_arm = g_arm(q) + perturbation
```

其中 `g` 是实际当前姿态下 Isaac 返回的广义重力向量，扰动只作用于注册的
joint index 3，幅值为 ±0.25 N·m。这个 holding vector 是输入设计的重力
基准，不是校准 torque measurement。native finger drive 仍在运行，完整九维
写入的后两项沿用 command buffer 中的当前值。

独立 evaluation 还计算了

```text
r_est = M(q) qdd_est + c(q,dq) + g(q) - tau_command
qdd_est = (dq_end[k] - dq_end[k-1]) / dt
```

`dq_end` 是 TGS 末步 solver velocity，所以 `qdd_est` 只是诊断估计。它没有
被当作 torque sensor，也没有把未知 L-tool inertia、接触反力或手指耦合项
删除。R6 的模型结果因此保持 `PARTIAL_MEASURED_RESPONSE_MODEL_RESIDUAL_NOT_CERTIFIED`；
它不足以启动正式 gravity+PD static hold、computed torque 或 cooperative QP。

