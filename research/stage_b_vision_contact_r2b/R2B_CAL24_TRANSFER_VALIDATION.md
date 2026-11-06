# R2-B cal24 到 Stage B 的转移验证


日期：2026-11-06  
研究分支：`research/stage-b-r2b-contact-recognition`  
校准父分支：`research/stage-b-r2b-camera-precision`  
校准实现提交：`915ddd44e4067c8687c0542e297b24c03485fa42`  
校准报告提交：`f374d75fb2ee7d6c008c89c1123541cbda860edc`  

## 结论

**Gate 0：`BLOCKED`。** 校准分支已经推送，且新分支从校准报告提交直接创建；但没有形成可接受的 Stage B cal24 转移证据，所以不能把 cal24 静态 holdout 数字当成动态 Cube 测量能力。

## 已核对的链路

- `git ls-remote origin refs/heads/research/stage-b-r2b-camera-precision` 返回 `f374d75fb2ee7d6c008c89c1123541cbda860edc`，该提交包含 `915ddd4` 的实现和研究报告。
- cal24 报告的静态 holdout 是 4 个视图：位置误差中位数 5.01 mm、P95 10.86 mm，旋转误差中位数 0.35°，重投影误差中位数 1.08 px。它仍是 rendered board 的 holdout，不是动态 Cube 或物理接触验证。
- 当前运行时 `isaac/scripts/task28_task27_one_pass.py` 使用 `/World/R2B_RearWallCamera`，但仍挂载单个 marker ID23；协议文件仍记录旧的 `/World/R2B_Camera` 和 ID23。运行时没有使用 cal24 四 marker board、cal24 K 和对应的 Cube 刚体变换。
- 对原 cal24 生成代码做了几何审计：marker group 已平移 `x=0.008 m`，cell 又沿 `+X` 增加 `0.008 m`，而 manifest 角点仍按单个 `0.008 m` 写入。该双偏移疑点足以阻止把原静态结果当作已转移。

## 一次有界 Stage B 探针

为满足“旧数据配置不兼容时只做一次短实验”的约束，运行了 1 次静态、无操作的 Cube_01 +X 刚性几何附着探针：

- run id：`r2b_cal24_transfer_probe01`
- 相机：`/World/R2B_RearWallCamera`，12 张 RGB 视图
- 标记：ID 10/11/12/13，0.060 m，四标记板作为 Cube 子层级几何
- 使用：cal24 K 文件；没有 Task27 操作、没有接触确认、没有控制器写入
- 结果：四个 ID 的检测计数均为 0/12，usable pose views 为 0，位置/姿态误差为空；12 张图可读但灰度均值中位数仅 0.397/255、最大 0.399/255。保存的视图是近黑空场景，因而这次探针是**探针执行失败**，不是转移通过。

证据：`evidence/r2b_cal24_transfer_probe01/transfer_result.json`。该板是静态渲染用刚性几何，不应被称为已验证的物理安装接触件。

## 决策

本窗口不再重复完整 Task27，也不重新研究双臂运输遮挡。要解除阻断，下一次实验必须先修正相机/视口采集与 marker 几何链路，再用同一 rear_11、同一 K、同一 Cube 附着板完成独立动态 holdout；在此之前，视觉只可支持粗进度和接近度候选。
