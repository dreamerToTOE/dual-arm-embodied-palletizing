# TASK28 D2 环境恢复与 D3 工具单因素隔离

## 结论

**D3 结果为受限的探索性证据，尚不能认定严格单因素因果。** 一次 C/D 任务完成了两组各 60 步采样，但任务最终状态为 `UNMATCHED`：四个 Cube USD 状态字段不相同。此外，名义高摩擦材质目标没有出现在场景控制记录中；USD 图 prim 虽被停用，日志仍显示已缓存的 ROS 图执行。这些问题本应在物理运行前识别，本次预检没有覆盖完整，故不能声称所有实验控制预检均通过。未进行第二次物理实验。

保留 **D2_PARTIAL、B_C2R_BLOCKED、B0_PARTIAL**；没有放宽既有门槛，没有 FR3 轨迹、力渐变、推物、完整 B0 或 RL。

## ROS 2 启动环境

4.5 的 `python.sh` 调用 `setup_python_env.sh`，后者添加 Kit 的 Python 和动态库目录，但不自动添加 Humble 库；Bridge 依赖 OmniGraph、Isaac Core、PhysX 等扩展，并通过独立的 `isaacsim.ros2.bridge.check` 检查加载。此前 D2 重试进程没有找到 `librosidl_runtime_c.so`。NVIDIA 的 [Isaac Sim 4.5 配置示例](https://docs.nvidia.com/learning/physical-ai/getting-started-with-isaac-sim/latest/leveraging-ros-2-and-hil-in-isaac-sim/04-deploying-isaac-ros-on-jetson.html) 指定内置 Bridge Humble 库目录和 Fast DDS。

本次隔离启动器 `isaac/scripts/task28_launch_bundled_humble.sh` 在启动 Python **之前**设置：

```text
ROS_DISTRO=humble
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
LD_LIBRARY_PATH=/home/ubuntu2004/isaacsim-4.5.0/exts/isaacsim.ros2.bridge/humble/lib
```

它清除继承的系统 ROS/overlay 搜索路径、`PYTHONPATH` 和 `LD_PRELOAD`，然后调用原 `python.sh`；后者仍会追加 Kit 自己的路径。没有改系统环境或安装。Bridge 探针记录的 `/proc/self/maps` 中 `librcl*`、`librmw*`、`librosidl*` 采样均来自 Isaac 内置库，并成功启用 Bridge，在独立 ROS domain 167 的播放图中创建并初始化非零 `ROS2Context` 句柄。

验证分两层：早期探针只验证节点创建，未验证实际 context 计算；一个无执行触发的同步图输出了 0，故不能证明 context 已初始化，且默认快速关闭掩盖了异常退出。改为播放时触发后，实际句柄非零。**正常扩展逐一关闭模式在打印关闭标记后发生 Python 退出阶段 segmentation fault，进程退出 1**（见 `ros_bridge_context_live.log`）；不能宣称完整正常卸载已恢复。快速关闭探针 `ros_bridge_fast.log` 的非零句柄为 `777640296`，进程退出 0；快速关闭只表示进程快速退出状态，不证明逐扩展清理成功。后续运行器要求正常关闭探针退出 0，因此当前不具备再次物理运行的完整环境预检条件。

证据目录：[Bridge 日志与 C/D 原始数据](../../offline/task28_stage_b_contact/evidence/task28_d3_tool_isolation_v1/)，包括 `ros_bridge_provenance.log`、`ros_bridge_context.log`、`ros_bridge_context_live.log` 与对应 exit 文件。

## C/D 设置和实际控制

两组使用独立 Isaac 进程和全新 Task27 场景；各经过 90 次应用更新、52 个预热物理步，窗口开始前对 Cube 唤醒一次，不在观测中反复唤醒。每组仅创建一个 Cube/Table 接触视图。C 没有复制工具；D 有七个原侧吸碰撞形状的完整静止代理。两组沿用原 Cube、桌面、其他场景对象、诊断 Cube、重力、原 TGS/stabilization 和工具的 C2R 放置方式；没有移动工具、施力或测量期写 Cube 状态。诊断场景中两条 FR3 均停用。

两组质量均 `0.800000012 kg`，重力 Z `-9.81000042 m/s²`，物理步长 `0.0166666675 s`；均记录 24 清醒步及 36 睡眠步。时间步、预热、求解器、桌面几何、碰撞与报告配置和绑定关系字段匹配。动态控制读取的初始线速度均为零；初始位置相差 `1.610 µm`，四元数分量向量差 `5.14e-6`。观测开始前的 USD `physics:velocity`、`physics:angularVelocity`、`xformOp:orient`、`xformOp:translate` 四个字段不相等；未同步记录动态控制角速度，因此无法把初始角速度视作已匹配。物理前静态测试也未能检查该运行时差异。

Cube 和桌面的 `material:binding` 都指向 `/World/Task27/HighFrictionMaterial`，但控制快照没有该材质 prim。源码中 `_make_material()` 先建立此 prim，随后 `_build_objects()` 删除 `/World/Task27` 再重建任务根。名义静摩擦 0.90、动摩擦 0.75 由源码设定，但本次缺少对有效运行时材质和摩擦值的验证；不得将名义值直接当作实际值。运行后添加了对无效绑定的预检和离线回归：当前场景若仍无有效目标将拒绝未来运行；该检查并非本次物理运行之前执行。

为了关闭无关 ROS 执行，本次在诊断副本对 `/World/Task27ROSGraph` 调用 `SetActive(False)`；但 C/D 日志分别有 **810/802 条错误**，其中包含该图的关节发布节点继续访问停用的 FR3。这说明 USD prim 停用不能证明已经创建的 OmniGraph 停止执行；`ros_graph_deactivated` 字段仅记录 USD prim 操作，不代表执行停止。生产场景未修改。两组同样有此问题，但无法排除间接影响。

## 原始观测

| 清醒期指标 | C：无工具 | D：静止工具 |
| --- | ---: | ---: |
| 步数 / 时间 | 24 / 0.400 s | 24 / 0.400 s |
| Cube/Table 法向力首步 → 末步 | 7.838227 → 7.848005 N | 7.838451 → 1.185916 N |
| 平均法向力 | 7.847575 N | 2.271468 N |
| Z 动量残差累计 | 1.16e-7 N·s | 2.230588 N·s |
| 观测窗口清醒期最大速度 | 0.0002473 m/s | 0.0002417 m/s |
| 清醒期相对首帧最大位移 | 0.0725 µm | 1.536 µm |
| 桌面摩擦冲量最大模 | 0.0001037 N·s | 0.0001126 N·s |
| 事件与矩阵冲量最大差 | 1.30e-8 N·s | 9.31e-9 N·s |

D 的四个吸杯/Cube 候选对均在每个清醒步出现，记录的各点法向冲量为 0；吸杯间隙约 2.998–3.000 mm，工具位移为 0。逐步记录的 Cube 其他接触事件冲量合计也为零。吸杯对的切向摩擦没有独立测得，且报告接口不保证暴露全部内部求解器贡献，故不能据此断言不存在所有非桌面作用。休眠期不产生报告并不能说明实际支撑消失。事件报告和矩阵同属 PhysX 后端，不构成独立力校准。

[曲线](../../offline/task28_stage_b_contact/evidence/task28_d3_tool_isolation_v1/cd/comparison.png)、[比较及自动匹配结果](../../offline/task28_stage_b_contact/evidence/task28_d3_tool_isolation_v1/cd/comparison.json)、[运行状态](../../offline/task28_stage_b_contact/evidence/task28_d3_tool_isolation_v1/cd/manifest.json)、[C 逐步数据](../../offline/task28_stage_b_contact/evidence/task28_d3_tool_isolation_v1/cd/C/timeseries.jsonl)、[D 逐步数据](../../offline/task28_stage_b_contact/evidence/task28_d3_tool_isolation_v1/cd/D/timeseries.jsonl)及两组 `contact_events.jsonl` 保留了每步接触对、法向、事件冲量、摩擦、睡眠、速度和动量残差。

## 判断、下一步与测试

本次显示工具存在与支撑读数衰减**相关**，且数值形态与先前 C2R 接近；由于初态检查未通过、有效材质未确认、ROS 图仍执行，不能证明工具代理是唯一原因，也不能判定未报告的内部求解器项。下一步应先在诊断副本修正材质构造顺序或建立可验证的同值材质，核对实际生效的材质；在图构造时禁止或明确断开 ROS 执行并核实无相关错误；记录并匹配角速度、位置和物理步。随后才考虑一次新的有界 C/D 对照。不要改生产场景来隐藏问题。

物理任务前聚焦离线测试 **28 passed**；运行后增加材质绑定拒绝测试，聚焦回归 **29 passed**。本次唯一 C/D 作业在完成两组后按现有检查诚实标记 `UNMATCHED`，没有自动重试。
