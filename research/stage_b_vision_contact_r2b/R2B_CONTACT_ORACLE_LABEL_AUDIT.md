# R2-B PhysX 接触 oracle 标签审计

本报告把 PhysX callback 作为独立离线 oracle，不进入在线 `estimator_payload`。
原始 contact trace 有 68,018 行，覆盖 physics step 1–15,478；最后一步没有
callback，因此在视觉投影中保持 UNKNOWN。每行保留 collider identity、接触点、
法向、impulse 和 separation。

标签按边输出 `SUPPORTED_ABSENT`、`POSSIBLE_PAIR` 或 `LOADED_SUPPORTED`：后者要求
非零 impulse（>1e-8 N·s）或超过 0.1 mm penetration，并有至少两步 pair persistence。
零 impulse、正 separation 的 callback 只证明几何近邻，不能称 loaded contact。

在 1,550 个相机 step 投影中：

| 边 | pair present | POSSIBLE_PAIR | LOADED_SUPPORTED |
|---|---:|---:|---:|
| LEFT_TOOL_CUBE | 433 | 379 | 54 |
| RIGHT_TOOL_CUBE | 735 | 728 | 7 |
| CUBE_TABLE | 1,364 | 127 | 1,237 |
| CUBE_DEEP_WALL | 316 | 278 | 38 |

`CUBE_SIDE_WALL` 没有受支持的 collider pair，标记为
`UNAVAILABLE_NO_REPORTED_PAIR`。这些标签只用于独立评分；没有 fabricated loaded、
jam 或 attachment 标签。side compaction 的多数 wall/tool pair 只有 presence，
不能据此宣称加载成功。
