# Task18：单个物体的运行时状态。
#
# 该消息是 Isaac Ground Truth 与未来视觉感知的统一契约。pose 是世界坐标下
# 的当前物体中心位姿；其余字段来自已知的 BoxSpec，供 Task19+ 生成碰撞体、
# 选择协调模式和计算抓取位姿。

builtin_interfaces/Time stamp
string id
geometry_msgs/Pose pose
float64[3] size
float64 mass
string payload_class
string[] allowed_modes
string[] grasp_candidate_ids
geometry_msgs/Pose[] grasp_candidates
