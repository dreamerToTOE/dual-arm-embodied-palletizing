# Task22-A：已通过 Task19 几何放置门禁的只读分派候选。
# 不包含机器人轨迹；Python selector 只能对候选排序，最终 MoveIt/FCL 验证仍属于
# 后续 Dispatch Gateway。

string candidate_id
string object_id
geometry_msgs/Pose object_pose
geometry_msgs/Pose pick_pose
geometry_msgs/Pose target_pose
string support_surface_id
float64 support_height
float64 placement_cost
uint32 placement_rank
float64[3] size
float64 mass
string payload_class
string[] allowed_modes
string[] grasp_candidate_ids
geometry_msgs/Pose[] grasp_candidates
