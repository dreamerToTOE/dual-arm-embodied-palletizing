# Task22-A：Python selector 的高层分派决定。
# 本消息不是关节/吸盘命令；MoveIt Dispatch Gateway 必须检查 scene_version、实时
# Ground Truth、IK 与 Task08/09 FCL 后，才可将它转化为物理执行。

std_msgs/Header header
uint64 scene_version
string candidate_id
string object_id
geometry_msgs/Pose target_pose
string support_surface_id
string coordination_mode
string preferred_arm
string fallback_arm
float64 selector_score
string selector_id
