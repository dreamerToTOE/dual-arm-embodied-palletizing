# Task22-D：一件物体完成真实物理执行后的唯一 World Commit。
#
# 只有执行器在以下条件均通过后才发布：
# 1. MoveIt/OMPL 与 FCL 完成最终门禁；
# 2. Isaac 吸盘完成释放；
# 3. Isaac Ground Truth 落稳位姿满足放置误差阈值；
# 4. 该真实位姿已写回执行 MoveIt World。
#
# Candidate provider 收到本消息后，才将该物体从待搬运集合移到已放置支撑集合，
# 并生成下一件物体的 scene_version。它不是控制命令。

std_msgs/Header header
uint64 source_seed
uint64 completed_scene_version
string object_id
geometry_msgs/Pose settled_pose
string coordination_mode
string executed_arm
