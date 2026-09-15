# Task23：二指夹爪“暂存后平推”原子作业。
#
# Python selector 明确给出 supply、staging、target 和双臂角色；本消息不是关节命令。
# C++ 执行器仍必须用实时 Ground Truth、MoveIt/FCL 与放置误差门禁后才可执行。

std_msgs/Header header
uint64 scene_version
uint32 sequence_index
string task_id
string object_id
geometry_msgs/Pose source_pose
geometry_msgs/Pose staging_pose
geometry_msgs/Pose target_pose
string supply_arm
string push_arm
float64 push_distance
