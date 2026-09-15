# Task22-A：同一个 Ground Truth scene snapshot 生成的可选作业集合。
# scene_version 在候选几何/物体状态改变时递增，禁止执行端消费旧版本。

std_msgs/Header header
uint64 scene_version
uint64 source_seed
TaskDispatchCandidate[] candidates
