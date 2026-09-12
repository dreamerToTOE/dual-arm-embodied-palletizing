# Task18：同一随机 episode 的完整运行时物体输入。
# seed 使失败布局可复现；header.stamp 是该批 Ground Truth 的统一采样时刻。

std_msgs/Header header
uint64 seed
BoxState[] boxes
