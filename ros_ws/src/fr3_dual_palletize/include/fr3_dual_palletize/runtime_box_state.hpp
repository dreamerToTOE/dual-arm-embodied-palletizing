#pragma once

#include <string>

#include <geometry_msgs/msg/pose.hpp>

#include "fr3_dual_palletize/msg/box_state.hpp"
#include "fr3_dual_palletize/palletizing_job.hpp"

namespace fr3_dual_palletize
{

// 将 Task18 的运行时输入还原为 Task17 的通用 BoxSpec。这里不接受任何固定
// 初始坐标：initial_pose 始终来自本次 episode 的 BoxState.pose。
bool boxStateToBoxSpec(
  const msg::BoxState& state,
  BoxSpec& box,
  std::string& error);

// 组合物体世界位姿和局部抓取候选，得到 MoveIt 可直接使用的世界 pick pose。
// 这允许随机 yaw，不假设顶部吸盘一定沿 world 轴对齐。
geometry_msgs::msg::Pose composePose(
  const geometry_msgs::msg::Pose& parent,
  const geometry_msgs::msg::Pose& local);

}  // namespace fr3_dual_palletize
