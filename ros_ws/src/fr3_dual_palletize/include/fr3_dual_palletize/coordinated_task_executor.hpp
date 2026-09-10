#pragma once

#include <cstddef>
#include <string>

#include <rclcpp/logger.hpp>

#include "fr3_dual_palletize/palletize_primitive.hpp"
#include "fr3_dual_palletize/task_trajectory_candidate.hpp"

namespace fr3_dual_palletize
{

// Task09 的真实执行参数。轨迹仍按 Task08-B 已验收的 10 ms 时间分辨率发布；
// 吸盘状态确认超时只在离散事件点触发全局 HOLD，不会让任一机械臂脱离协调时间轴。
struct CoordinatedTaskExecutionConfig
{
  double command_period_sec{0.01};
  double grasp_timeout_sec{2.0};
  double release_timeout_sec{2.0};
  double final_hold_sec{0.50};
};

struct CoordinatedTaskExecutionResult
{
  bool valid{false};
  bool completed{false};
  std::size_t events_executed{0};
  double candidate_makespan_sec{0.0};
  double physical_pause_sec{0.0};
  double wall_duration_sec{0.0};
  std::string error;
};

// 只消费已经由 Task08-B/FCL 验证为 SAFE 的两个完整候选。
// 它使用一个共享时钟向左右 Isaac command topic 发布插值关节目标。到达
// SUCTION_ON / ATTACH / SUCTION_OFF / DETACH 时，两臂同步保持当前安全采样姿态；
// 所有物理抓放与 MoveIt Planning Scene 状态切换完成后再恢复同一时间轴。
class CoordinatedTaskExecutor
{
public:
  CoordinatedTaskExecutor(
    rclcpp::Logger logger,
    PalletizePrimitive& left,
    PalletizePrimitive& right);

  CoordinatedTaskExecutionResult execute(
    const TaskTrajectoryCandidate& left_candidate,
    const TaskTrajectoryCandidate& right_candidate,
    const CoordinatedTaskExecutionConfig& config = CoordinatedTaskExecutionConfig());

private:
  rclcpp::Logger logger_;
  PalletizePrimitive& left_;
  PalletizePrimitive& right_;
};

}  // namespace fr3_dual_palletize
