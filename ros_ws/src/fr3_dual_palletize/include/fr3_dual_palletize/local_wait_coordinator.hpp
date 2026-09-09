#pragma once

#include <cstddef>
#include <string>

#include "fr3_dual_palletize/spatiotemporal_conflict_detector.hpp"

namespace fr3_dual_palletize
{

enum class LocalWaitStrategy
{
  SIMULTANEOUS,
  LOCAL_WAIT_LEFT,
  LOCAL_WAIT_RIGHT,
  NO_SOLUTION,
};

const char* localWaitStrategyName(LocalWaitStrategy strategy);

struct LocalWaitCoordinationConfig
{
  double sample_period_sec{0.01};
  double wait_step_sec{0.05};
  double max_wait_sec{15.0};
  // true 表示左臂优先通过公共工作区；相同代价时优先让右臂等待。
  bool prefer_left{true};
};

struct LocalWaitCoordinationResult
{
  bool valid{false};
  bool coordinated{false};
  LocalWaitStrategy strategy{LocalWaitStrategy::NO_SOLUTION};
  std::string yielding_arm;
  double wait_start_time_sec{-1.0};
  double wait_duration_sec{0.0};
  double original_makespan_sec{0.0};
  double coordinated_makespan_sec{0.0};
  std::size_t schedules_checked{0};
  ConflictReport simultaneous_report;
  ConflictReport verification_report;
  TaskTrajectoryCandidate coordinated_left;
  TaskTrajectoryCandidate coordinated_right;
  std::string error;
};

// Task09-B：保持 MoveIt 给出的空间路径不变，只在 LIFT 完成、进入公共工作区
// 之前插入局部 HOLD。每个候选仍由 Task08-B 的完整几何时空检查验收。
class LocalWaitCoordinator
{
public:
  explicit LocalWaitCoordinator(const SpatioTemporalConflictDetector& detector);

  LocalWaitCoordinationResult solve(
    const TaskTrajectoryCandidate& left,
    const TaskTrajectoryCandidate& right,
    const LocalWaitCoordinationConfig& config = LocalWaitCoordinationConfig()) const;

private:
  const SpatioTemporalConflictDetector& detector_;
};

}  // namespace fr3_dual_palletize
