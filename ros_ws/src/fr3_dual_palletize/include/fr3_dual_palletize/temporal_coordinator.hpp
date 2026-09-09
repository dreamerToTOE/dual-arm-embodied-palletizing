#pragma once

#include <cstddef>
#include <string>

#include "fr3_dual_palletize/spatiotemporal_conflict_detector.hpp"

namespace fr3_dual_palletize
{

enum class TemporalStrategy { SIMULTANEOUS, DELAY_LEFT, DELAY_RIGHT, NO_SOLUTION };

const char* temporalStrategyName(TemporalStrategy strategy);

struct TemporalCoordinationConfig
{
  double sample_period_sec{0.01};
  double delay_step_sec{0.25};
  double max_delay_sec{30.0};
  // 首轮优先保证 left 先行，因此同一 delay 下先尝试 delay right。
  bool prefer_left{true};
};

struct TemporalCoordinationPlan
{
  bool valid{false};
  bool coordinated{false};
  TemporalStrategy strategy{TemporalStrategy::NO_SOLUTION};
  double left_start_delay_sec{0.0};
  double right_start_delay_sec{0.0};
  std::size_t schedules_checked{0};
  ConflictReport simultaneous_report;
  ConflictReport scheduled_report;
  std::string error;
};

// Task09-A：不改轨迹、不执行机器人，仅搜索使完整候选预测安全的延迟启动策略。
class TemporalCoordinator
{
public:
  explicit TemporalCoordinator(const SpatioTemporalConflictDetector& detector);

  TemporalCoordinationPlan solve(
    const TaskTrajectoryCandidate& left,
    const TaskTrajectoryCandidate& right,
    const TemporalCoordinationConfig& config = TemporalCoordinationConfig()) const;

private:
  const SpatioTemporalConflictDetector& detector_;
};

}  // namespace fr3_dual_palletize
