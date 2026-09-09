#include "fr3_dual_palletize/temporal_coordinator.hpp"

#include <algorithm>
#include <cmath>

namespace fr3_dual_palletize
{

const char* temporalStrategyName(TemporalStrategy strategy)
{
  switch (strategy)
  {
    case TemporalStrategy::SIMULTANEOUS: return "SIMULTANEOUS";
    case TemporalStrategy::DELAY_LEFT: return "DELAY_LEFT";
    case TemporalStrategy::DELAY_RIGHT: return "DELAY_RIGHT";
    case TemporalStrategy::NO_SOLUTION: return "NO_SOLUTION";
  }
  return "UNKNOWN";
}

TemporalCoordinator::TemporalCoordinator(const SpatioTemporalConflictDetector& detector)
  : detector_(detector)
{
}

TemporalCoordinationPlan TemporalCoordinator::solve(
  const TaskTrajectoryCandidate& left,
  const TaskTrajectoryCandidate& right,
  const TemporalCoordinationConfig& config) const
{
  TemporalCoordinationPlan plan;
  if (config.sample_period_sec <= 0.0 || config.delay_step_sec <= 0.0 ||
      config.max_delay_sec < 0.0)
  {
    plan.error = "Task09 参数无效。";
    return plan;
  }

  plan.simultaneous_report = detector_.check(
    left, right, config.sample_period_sec);
  if (!plan.simultaneous_report.valid)
  {
    plan.error = plan.simultaneous_report.error;
    return plan;
  }

  if (!plan.simultaneous_report.conflict)
  {
    plan.valid = true;
    plan.coordinated = true;
    plan.strategy = TemporalStrategy::SIMULTANEOUS;
    plan.scheduled_report = plan.simultaneous_report;
    return plan;
  }

  const std::size_t steps = static_cast<std::size_t>(
    std::floor(config.max_delay_sec / config.delay_step_sec));
  for (std::size_t step = 1; step <= steps; ++step)
  {
    const double delay = static_cast<double>(step) * config.delay_step_sec;
    const TemporalStrategy first = config.prefer_left ?
      TemporalStrategy::DELAY_RIGHT : TemporalStrategy::DELAY_LEFT;
    const TemporalStrategy second = config.prefer_left ?
      TemporalStrategy::DELAY_LEFT : TemporalStrategy::DELAY_RIGHT;

    for (const auto strategy : {first, second})
    {
      const double left_delay = strategy == TemporalStrategy::DELAY_LEFT ? delay : 0.0;
      const double right_delay = strategy == TemporalStrategy::DELAY_RIGHT ? delay : 0.0;
      auto report = detector_.check(
        left, right, config.sample_period_sec, left_delay, right_delay);
      ++plan.schedules_checked;
      if (!report.valid)
      {
        plan.error = report.error;
        return plan;
      }
      if (!report.conflict)
      {
        plan.valid = true;
        plan.coordinated = true;
        plan.strategy = strategy;
        plan.left_start_delay_sec = left_delay;
        plan.right_start_delay_sec = right_delay;
        plan.scheduled_report = std::move(report);
        return plan;
      }
    }
  }

  plan.valid = true;
  plan.error = "在给定 max_delay_sec 内未找到安全的单臂延迟启动策略。";
  return plan;
}

}  // namespace fr3_dual_palletize
