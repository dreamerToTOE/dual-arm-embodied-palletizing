#include "fr3_dual_palletize/local_wait_coordinator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

namespace fr3_dual_palletize
{
namespace
{
constexpr double TIME_EPSILON = 1e-9;
constexpr const char* WAIT_STAGE_NAME = "LIFT_TO_PRE_PLACE";

double pointTime(const trajectory_msgs::msg::JointTrajectoryPoint& point)
{
  return static_cast<double>(point.time_from_start.sec) +
         static_cast<double>(point.time_from_start.nanosec) * 1e-9;
}

void setPointTime(
  trajectory_msgs::msg::JointTrajectoryPoint& point,
  double time_sec)
{
  auto sec = static_cast<std::int32_t>(std::floor(time_sec));
  auto nanosec = static_cast<std::int64_t>(std::llround(
    (time_sec - static_cast<double>(sec)) * 1e9));
  if (nanosec >= 1000000000LL)
  {
    ++sec;
    nanosec -= 1000000000LL;
  }
  point.time_from_start.sec = sec;
  point.time_from_start.nanosec = static_cast<std::uint32_t>(nanosec);
}

std::vector<double> interpolate(
  const trajectory_msgs::msg::JointTrajectory& trajectory,
  double time_sec)
{
  const auto& points = trajectory.points;
  if (time_sec <= pointTime(points.front()))
  {
    return points.front().positions;
  }
  if (time_sec >= pointTime(points.back()))
  {
    return points.back().positions;
  }

  for (std::size_t index = 1; index < points.size(); ++index)
  {
    const double before_time = pointTime(points[index - 1]);
    const double after_time = pointTime(points[index]);
    if (time_sec > after_time)
    {
      continue;
    }

    const double span = after_time - before_time;
    if (span <= TIME_EPSILON)
    {
      return points[index].positions;
    }

    const double ratio = (time_sec - before_time) / span;
    std::vector<double> values(points[index].positions.size());
    for (std::size_t joint = 0; joint < values.size(); ++joint)
    {
      values[joint] = points[index - 1].positions[joint] +
        ratio * (points[index].positions[joint] - points[index - 1].positions[joint]);
    }
    return values;
  }
  return points.back().positions;
}

bool findLocalWaitStart(
  const TaskTrajectoryCandidate& candidate,
  double& wait_start_time_sec,
  std::string& error)
{
  for (const auto& stage : candidate.stages)
  {
    if (stage.name == WAIT_STAGE_NAME &&
        stage.end_time_sec > stage.start_time_sec + TIME_EPSILON)
    {
      wait_start_time_sec = stage.start_time_sec;
      return true;
    }
  }

  error = candidate.label +
    " 缺少 LIFT_TO_PRE_PLACE 阶段边界，无法选择局部等待点。";
  return false;
}

bool insertLocalWait(
  const TaskTrajectoryCandidate& input,
  double wait_start_time_sec,
  double wait_duration_sec,
  TaskTrajectoryCandidate& output,
  std::string& error)
{
  if (input.trajectory.points.empty() || wait_duration_sec <= 0.0)
  {
    error = "局部等待参数或轨迹无效。";
    return false;
  }

  const double first_time = pointTime(input.trajectory.points.front());
  const double final_time = pointTime(input.trajectory.points.back());
  if (wait_start_time_sec < first_time - TIME_EPSILON ||
      wait_start_time_sec >= final_time - TIME_EPSILON)
  {
    error = input.label + " 的局部等待点超出轨迹有效范围。";
    return false;
  }

  output = input;
  output.trajectory.points.clear();
  bool wait_inserted = false;

  auto appendWait = [&]()
  {
    trajectory_msgs::msg::JointTrajectoryPoint at_wait;
    at_wait.positions = interpolate(input.trajectory, wait_start_time_sec);
    at_wait.velocities.assign(at_wait.positions.size(), 0.0);
    at_wait.accelerations.assign(at_wait.positions.size(), 0.0);
    setPointTime(at_wait, wait_start_time_sec);
    output.trajectory.points.push_back(at_wait);

    auto hold = at_wait;
    setPointTime(hold, wait_start_time_sec + wait_duration_sec);
    output.trajectory.points.push_back(std::move(hold));
    wait_inserted = true;
  };

  for (const auto& original_point : input.trajectory.points)
  {
    const double original_time = pointTime(original_point);
    if (!wait_inserted && original_time >= wait_start_time_sec - TIME_EPSILON)
    {
      // 如果恰好已有轨迹点，仍通过统一插值写入一次，避免重复时间点。
      appendWait();
      if (std::abs(original_time - wait_start_time_sec) <= TIME_EPSILON)
      {
        continue;
      }
    }

    auto point = original_point;
    if (wait_inserted)
    {
      setPointTime(point, original_time + wait_duration_sec);
    }
    output.trajectory.points.push_back(std::move(point));
  }

  if (!wait_inserted)
  {
    error = input.label + " 未能插入局部等待点。";
    return false;
  }

  for (auto& event : output.events)
  {
    if (event.time_sec > wait_start_time_sec + TIME_EPSILON)
    {
      event.time_sec += wait_duration_sec;
    }
  }
  for (auto& stage : output.stages)
  {
    if (stage.start_time_sec >= wait_start_time_sec - TIME_EPSILON)
    {
      stage.start_time_sec += wait_duration_sec;
      stage.end_time_sec += wait_duration_sec;
    }
    else if (stage.end_time_sec > wait_start_time_sec + TIME_EPSILON)
    {
      stage.end_time_sec += wait_duration_sec;
    }
  }

  output.duration_sec = pointTime(output.trajectory.points.back());
  output.goal_q = output.trajectory.points.back().positions;
  return true;
}

double makespan(
  const TaskTrajectoryCandidate& left,
  const TaskTrajectoryCandidate& right)
{
  return std::max(left.duration_sec, right.duration_sec);
}

struct WaitCandidate
{
  bool found{false};
  LocalWaitStrategy strategy{LocalWaitStrategy::NO_SOLUTION};
  std::string yielding_arm;
  double wait_start_time_sec{-1.0};
  double wait_duration_sec{0.0};
  TaskTrajectoryCandidate left;
  TaskTrajectoryCandidate right;
  ConflictReport verification_report;
};

bool isBetterCandidate(
  const WaitCandidate& candidate,
  const WaitCandidate& current,
  bool prefer_left)
{
  if (!current.found)
  {
    return true;
  }
  if (candidate.wait_duration_sec + TIME_EPSILON < current.wait_duration_sec)
  {
    return true;
  }
  if (std::abs(candidate.wait_duration_sec - current.wait_duration_sec) <= TIME_EPSILON)
  {
    const double candidate_makespan = makespan(candidate.left, candidate.right);
    const double current_makespan = makespan(current.left, current.right);
    if (candidate_makespan + TIME_EPSILON < current_makespan)
    {
      return true;
    }
    if (std::abs(candidate_makespan - current_makespan) <= TIME_EPSILON)
    {
      // priority left 时，右臂等待；反之左臂等待。
      return prefer_left ?
        candidate.strategy == LocalWaitStrategy::LOCAL_WAIT_RIGHT :
        candidate.strategy == LocalWaitStrategy::LOCAL_WAIT_LEFT;
    }
  }
  return false;
}

bool hasTerminalConflict(const ConflictReport& report)
{
  if (report.conflict_windows.empty())
  {
    return false;
  }
  // 在 horizon 采样点，两条轨迹均已经到达各自的终止姿态；若该点仍冲突，
  // 单纯拉长其中一条轨迹中的 HOLD 不可能改变最终同时静止时的几何关系。
  return report.conflict_windows.back().end_time_sec >=
    report.horizon_sec - report.sample_period_sec - TIME_EPSILON;
}

}  // namespace

const char* localWaitStrategyName(LocalWaitStrategy strategy)
{
  switch (strategy)
  {
    case LocalWaitStrategy::SIMULTANEOUS: return "SIMULTANEOUS";
    case LocalWaitStrategy::LOCAL_WAIT_LEFT: return "LOCAL_WAIT_LEFT";
    case LocalWaitStrategy::LOCAL_WAIT_RIGHT: return "LOCAL_WAIT_RIGHT";
    case LocalWaitStrategy::NO_SOLUTION: return "NO_SOLUTION";
  }
  return "UNKNOWN";
}

LocalWaitCoordinator::LocalWaitCoordinator(const SpatioTemporalConflictDetector& detector)
  : detector_(detector)
{
}

LocalWaitCoordinationResult LocalWaitCoordinator::solve(
  const TaskTrajectoryCandidate& left,
  const TaskTrajectoryCandidate& right,
  const LocalWaitCoordinationConfig& config) const
{
  LocalWaitCoordinationResult result;
  result.original_makespan_sec = makespan(left, right);
  if (config.sample_period_sec <= 0.0 || config.wait_step_sec <= 0.0 ||
      config.max_wait_sec < config.wait_step_sec)
  {
    result.error = "Task09-B 局部等待参数无效。";
    return result;
  }

  result.simultaneous_report = detector_.check(left, right, config.sample_period_sec);
  if (!result.simultaneous_report.valid)
  {
    result.error = result.simultaneous_report.error;
    return result;
  }

  if (!result.simultaneous_report.conflict)
  {
    result.valid = true;
    result.coordinated = true;
    result.strategy = LocalWaitStrategy::SIMULTANEOUS;
    result.coordinated_left = left;
    result.coordinated_right = right;
    result.coordinated_makespan_sec = result.original_makespan_sec;
    result.verification_report = result.simultaneous_report;
    return result;
  }

  if (hasTerminalConflict(result.simultaneous_report))
  {
    result.valid = true;
    result.error =
      "完整候选在终止 RETREAT 姿态仍存在冲突；局部等待无法改变终态几何，"
      "需要安全退出轨迹或局部空间重规划。";
    return result;
  }

  double left_wait_start = -1.0;
  double right_wait_start = -1.0;
  if (!findLocalWaitStart(left, left_wait_start, result.error) ||
      !findLocalWaitStart(right, right_wait_start, result.error))
  {
    return result;
  }

  WaitCandidate best;
  const std::size_t steps = static_cast<std::size_t>(std::floor(
    config.max_wait_sec / config.wait_step_sec));

  for (const auto strategy : {
         config.prefer_left ? LocalWaitStrategy::LOCAL_WAIT_RIGHT :
                              LocalWaitStrategy::LOCAL_WAIT_LEFT,
         config.prefer_left ? LocalWaitStrategy::LOCAL_WAIT_LEFT :
                              LocalWaitStrategy::LOCAL_WAIT_RIGHT})
  {
    const bool wait_left = strategy == LocalWaitStrategy::LOCAL_WAIT_LEFT;
    const auto& original = wait_left ? left : right;
    const double wait_start = wait_left ? left_wait_start : right_wait_start;

    for (std::size_t step = 1; step <= steps; ++step)
    {
      const double wait_duration =
        static_cast<double>(step) * config.wait_step_sec;
      TaskTrajectoryCandidate altered;
      std::string insertion_error;
      if (!insertLocalWait(
            original, wait_start, wait_duration, altered, insertion_error))
      {
        result.error = insertion_error;
        return result;
      }

      const auto& test_left = wait_left ? altered : left;
      const auto& test_right = wait_left ? right : altered;
      auto report = detector_.check(
        test_left, test_right, config.sample_period_sec);
      ++result.schedules_checked;
      if (!report.valid)
      {
        result.error = report.error;
        return result;
      }
      if (report.conflict)
      {
        continue;
      }

      WaitCandidate candidate;
      candidate.found = true;
      candidate.strategy = strategy;
      candidate.yielding_arm = wait_left ? "LEFT" : "RIGHT";
      candidate.wait_start_time_sec = wait_start;
      candidate.wait_duration_sec = wait_duration;
      candidate.left = test_left;
      candidate.right = test_right;
      candidate.verification_report = std::move(report);
      if (isBetterCandidate(candidate, best, config.prefer_left))
      {
        best = std::move(candidate);
      }
      // 当前让行臂按递增 Δt 搜索；它的首个 SAFE 候选即为该臂最小等待。
      break;
    }
  }

  result.valid = true;
  if (!best.found)
  {
    result.error = "在给定 max_wait_sec 内，LIFT 后局部等待未找到安全解。";
    return result;
  }

  result.coordinated = true;
  result.strategy = best.strategy;
  result.yielding_arm = best.yielding_arm;
  result.wait_start_time_sec = best.wait_start_time_sec;
  result.wait_duration_sec = best.wait_duration_sec;
  result.coordinated_left = std::move(best.left);
  result.coordinated_right = std::move(best.right);
  result.coordinated_makespan_sec = makespan(
    result.coordinated_left, result.coordinated_right);
  result.verification_report = std::move(best.verification_report);
  return result;
}

}  // namespace fr3_dual_palletize
