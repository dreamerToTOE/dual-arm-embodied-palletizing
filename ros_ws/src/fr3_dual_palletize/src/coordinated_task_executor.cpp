#include "fr3_dual_palletize/coordinated_task_executor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <utility>
#include <vector>

namespace fr3_dual_palletize
{
namespace
{
constexpr double TIME_EPSILON = 1e-6;

double pointTime(const trajectory_msgs::msg::JointTrajectoryPoint& point)
{
  return static_cast<double>(point.time_from_start.sec) +
    static_cast<double>(point.time_from_start.nanosec) * 1e-9;
}

struct ScheduledEvent
{
  double time_sec{0.0};
  PalletizePrimitive* primitive{nullptr};
  const TaskEvent* event{nullptr};
  const char* arm_name{nullptr};
};

bool validCandidate(
  const TaskTrajectoryCandidate& candidate,
  std::string& error)
{
  if (candidate.trajectory.joint_names.empty() ||
      candidate.trajectory.points.empty())
  {
    error = candidate.label + " 缺少可执行 JointTrajectory。";
    return false;
  }

  const double duration = pointTime(candidate.trajectory.points.back());
  if (duration <= 0.0)
  {
    error = candidate.label + " 的轨迹时长无效。";
    return false;
  }

  double last_event_time = -TIME_EPSILON;
  bool have_suction_on = false;
  bool have_attach = false;
  bool have_suction_off = false;
  bool have_detach = false;
  for (const auto& event : candidate.events)
  {
    if (event.time_sec + TIME_EPSILON < last_event_time ||
        event.time_sec > duration + TIME_EPSILON ||
        event.object_name != candidate.object_id)
    {
      error = candidate.label + " 的 TaskEvent 时间轴或 object_id 无效。";
      return false;
    }
    last_event_time = event.time_sec;

    switch (event.type)
    {
      case TaskEventType::SUCTION_ON:
        have_suction_on = true;
        break;
      case TaskEventType::ATTACH:
        have_attach = true;
        break;
      case TaskEventType::SUCTION_OFF:
        have_suction_off = true;
        break;
      case TaskEventType::DETACH:
        have_detach = true;
        break;
    }
  }

  if (!have_suction_on || !have_attach || !have_suction_off || !have_detach)
  {
    error = candidate.label + " 缺少完整的 SUCTION/ATTACH/DETACH 事件序列。";
    return false;
  }
  return true;
}

}  // namespace

CoordinatedTaskExecutor::CoordinatedTaskExecutor(
  rclcpp::Logger logger,
  PalletizePrimitive& left,
  PalletizePrimitive& right)
  : logger_(std::move(logger)), left_(left), right_(right)
{
}

CoordinatedTaskExecutionResult CoordinatedTaskExecutor::execute(
  const TaskTrajectoryCandidate& left_candidate,
  const TaskTrajectoryCandidate& right_candidate,
  const CoordinatedTaskExecutionConfig& config)
{
  CoordinatedTaskExecutionResult result;
  if (config.command_period_sec <= 0.0 || config.grasp_timeout_sec <= 0.0 ||
      config.release_timeout_sec <= 0.0 || config.final_hold_sec < 0.0)
  {
    result.error = "Task09 执行参数无效。";
    return result;
  }
  if (!validCandidate(left_candidate, result.error) ||
      !validCandidate(right_candidate, result.error))
  {
    return result;
  }
  if (!left_.waitForTaskExecutionBridge() ||
      !right_.waitForTaskExecutionBridge())
  {
    result.error = "Isaac joint command 或 Surface Gripper bridge 未就绪。";
    return result;
  }

  std::vector<ScheduledEvent> events;
  events.reserve(left_candidate.events.size() + right_candidate.events.size());
  for (const auto& event : left_candidate.events)
  {
    events.push_back(ScheduledEvent{event.time_sec, &left_, &event, "LEFT"});
  }
  for (const auto& event : right_candidate.events)
  {
    events.push_back(ScheduledEvent{event.time_sec, &right_, &event, "RIGHT"});
  }
  std::stable_sort(
    events.begin(), events.end(),
    [](const ScheduledEvent& first, const ScheduledEvent& second)
    {
      return first.time_sec < second.time_sec;
    });

  result.candidate_makespan_sec = std::max(
    left_candidate.duration_sec, right_candidate.duration_sec);
  RCLCPP_INFO(logger_, "========== Task09-C COORDINATED ISAAC EXECUTION ==========");
  RCLCPP_INFO(
    logger_,
    "FCL SAFE candidate accepted: makespan=%.3f s, events=%zu",
    result.candidate_makespan_sec, events.size());
  RCLCPP_INFO(
    logger_,
    "执行语义：共享时钟同步发送两臂关节命令；抓放事件期间两臂共同 HOLD。"
  );

  const auto execution_start = std::chrono::steady_clock::now();
  auto active_start = execution_start;
  std::size_t next_event = 0;
  bool failed = false;

  while (!failed)
  {
    const auto now = std::chrono::steady_clock::now();
    double active_time = std::chrono::duration<double>(now - active_start).count();
    const double event_time = next_event < events.size() ?
      events[next_event].time_sec : result.candidate_makespan_sec;

    // 不跨越事件采样：先精确发送事件时刻的双臂姿态，再共同暂停处理物理状态。
    const double command_time = std::min(active_time, event_time);
    if (!left_.publishTaskTrajectorySample(left_candidate.trajectory, command_time) ||
        !right_.publishTaskTrajectorySample(right_candidate.trajectory, command_time))
    {
      result.error = "发布双臂同步关节命令失败。";
      failed = true;
      break;
    }

    if (next_event < events.size() &&
        active_time + TIME_EPSILON >= events[next_event].time_sec)
    {
      const double hold_time = events[next_event].time_sec;
      const auto pause_start = std::chrono::steady_clock::now();
      RCLCPP_INFO(logger_, "Task09-C GLOBAL HOLD at t=%.3f s", hold_time);

      // 同一时刻的多个事件都在同一次全局 HOLD 内处理，避免两臂出现未建模的相对运动。
      while (next_event < events.size() &&
             std::abs(events[next_event].time_sec - hold_time) <= TIME_EPSILON)
      {
        const auto& scheduled = events[next_event];
        RCLCPP_INFO(
          logger_,
          "Task09-C %s event: %s at t=%.3f s",
          scheduled.arm_name,
          taskEventTypeName(scheduled.event->type),
          scheduled.time_sec);
        if (!scheduled.primitive->applyTaskEvent(
              *scheduled.event,
              config.grasp_timeout_sec,
              config.release_timeout_sec))
        {
          result.error = std::string("Task09-C ") + scheduled.arm_name +
            " event " + taskEventTypeName(scheduled.event->type) + " 失败。";
          failed = true;
          break;
        }
        ++result.events_executed;
        ++next_event;
      }

      const auto pause_end = std::chrono::steady_clock::now();
      const double pause_sec = std::chrono::duration<double>(
        pause_end - pause_start).count();
      result.physical_pause_sec += pause_sec;
      active_start += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(pause_sec));
      continue;
    }

    if (active_time >= result.candidate_makespan_sec)
    {
      break;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(
      std::min(config.command_period_sec, std::max(0.0, event_time - active_time))));
  }

  if (failed)
  {
    // 失败时停止继续推进轨迹，并让两个吸盘进入安全释放命令。机械臂保持最后命令姿态，
    // 不做未经 MoveIt/FCL 验证的恢复动作。
    left_.emergencySuctionOff();
    right_.emergencySuctionOff();
    result.wall_duration_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - execution_start).count();
    RCLCPP_ERROR(logger_, "Task09-C EXECUTION FAIL: %s", result.error.c_str());
    return result;
  }

  const auto final_hold_start = std::chrono::steady_clock::now();
  while (std::chrono::duration<double>(
           std::chrono::steady_clock::now() - final_hold_start).count() < config.final_hold_sec)
  {
    if (!left_.publishTaskTrajectorySample(
          left_candidate.trajectory, result.candidate_makespan_sec) ||
        !right_.publishTaskTrajectorySample(
          right_candidate.trajectory, result.candidate_makespan_sec))
    {
      result.error = "最终保持阶段发布关节命令失败。";
      return result;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(config.command_period_sec));
  }

  result.valid = true;
  result.completed = true;
  result.wall_duration_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - execution_start).count();
  RCLCPP_INFO(
    logger_,
    "Task09-C EXECUTION PASS: events=%zu, candidate=%.3f s, pause=%.3f s, wall=%.3f s",
    result.events_executed,
    result.candidate_makespan_sec,
    result.physical_pause_sec,
    result.wall_duration_sec);
  return result;
}

}  // namespace fr3_dual_palletize
