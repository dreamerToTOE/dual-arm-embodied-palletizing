#pragma once

#include <string>

namespace fr3_dual_palletize
{

// 完整任务轨迹中改变携带物状态的离散事件。
// time_sec 使用 TaskTrajectoryCandidate 的统一相对时间轴。
enum class TaskEventType
{
  ATTACH,
  DETACH,
};

struct TaskEvent
{
  double time_sec{0.0};
  TaskEventType type{TaskEventType::ATTACH};
  std::string object_name;
  std::string link_name;
};

inline const char* taskEventTypeName(TaskEventType type)
{
  switch (type)
  {
    case TaskEventType::ATTACH:
      return "ATTACH";
    case TaskEventType::DETACH:
      return "DETACH";
  }

  return "UNKNOWN";
}

}  // namespace fr3_dual_palletize
