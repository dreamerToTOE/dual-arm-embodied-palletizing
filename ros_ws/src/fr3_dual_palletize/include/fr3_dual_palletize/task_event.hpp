#pragma once

#include <string>

namespace fr3_dual_palletize
{

// 完整任务轨迹中改变携带物状态的离散事件。
// time_sec 使用 TaskTrajectoryCandidate 的统一相对时间轴。
enum class TaskEventType
{
  // 物理 Surface Gripper 的命令边界。它们不改变 Task08-B 中 Box 的
  // World / Attached 几何状态，后者仍只由 ATTACH / DETACH 表示。
  SUCTION_ON,
  ATTACH,
  SUCTION_OFF,
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
    case TaskEventType::SUCTION_ON:
      return "SUCTION_ON";
    case TaskEventType::ATTACH:
      return "ATTACH";
    case TaskEventType::SUCTION_OFF:
      return "SUCTION_OFF";
    case TaskEventType::DETACH:
      return "DETACH";
  }

  return "UNKNOWN";
}

}  // namespace fr3_dual_palletize
