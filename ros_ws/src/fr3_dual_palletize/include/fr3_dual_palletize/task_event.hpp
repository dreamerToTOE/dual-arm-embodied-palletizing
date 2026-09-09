#pragma once

#include <string>

namespace fr3_dual_palletize
{

enum class TaskEventType
{
  ATTACH,
  DETACH
};

struct TaskEvent
{
  double time_sec{0.0};
  TaskEventType type{TaskEventType::ATTACH};
  std::string object_name;
};

}  // namespace fr3_dual_palletize
