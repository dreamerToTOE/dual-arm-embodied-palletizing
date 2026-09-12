#pragma once

#include <string>

#include "fr3_dual_palletize/palletizing_job.hpp"

namespace fr3_dual_palletize
{

// 从 YAML 读取并完整校验 Task17 数据模型。错误通过 error 返回，调用者可以安全地
// 拒绝启动任务，而不会把半完整参数送入 MoveIt / Isaac。
bool loadPalletizingJobYaml(
  const std::string& path,
  PalletizingJob& job,
  std::string& error);

bool validatePalletizingJob(const PalletizingJob& job, std::string& error);

}  // namespace fr3_dual_palletize
