// Task20-A：多尺寸随机 episode 的只读任务规划与统计。
//
// 输入只来自 Task18 /task18/box_states。节点将每个运行时 BoxState 转为 Task17
// BoxSpec，再交给 Task19 PlacementPlanner 连续生成 target。它不启动 MoveIt、不
// 发布关节/吸盘命令；Task20-B 才把这些通用任务接入 Task16/Task08/Task21。

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "fr3_dual_palletize/msg/box_state_array.hpp"
#include "fr3_dual_palletize/placement_planner.hpp"
#include "fr3_dual_palletize/runtime_box_state.hpp"

namespace
{

class InputBuffer
{
public:
  explicit InputBuffer(const rclcpp::Node::SharedPtr& node)
  {
    subscription_ = node->create_subscription<fr3_dual_palletize::msg::BoxStateArray>(
      "/task18/box_states", rclcpp::QoS(10),
      [this](const fr3_dual_palletize::msg::BoxStateArray::SharedPtr message)
      {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          state_ = *message;
          received_ = true;
        }
        condition_.notify_all();
      });
  }

  bool wait(double timeout_sec)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, std::chrono::duration<double>(timeout_sec),
      [this]() { return received_; });
  }

  fr3_dual_palletize::msg::BoxStateArray state() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool received_{false};
  fr3_dual_palletize::msg::BoxStateArray state_;
  rclcpp::Subscription<fr3_dual_palletize::msg::BoxStateArray>::SharedPtr subscription_;
};

double volume(const fr3_dual_palletize::BoxSpec& box)
{
  return box.dimensions[0] * box.dimensions[1] * box.dimensions[2];
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task20_multi_size_episode");
  const double timeout_sec = node->declare_parameter<double>("timeout_sec", 15.0);
  const auto expected_seed = node->declare_parameter<std::int64_t>("expected_seed", -1);
  const auto episode_index = node->declare_parameter<std::int64_t>("episode_index", 0);
  fr3_dual_palletize::PalletRegion region;
  region.frame_id = "world";
  region.min_x = node->declare_parameter<double>("pallet_min_x", 0.500);
  region.max_x = node->declare_parameter<double>("pallet_max_x", 1.050);
  region.min_y = node->declare_parameter<double>("pallet_min_y", -0.300);
  region.max_y = node->declare_parameter<double>("pallet_max_y", 0.300);
  region.support_height = node->declare_parameter<double>("pallet_support_height", 0.050);
  if (timeout_sec <= 0.0 || region.min_x >= region.max_x || region.min_y >= region.max_y)
  {
    RCLCPP_ERROR(node->get_logger(), "Task20 参数无效：timeout 或 pallet_region 边界错误。");
    rclcpp::shutdown(); return 1;
  }

  InputBuffer buffer(node);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  RCLCPP_INFO(node->get_logger(),
    "Task20-A 只读 multi-size episode 已启动：等待 /task18/box_states。");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some();
    if (buffer.wait(0.02)) break;
  }
  const auto input = buffer.state();
  if (input.boxes.empty() ||
      (expected_seed >= 0 && input.seed != static_cast<std::uint64_t>(expected_seed)))
  {
    RCLCPP_ERROR(node->get_logger(), "Task20 FAIL：没有有效运行时输入或 seed 不匹配。");
    rclcpp::shutdown(); return 1;
  }

  std::vector<fr3_dual_palletize::BoxSpec> boxes;
  std::set<std::string> classes;
  for (const auto& state : input.boxes)
  {
    fr3_dual_palletize::BoxSpec box;
    std::string error;
    if (!fr3_dual_palletize::boxStateToBoxSpec(state, box, error))
    {
      RCLCPP_ERROR(node->get_logger(), "Task20 FAIL：BoxState %s 无效：%s", state.id.c_str(), error.c_str());
      rclcpp::shutdown(); return 1;
    }
    classes.insert(box.payload_class);
    boxes.push_back(std::move(box));
  }
  // 大件先落在低层，减少后续高度与支撑复杂度；这只定义 planning order，不定义
  // 松/紧协调模式，后者由 Task21 Router 处理。
  std::stable_sort(boxes.begin(), boxes.end(),
    [](const auto& first, const auto& second) { return volume(first) > volume(second); });

  fr3_dual_palletize::PlacementPlanner planner;
  std::vector<fr3_dual_palletize::PlacedBox> placed;
  std::size_t planning_failures = 0;
  std::size_t candidates_total = 0;
  for (const auto& box : boxes)
  {
    const auto plan = planner.plan(box, region, placed);
    candidates_total += plan.candidates.size();
    if (!plan.has_solution)
    {
      ++planning_failures;
      RCLCPP_ERROR(node->get_logger(), "Box=%s PLAN_FAIL: %s", box.id.c_str(), plan.error.c_str());
      continue;
    }
    const auto& pose = plan.chosen.placement.target_pose;
    RCLCPP_INFO(node->get_logger(),
      "Box=%s class=%s size=(%.3f, %.3f, %.3f) candidates=%zu target=(%.3f, %.3f, %.3f) support=%s model=%s",
      box.id.c_str(), box.payload_class.c_str(), box.dimensions[0], box.dimensions[1], box.dimensions[2],
      plan.candidates.size(), pose.position.x, pose.position.y, pose.position.z,
      plan.chosen.placement.support_surface_id.c_str(), plan.selection_model_id.c_str());
    placed.push_back({box, pose});
  }

  RCLCPP_INFO(node->get_logger(),
    "========== Task20-A EPISODE STATS ==========");
  RCLCPP_INFO(node->get_logger(),
    "episode=%ld seed=%lu input_boxes=%zu payload_classes=%zu planned=%zu planning_failures=%zu candidates_total=%zu",
    episode_index, static_cast<unsigned long>(input.seed), boxes.size(), classes.size(), placed.size(),
    planning_failures, candidates_total);
  RCLCPP_INFO(node->get_logger(),
    "execution/IK/FCL/replans=NOT_RUN（Task20-A 为只读输入与自动 placement 验证；不执行机械臂）。");
  if (planning_failures != 0)
  {
    RCLCPP_ERROR(node->get_logger(), "Task20-A FAIL：seed=%lu 可复现。", static_cast<unsigned long>(input.seed));
    rclcpp::shutdown(); return 1;
  }
  RCLCPP_INFO(node->get_logger(),
    "Task20-A PASS：随机运行时 BoxSpec 已生成连续多尺寸 PlacementSpec；未使用固定物体数量、尺寸或目标坐标。");
  rclcpp::shutdown(); return 0;
}
