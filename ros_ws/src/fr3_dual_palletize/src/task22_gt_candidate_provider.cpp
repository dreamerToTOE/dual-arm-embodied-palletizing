// Task22-A：将 Task18 Ground Truth 与 Task19 placement planner 连接成只读候选流。
//
// 本节点不创建 MoveGroup、不调用 OMPL/FCL、不发布 robot/suction command。它只负责
// 生成带 scene_version 的 object + pick + placement candidates，供 Python selector
// 做快速调度；后续 Task22-B Gateway 才能消费 TaskDispatch 并执行最终安全验证。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "fr3_dual_palletize/msg/box_state_array.hpp"
#include "fr3_dual_palletize/msg/task_dispatch_candidate.hpp"
#include "fr3_dual_palletize/msg/task_dispatch_candidate_array.hpp"
#include "fr3_dual_palletize/palletizing_job.hpp"
#include "fr3_dual_palletize/placement_planner.hpp"
#include "fr3_dual_palletize/runtime_box_state.hpp"

namespace
{

double volume(const fr3_dual_palletize::BoxSpec& box)
{
  return box.dimensions[0] * box.dimensions[1] * box.dimensions[2];
}

long long quantize(double value, double resolution)
{
  return static_cast<long long>(std::llround(value / resolution));
}

std::string snapshotSignature(const fr3_dual_palletize::msg::BoxStateArray& message)
{
  // Ground Truth 在 PhysX 静止期仍可能有极小浮点扰动。按 1 mm / 0.01 kg 离散化，
  // 只有任务相关几何真实变化才递增 scene_version，避免 selector 每帧重复决策。
  std::vector<const fr3_dual_palletize::msg::BoxState*> sorted;
  sorted.reserve(message.boxes.size());
  for (const auto& box : message.boxes)
  {
    sorted.push_back(&box);
  }
  std::sort(sorted.begin(), sorted.end(), [](const auto* first, const auto* second)
    { return first->id < second->id; });

  std::ostringstream stream;
  stream << message.seed << '|';
  for (const auto* box : sorted)
  {
    stream << box->id << ':'
           << quantize(box->pose.position.x, 1.0e-3) << ','
           << quantize(box->pose.position.y, 1.0e-3) << ','
           << quantize(box->pose.position.z, 1.0e-3) << ','
           << quantize(box->size[0], 1.0e-3) << ','
           << quantize(box->size[1], 1.0e-3) << ','
           << quantize(box->size[2], 1.0e-3) << ','
           << quantize(box->mass, 1.0e-2) << ';';
  }
  return stream.str();
}

class GroundTruthCandidateProvider final : public rclcpp::Node
{
public:
  GroundTruthCandidateProvider()
  : Node("task22_gt_candidate_provider")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/task18/box_states");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/task22/dispatch_candidates");
    max_candidates_per_box_ = declare_parameter<int>("max_candidates_per_box", 3);
    region_.frame_id = "world";
    region_.min_x = declare_parameter<double>("pallet_min_x", 0.555);
    region_.max_x = declare_parameter<double>("pallet_max_x", 0.785);
    region_.min_y = declare_parameter<double>("pallet_min_y", -0.155);
    region_.max_y = declare_parameter<double>("pallet_max_y", 0.145);
    region_.support_height = declare_parameter<double>("pallet_support_height", 0.050);

    if (max_candidates_per_box_ <= 0 || region_.min_x >= region_.max_x ||
        region_.min_y >= region_.max_y)
    {
      throw std::runtime_error("Task22-A provider 参数无效。");
    }

    publisher_ = create_publisher<fr3_dual_palletize::msg::TaskDispatchCandidateArray>(
      output_topic_, rclcpp::QoS(1).reliable().transient_local());
    subscription_ = create_subscription<fr3_dual_palletize::msg::BoxStateArray>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&GroundTruthCandidateProvider::onBoxes, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Task22-A Ground Truth candidate provider ready: %s -> %s, max_candidates_per_box=%d",
      input_topic_.c_str(), output_topic_.c_str(), max_candidates_per_box_);
  }

private:
  void onBoxes(const fr3_dual_palletize::msg::BoxStateArray::SharedPtr message)
  {
    const auto signature = snapshotSignature(*message);
    if (signature == last_signature_)
    {
      return;
    }

    std::vector<fr3_dual_palletize::BoxSpec> boxes;
    boxes.reserve(message->boxes.size());
    for (const auto& state : message->boxes)
    {
      fr3_dual_palletize::BoxSpec box;
      std::string error;
      if (!fr3_dual_palletize::boxStateToBoxSpec(state, box, error))
      {
        RCLCPP_ERROR(get_logger(), "Task22-A 忽略无效 BoxState: %s", error.c_str());
        return;
      }
      boxes.push_back(std::move(box));
    }
    if (boxes.empty())
    {
      RCLCPP_WARN(get_logger(), "Task22-A 收到空 BoxStateArray，不发布候选。");
      return;
    }

    std::stable_sort(
      boxes.begin(), boxes.end(),
      [](const auto& first, const auto& second) { return volume(first) > volume(second); });

    const auto began = std::chrono::steady_clock::now();
    fr3_dual_palletize::PlacementPlanner planner;
    std::vector<fr3_dual_palletize::PlacedBox> preview_placed;
    fr3_dual_palletize::msg::TaskDispatchCandidateArray output;
    output.header = message->header;
    output.source_seed = message->seed;
    output.scene_version = ++scene_version_;

    for (const auto& box : boxes)
    {
      const auto plan = planner.plan(box, region_, preview_placed);
      if (!plan.has_solution)
      {
        RCLCPP_WARN(
          get_logger(), "Task22-A box=%s 没有当前几何 placement candidate: %s",
          box.id.c_str(), plan.error.c_str());
        continue;
      }

      const auto count = std::min(
        plan.candidates.size(), static_cast<std::size_t>(max_candidates_per_box_));
      const auto pick_pose = fr3_dual_palletize::composePose(
        box.initial_pose, box.grasp_candidates.front().local_pose);
      for (std::size_t index = 0; index < count; ++index)
      {
        const auto& source = plan.candidates[index];
        fr3_dual_palletize::msg::TaskDispatchCandidate candidate;
        candidate.candidate_id = "task22:" + std::to_string(output.scene_version) + ":" +
          box.id + ":" + std::to_string(index);
        candidate.object_id = box.id;
        candidate.object_pose = box.initial_pose;
        candidate.pick_pose = pick_pose;
        candidate.target_pose = source.placement.target_pose;
        candidate.support_surface_id = source.placement.support_surface_id;
        candidate.support_height = source.placement.support_height;
        candidate.placement_cost = source.selection_cost;
        candidate.placement_rank = static_cast<std::uint32_t>(index);
        candidate.size = {box.dimensions[0], box.dimensions[1], box.dimensions[2]};
        candidate.mass = box.mass;
        candidate.payload_class = box.payload_class;
        candidate.allowed_modes = box.allowed_modes;
        candidate.grasp_candidate_ids.reserve(box.grasp_candidates.size());
        candidate.grasp_candidates.reserve(box.grasp_candidates.size());
        for (const auto& grasp : box.grasp_candidates)
        {
          candidate.grasp_candidate_ids.push_back(grasp.id);
          candidate.grasp_candidates.push_back(grasp.local_pose);
        }
        output.candidates.push_back(std::move(candidate));
      }

      // 这是 batch preview 的几何上下文，不是物理 World Commit。Python 后续选择的
      // candidate 仍会在 Task22-B 中按真实 scene_version 重新验证。
      preview_placed.push_back({box, plan.chosen.placement.target_pose});
    }

    if (output.candidates.empty())
    {
      RCLCPP_ERROR(get_logger(), "Task22-A 当前 snapshot 没有可发布的候选。");
      return;
    }

    last_signature_ = signature;
    publisher_->publish(output);
    const double wall_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - began).count();
    RCLCPP_INFO(
      get_logger(),
      "Task22-A CANDIDATES READY: scene_version=%lu seed=%lu boxes=%zu candidates=%zu "
      "placement_wall=%.3f ms [read-only, no MoveIt/OMPL/FCL]",
      static_cast<unsigned long>(output.scene_version),
      static_cast<unsigned long>(output.source_seed),
      boxes.size(), output.candidates.size(), wall_ms);
  }

  std::string input_topic_;
  std::string output_topic_;
  int max_candidates_per_box_{3};
  fr3_dual_palletize::PalletRegion region_;
  std::uint64_t scene_version_{0};
  std::string last_signature_;
  rclcpp::Publisher<fr3_dual_palletize::msg::TaskDispatchCandidateArray>::SharedPtr publisher_;
  rclcpp::Subscription<fr3_dual_palletize::msg::BoxStateArray>::SharedPtr subscription_;
};

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<GroundTruthCandidateProvider>());
  }
  catch (const std::exception& exception)
  {
    RCLCPP_FATAL(rclcpp::get_logger("task22_gt_candidate_provider"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
