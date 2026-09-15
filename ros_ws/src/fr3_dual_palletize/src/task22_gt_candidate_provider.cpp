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
#include <unordered_map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "fr3_dual_palletize/msg/box_state_array.hpp"
#include "fr3_dual_palletize/msg/task_dispatch_candidate.hpp"
#include "fr3_dual_palletize/msg/task_dispatch_candidate_array.hpp"
#include "fr3_dual_palletize/msg/task_world_commit.hpp"
#include "fr3_dual_palletize/palletizing_job.hpp"
#include "fr3_dual_palletize/placement_planner.hpp"
#include "fr3_dual_palletize/runtime_box_state.hpp"

namespace
{

double volume(const fr3_dual_palletize::BoxSpec& box)
{
  return box.dimensions[0] * box.dimensions[1] * box.dimensions[2];
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
    world_commit_topic_ = declare_parameter<std::string>(
      "world_commit_topic", "/task22/world_commit");
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
    world_commit_subscription_ = create_subscription<fr3_dual_palletize::msg::TaskWorldCommit>(
      world_commit_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&GroundTruthCandidateProvider::onWorldCommit, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Task22-A Ground Truth candidate provider ready: %s -> %s, commit=%s, "
      "max_candidates_per_box=%d",
      input_topic_.c_str(), output_topic_.c_str(), world_commit_topic_.c_str(),
      max_candidates_per_box_);
  }

private:
  void onBoxes(const fr3_dual_palletize::msg::BoxStateArray::SharedPtr message)
  {
    latest_boxes_ = *message;
    if (!have_episode_ || source_seed_ != message->seed)
    {
      // 新 seed 是一轮新的真实 episode。绝不携带上一轮的已放置 World Commit，
      // 以免旧 support map 污染新场景。
      source_seed_ = message->seed;
      completed_.clear();
      have_episode_ = true;
      candidate_published_for_current_state_ = false;
      RCLCPP_INFO(
        get_logger(), "Task22-D provider opened runtime episode: seed=%lu boxes=%zu",
        static_cast<unsigned long>(source_seed_), latest_boxes_.boxes.size());
    }

    // 稳定串行模式故意不随着正在搬运的 PhysX 物体逐帧重发候选。当前 dispatch
    // 被执行器用 source-pose 门禁复核；下一版候选只允许由成功的 World Commit
    // 触发，严格保证 “release/settle/commit -> next selection”。
    if (!candidate_published_for_current_state_)
    {
      publishCurrentCandidates("initial_or_committed_state");
    }
  }

  void onWorldCommit(const fr3_dual_palletize::msg::TaskWorldCommit::SharedPtr commit)
  {
    if (!have_episode_ || commit->source_seed != source_seed_)
    {
      RCLCPP_WARN(
        get_logger(), "Task22-D ignored World Commit object=%s due to episode seed mismatch.",
        commit->object_id.c_str());
      return;
    }
    const auto found = std::find_if(
      latest_boxes_.boxes.begin(), latest_boxes_.boxes.end(),
      [&commit](const auto& box) { return box.id == commit->object_id; });
    if (found == latest_boxes_.boxes.end())
    {
      RCLCPP_ERROR(
        get_logger(), "Task22-D rejected World Commit for unknown object=%s.",
        commit->object_id.c_str());
      return;
    }
    if (completed_.count(commit->object_id) != 0U)
    {
      RCLCPP_WARN(
        get_logger(), "Task22-D ignored duplicate World Commit object=%s.",
        commit->object_id.c_str());
      return;
    }
    completed_[commit->object_id] = commit->settled_pose;
    candidate_published_for_current_state_ = false;
    RCLCPP_INFO(
      get_logger(), "Task22-D WORLD COMMIT accepted: object=%s mode=%s arm=%s; "
      "rebuilding only the next serial candidate snapshot.",
      commit->object_id.c_str(), commit->coordination_mode.c_str(), commit->executed_arm.c_str());
    publishCurrentCandidates("world_commit");
  }

  void publishCurrentCandidates(const std::string& trigger)
  {
    if (!have_episode_ || latest_boxes_.boxes.empty())
    {
      return;
    }
    const auto began = std::chrono::steady_clock::now();
    std::vector<fr3_dual_palletize::BoxSpec> pending;
    std::vector<fr3_dual_palletize::PlacedBox> committed_placed;
    pending.reserve(latest_boxes_.boxes.size());
    committed_placed.reserve(completed_.size());
    for (const auto& state : latest_boxes_.boxes)
    {
      fr3_dual_palletize::BoxSpec box;
      std::string error;
      if (!fr3_dual_palletize::boxStateToBoxSpec(state, box, error))
      {
        RCLCPP_ERROR(get_logger(), "Task22-A ignored invalid BoxState: %s", error.c_str());
        return;
      }
      const auto committed = completed_.find(box.id);
      if (committed != completed_.end())
      {
        box.initial_pose = committed->second;
        committed_placed.push_back({box, committed->second});
      }
      else
      {
        pending.push_back(std::move(box));
      }
    }
    std::stable_sort(
      pending.begin(), pending.end(),
      [](const auto& first, const auto& second) { return volume(first) > volume(second); });

    fr3_dual_palletize::PlacementPlanner planner;
    fr3_dual_palletize::msg::TaskDispatchCandidateArray output;
    output.header = latest_boxes_.header;
    output.source_seed = source_seed_;
    output.scene_version = ++scene_version_;

    // 每个 pending box 都只相对“已真实落稳”的 support map 产生候选；不会把尚未
    // 执行的 preview 物体当作障碍或支撑。因为下一轮总会 World Commit 后重算，
    // 这比一次性离线虚拟码垛更稳健。
    for (const auto& box : pending)
    {
      const auto plan = planner.plan(box, region_, committed_placed);
      if (!plan.has_solution || box.grasp_candidates.empty())
      {
        RCLCPP_WARN(
          get_logger(), "Task22-A box=%s has no runtime candidate: %s",
          box.id.c_str(), plan.has_solution ? "no grasp candidate" : plan.error.c_str());
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
        for (const auto& grasp : box.grasp_candidates)
        {
          candidate.grasp_candidate_ids.push_back(grasp.id);
          candidate.grasp_candidates.push_back(grasp.local_pose);
        }
        output.candidates.push_back(std::move(candidate));
      }
    }
    candidate_published_for_current_state_ = true;
    publisher_->publish(output);
    const double wall_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - began).count();
    RCLCPP_INFO(
      get_logger(),
      "Task22-D CANDIDATES READY: trigger=%s scene_version=%lu seed=%lu pending=%zu "
      "committed=%zu candidates=%zu placement_wall=%.3f ms [serial, no MoveIt/OMPL/FCL]",
      trigger.c_str(), static_cast<unsigned long>(output.scene_version),
      static_cast<unsigned long>(output.source_seed), pending.size(), completed_.size(),
      output.candidates.size(), wall_ms);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string world_commit_topic_;
  int max_candidates_per_box_{3};
  fr3_dual_palletize::PalletRegion region_;
  std::uint64_t scene_version_{0};
  std::uint64_t source_seed_{0};
  bool have_episode_{false};
  bool candidate_published_for_current_state_{false};
  fr3_dual_palletize::msg::BoxStateArray latest_boxes_;
  std::unordered_map<std::string, geometry_msgs::msg::Pose> completed_;
  rclcpp::Publisher<fr3_dual_palletize::msg::TaskDispatchCandidateArray>::SharedPtr publisher_;
  rclcpp::Subscription<fr3_dual_palletize::msg::BoxStateArray>::SharedPtr subscription_;
  rclcpp::Subscription<fr3_dual_palletize::msg::TaskWorldCommit>::SharedPtr
    world_commit_subscription_;
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
