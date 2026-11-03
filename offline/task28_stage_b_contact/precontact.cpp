#include "contact_observation.hpp"
#include "task28_b0_snapshot.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <moveit/lma_kinematics_plugin/lma_kinematics_plugin.h>
#include <moveit/planning_scene/planning_scene.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/RandomNumbers.h>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <urdf_parser/urdf_parser.h>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace
{
constexpr double kHalf = .060;
constexpr double kGap = .003;
constexpr double kWorldShift = .100;
constexpr double kMaxJointStep = .005;
const std::array<std::string, 5> kWorldIds = {"task26_cube_1", "task26_table", "task26_box_wall_deep",
                                              "task26_box_wall_minus_y", "task26_box_wall_plus_y"};
using Joints = std::array<double, 7>;
struct Fixture
{
  Eigen::Vector3d cube;
  Eigen::Quaterniond orientation;
  Joints left{}, right{};
};
std::string readFile(const char *path)
{
  std::ifstream stream(path);
  if (!stream)
    throw std::runtime_error("cannot open file");
  std::stringstream bytes;
  bytes << stream.rdbuf();
  return bytes.str();
}
Fixture fixtureFrom(const YAML::Node &root)
{
  if (!root.IsMap() || root["frame"].as<std::string>() != "world" || root["physics_step"].as<int>() <= 0 ||
      !std::isfinite(root["simulation_time_s"].as<double>()) || root["simulation_time_s"].as<double>() <= 0 ||
      !std::isfinite(root["rail_world_shift_x_m"].as<double>()) ||
      std::abs(root["rail_world_shift_x_m"].as<double>() - kWorldShift) > 1e-9)
    throw std::runtime_error("invalid fixture metadata");
  std::array<double, 3> pos{};
  std::array<double, 4> quat{};
  Fixture f;
  if (!task28_b0::finiteArray(root["cube_xyz_m"], pos) ||
      !task28_b0::finiteArray(root["cube_quaternion_wxyz"], quat) ||
      !task28_b0::finiteArray(root["left_q_rad"], f.left) ||
      !task28_b0::finiteArray(root["right_q_rad"], f.right))
    throw std::runtime_error("missing or non-finite fixture state");
  f.cube = Eigen::Vector3d(pos[0] - kWorldShift, pos[1], pos[2]);
  f.orientation = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]);
  if (std::abs(f.orientation.norm() - 1.0) > .001)
    throw std::runtime_error("invalid cube quaternion");
  f.orientation.normalize();
  return f;
}
void addBox(planning_scene::PlanningScene &scene, const std::string &id, const Eigen::Vector3d &position,
            const Eigen::Vector3d &size,
            const Eigen::Quaterniond &orientation = Eigen::Quaterniond::Identity())
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = id;
  object.operation = object.ADD;
  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = {size.x(), size.y(), size.z()};
  object.primitives.push_back(box);
  geometry_msgs::msg::Pose pose;
  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();
  pose.orientation.w = orientation.w();
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  object.primitive_poses.push_back(pose);
  if (!scene.processCollisionObjectMsg(object))
    throw std::runtime_error("collision object rejected: " + id);
}
void addWorld(planning_scene::PlanningScene &scene, const Fixture &f, bool include_cube)
{
  // Identical Task27 table/walls and rail-frame shift used by the historical B0 FCL replay.
  if (include_cube)
    addBox(scene, kWorldIds[0], f.cube, {.12, .12, .12}, f.orientation);
  addBox(scene, kWorldIds[1], {.45, 0, .1}, {1.5, .8, .2});
  addBox(scene, kWorldIds[2], {1.07, 0, .275}, {.02, .646, .15});
  addBox(scene, kWorldIds[3], {.935, -.313, .275}, {.29, .02, .15});
  addBox(scene, kWorldIds[4], {.935, .313, .275}, {.29, .02, .15});
}
bool completeWorld(const planning_scene::PlanningScene &scene)
{
  for (const auto &id : kWorldIds)
    if (!scene.getWorld()->hasObject(id))
      return false;
  return true;
}
std::vector<std::string> names(const std::string &side)
{
  std::vector<std::string> out;
  for (int i = 1; i <= 7; ++i)
    out.push_back(side + "_fr3_joint" + std::to_string(i));
  return out;
}
bool validState(const planning_scene::PlanningScene &scene, const moveit::core::RobotModelConstPtr &model,
                const std::vector<std::string> &left_names, const std::vector<std::string> &right_names,
                const Joints &left, const Joints &right)
{
  if (!completeWorld(scene))
    return false;
  for (double q : left)
    if (!std::isfinite(q))
      return false;
  for (double q : right)
    if (!std::isfinite(q))
      return false;
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  state.setVariablePositions(left_names, std::vector<double>(left.begin(), left.end()));
  state.setVariablePositions(right_names, std::vector<double>(right.begin(), right.end()));
  state.update();
  collision_detection::CollisionRequest request;
  collision_detection::CollisionResult result;
  scene.checkCollision(request, result, state);
  return !result.collision && state.satisfiesBounds(model->getJointModelGroup("left_arm")) &&
         state.satisfiesBounds(model->getJointModelGroup("right_arm"));
}
bool validPath(const planning_scene::PlanningScene &scene, const moveit::core::RobotModelConstPtr &model,
               const std::vector<std::string> &ln, const std::vector<std::string> &rn,
               const std::vector<Joints> &path, const Joints &right, std::size_t *checked)
{
  if (checked)
    *checked = 0;
  if (path.size() < 2 || !completeWorld(scene))
    return false;
  for (std::size_t i = 0; i < path.size(); ++i)
  {
    if (i)
      for (std::size_t j = 0; j < 7; ++j)
        if (std::abs(path[i][j] - path[i - 1][j]) > kMaxJointStep + 1e-9)
          return false;
    if (!validState(scene, model, ln, rn, path[i], right))
      return false;
    if (checked)
      ++*checked;
  }
  return true;
}
void require(bool condition, const char *message)
{
  if (!condition)
    throw std::runtime_error(message);
}
} // namespace

int main(int argc, char **argv)
{
  if (argc != 5)
  {
    std::cerr << "usage: precontact ROBOT_URDF ROBOT_SRDF FIXTURE_JSON OUTPUT_CSV\n";
    return 2;
  }
  try
  {
    ompl::RNG::setSeed(28);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("task28_stage_b_precontact_offline");
    auto urdf = urdf::parseURDF(readFile(argv[1]));
    auto srdf = std::make_shared<srdf::Model>();
    require(urdf && srdf->initString(*urdf, readFile(argv[2])), "invalid robot model");
    auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    const YAML::Node input = YAML::LoadFile(argv[3]);
    const Fixture f = fixtureFrom(input);
    const auto ln = names("left"), rn = names("right");
    auto scene = std::make_shared<planning_scene::PlanningScene>(model);
    addWorld(*scene, f, true);
    require(completeWorld(*scene) && validState(*scene, model, ln, rn, f.left, f.right),
            "invalid initial state");

    // Negative controls: missing state, wrong frame, absent world object, and absent/unvalidated force.
    auto bad = YAML::Clone(input);
    bad.remove("left_q_rad");
    bool rejected = false;
    try
    {
      (void)fixtureFrom(bad);
    }
    catch (const std::exception &)
    {
      rejected = true;
    }
    require(rejected, "missing joint state accepted");
    bad = YAML::Clone(input);
    bad["frame"] = "map";
    rejected = false;
    try
    {
      (void)fixtureFrom(bad);
    }
    catch (const std::exception &)
    {
      rejected = true;
    }
    require(rejected, "incorrect frame accepted");
    bad = YAML::Clone(input);
    bad["left_q_rad"][0] = std::numeric_limits<double>::quiet_NaN();
    rejected = false;
    try
    {
      (void)fixtureFrom(bad);
    }
    catch (const std::exception &)
    {
      rejected = true;
    }
    require(rejected, "non-finite fixture state accepted");
    auto incomplete_scene = std::make_shared<planning_scene::PlanningScene>(model);
    addWorld(*incomplete_scene, f, false);
    require(!validState(*incomplete_scene, model, ln, rn, f.left, f.right), "missing Cube accepted");
    require(!task28_b0::legacySidePressAllowed(true), "B0 legacy Cube-removal bypass enabled");
    require(task28_b0::legacySidePressAllowed(false), "non-B0 legacy behavior changed");
    ContactObservation contact;
    require(!contactObservationUsable(contact), "missing contact force accepted");
    contact.state_valid = contact.force_observed = contact.world_frame = true;
    contact.state_step = contact.force_step = 1;
    contact.normal_world = {0, 1, 0};
    require(!contactObservationUsable(contact), "unvalidated contact force accepted");
    contact.force_source_validated = true;
    contact.force_source = ForceSource::JointTorque;
    require(!contactObservationUsable(contact), "commanded joint torque accepted as contact force");
    contact.force_source = ForceSource::ModelOnly;
    require(!contactObservationUsable(contact), "unvalidated model force accepted");
    contact.force_source = ForceSource::MeasuredContact;
    require(contactObservationUsable(contact), "valid synthetic contact observation rejected");
    contact.world_frame = false;
    require(!contactObservationUsable(contact), "wrong force frame accepted");
    contact.world_frame = true;
    contact.normal_world = {0, 0, 0};
    require(!contactObservationUsable(contact), "invalid contact normal accepted");
    contact.normal_world = {0, 1, 0};
    contact.force_source = ForceSource::IndependentlyValidatedEstimate;
    require(contactObservationUsable(contact), "independently validated synthetic estimate rejected");
    contact.force_source_validated = false;
    require(!contactObservationUsable(contact), "estimate missing validation accepted");
    contact.force_source_validated = true;
    contact.force_step = 2;
    require(!contactObservationUsable(contact), "unsynchronized contact force accepted");
    contact.force_step = 1;
    contact.force_world[1] = std::numeric_limits<double>::quiet_NaN();
    require(!contactObservationUsable(contact), "non-finite force accepted");

    // The target is the -Y face of the measured Cube, offset outward by 3 mm.
    const Eigen::Vector3d contact_p = f.cube + f.orientation * Eigen::Vector3d(0, -(kHalf + kGap), 0);
    const Eigen::Vector3d target_p = contact_p + Eigen::Vector3d(0, 0, .2);
    const Eigen::Quaterniond target_q =
        f.orientation * Eigen::Quaterniond(0, std::sqrt(.5), std::sqrt(.5), 0);
    geometry_msgs::msg::Pose target;
    target.position.x = target_p.x();
    target.position.y = target_p.y();
    target.position.z = target_p.z();
    target.orientation.x = target_q.x();
    target.orientation.y = target_q.y();
    target.orientation.z = target_q.z();
    target.orientation.w = target_q.w();
    lma_kinematics_plugin::LMAKinematicsPlugin ik;
    require(ik.initialize(node, *model, "left_arm", "world", {"left_fr3_side_suction_tcp"}, .005),
            "IK init failed");
    require(ik.getJointNames() == ln, "IK joint order mismatch");
    std::mt19937 rng(28);
    Joints goal{};
    bool found = false;
    int ik_attempt = 0;
    for (int i = 0; i < 300 && !found; ++i)
    {
      std::vector<double> seed(f.left.begin(), f.left.end());
      if (i)
        for (std::size_t j = 0; j < 7; ++j)
        {
          const auto b = model->getVariableBounds(ln[j]);
          std::uniform_real_distribution<double> uniform(b.min_position_, b.max_position_);
          seed[j] = uniform(rng);
        }
      std::vector<double> solution;
      moveit_msgs::msg::MoveItErrorCodes error;
      if (!ik.getPositionIK(target, seed, solution, error) || solution.size() != 7)
        continue;
      std::copy(solution.begin(), solution.end(), goal.begin());
      found = validState(*scene, model, ln, rn, goal, f.right);
      ik_attempt = i + 1;
    }
    require(found, "no collision-free pre-contact IK solution");
    moveit::core::RobotState high_state(model);
    high_state.setToDefaultValues();
    high_state.setVariablePositions(ln, std::vector<double>(goal.begin(), goal.end()));
    high_state.setVariablePositions(rn, std::vector<double>(f.right.begin(), f.right.end()));
    high_state.update();
    const auto high_tcp = high_state.getGlobalLinkTransform("left_fr3_side_suction_tcp");
    require((high_tcp.translation() - target_p).norm() < .0001 &&
                Eigen::Quaterniond(high_tcp.rotation()).angularDistance(target_q) < .002,
            "high approach target mismatch");

    auto space = std::make_shared<ob::RealVectorStateSpace>(7);
    ob::RealVectorBounds bounds(7);
    for (std::size_t i = 0; i < 7; ++i)
    {
      const auto b = model->getVariableBounds(ln[i]);
      bounds.setLow(i, b.min_position_);
      bounds.setHigh(i, b.max_position_);
    }
    space->setBounds(bounds);
    og::SimpleSetup setup(space);
    setup.setStateValidityChecker(
        [&](const ob::State *state)
        {
          Joints q;
          const auto *data = state->as<ob::RealVectorStateSpace::StateType>();
          std::copy(data->values, data->values + 7, q.begin());
          return validState(*scene, model, ln, rn, q, f.right);
        });
    setup.getSpaceInformation()->setStateValidityCheckingResolution(.0002);
    ob::ScopedState<> start(space), end(space);
    for (std::size_t i = 0; i < 7; ++i)
    {
      start[i] = f.left[i];
      end[i] = goal[i];
    }
    setup.setStartAndGoalStates(start, end);
    setup.setPlanner(std::make_shared<og::RRTConnect>(setup.getSpaceInformation()));
    const auto solved = setup.solve(10.0);
    require(solved == ob::PlannerStatus::EXACT_SOLUTION, "no exact pre-contact trajectory");
    auto geometric = setup.getSolutionPath();
    double path_length = geometric.length();
    geometric.interpolate(
        std::max<std::size_t>(2, static_cast<std::size_t>(std::ceil(path_length / kMaxJointStep)) + 1));
    std::vector<Joints> path;
    for (const auto *state : geometric.getStates())
    {
      Joints q;
      const auto *data = state->as<ob::RealVectorStateSpace::StateType>();
      std::copy(data->values, data->values + 7, q.begin());
      path.push_back(q);
    }
    const std::size_t approach_samples = path.size();
    Joints previous = goal;
    double max_alignment_deviation = 0.0;
    for (int segment = 1; segment <= 40; ++segment)
    {
      auto alignment_target = target;
      alignment_target.position.z -= .2 * segment / 40.0;
      std::vector<double> solution;
      moveit_msgs::msg::MoveItErrorCodes error;
      require(ik.getPositionIK(alignment_target, std::vector<double>(previous.begin(), previous.end()),
                               solution, error) &&
                  solution.size() == 7,
              "pre-contact alignment IK failed");
      Joints next;
      std::copy(solution.begin(), solution.end(), next.begin());
      double max_delta = 0;
      for (std::size_t j = 0; j < 7; ++j)
        max_delta = std::max(max_delta, std::abs(next[j] - previous[j]));
      const int count = std::max(1, static_cast<int>(std::ceil(max_delta / kMaxJointStep)));
      for (int i = 1; i <= count; ++i)
      {
        Joints q;
        double norm2 = 0;
        for (std::size_t j = 0; j < 7; ++j)
        {
          q[j] = previous[j] + (next[j] - previous[j]) * i / count;
          const double delta = q[j] - path.back()[j];
          norm2 += delta * delta;
        }
        require(validState(*scene, model, ln, rn, q, f.right), "alignment path collision");
        moveit::core::RobotState state(model);
        state.setToDefaultValues();
        state.setVariablePositions(ln, std::vector<double>(q.begin(), q.end()));
        state.setVariablePositions(rn, std::vector<double>(f.right.begin(), f.right.end()));
        state.update();
        const auto tip = state.getGlobalLinkTransform("left_fr3_side_suction_tcp");
        const double lateral =
            std::hypot(tip.translation().x() - contact_p.x(), tip.translation().y() - contact_p.y());
        max_alignment_deviation = std::max(max_alignment_deviation, lateral);
        const double segment_upper_z = target_p.z() - .2 * (segment - 1) / 40.0;
        require(lateral < .001 && tip.translation().z() <= segment_upper_z + .001 &&
                    tip.translation().z() >= alignment_target.position.z - .001 &&
                    Eigen::Quaterniond(tip.rotation()).angularDistance(target_q) < .005,
                "pre-contact alignment departs from the target line or orientation");
        if (i == count)
          require(std::abs(tip.translation().z() - alignment_target.position.z) < .0001,
                  "pre-contact alignment segment endpoint mismatch");
        path_length += std::sqrt(norm2);
        path.push_back(q);
      }
      previous = next;
    }
    goal = previous;
    moveit::core::RobotState goal_state(model);
    goal_state.setToDefaultValues();
    goal_state.setVariablePositions(ln, std::vector<double>(goal.begin(), goal.end()));
    goal_state.setVariablePositions(rn, std::vector<double>(f.right.begin(), f.right.end()));
    goal_state.update();
    const auto tcp = goal_state.getGlobalLinkTransform("left_fr3_side_suction_tcp");
    const double position_error = (tcp.translation() - contact_p).norm();
    const double angle_error = Eigen::Quaterniond(tcp.rotation()).angularDistance(target_q);
    const double actual_gap = -(f.orientation.inverse() * (tcp.translation() - f.cube)).y() - kHalf;
    require(position_error < .0001 && angle_error < .002 && actual_gap > 0.0, "pre-contact target mismatch");

    // Reproduce the old 4 mm overtravel from a 1 mm gap at the nominal Cube pose.
    auto penetrating_target = target;
    const Eigen::Vector3d penetrating_p = f.cube + f.orientation * Eigen::Vector3d(0, -(kHalf - .003), 0);
    penetrating_target.position.x = penetrating_p.x();
    penetrating_target.position.y = penetrating_p.y();
    penetrating_target.position.z = penetrating_p.z();
    std::vector<double> penetrating_solution;
    moveit_msgs::msg::MoveItErrorCodes penetrating_error;
    require(ik.getPositionIK(penetrating_target, std::vector<double>(goal.begin(), goal.end()),
                             penetrating_solution, penetrating_error) &&
                penetrating_solution.size() == 7,
            "penetrating endpoint negative-control IK failed");
    Joints penetrating;
    std::copy(penetrating_solution.begin(), penetrating_solution.end(), penetrating.begin());
    moveit::core::RobotState penetrating_state(model);
    penetrating_state.setToDefaultValues();
    penetrating_state.setVariablePositions(ln, penetrating_solution);
    penetrating_state.setVariablePositions(rn, std::vector<double>(f.right.begin(), f.right.end()));
    penetrating_state.update();
    const auto penetrating_tcp = penetrating_state.getGlobalLinkTransform("left_fr3_side_suction_tcp");
    require((penetrating_tcp.translation() - penetrating_p).norm() < .0001 &&
                Eigen::Quaterniond(penetrating_tcp.rotation()).angularDistance(target_q) < .002,
            "penetrating endpoint FK mismatch");
    collision_detection::CollisionRequest negative_request;
    negative_request.contacts = true;
    negative_request.max_contacts = 20;
    collision_detection::CollisionResult negative_result;
    scene->checkCollision(negative_request, negative_result, penetrating_state);
    bool tool_cube_collision = false;
    double max_penetration = 0.0;
    for (const auto &[pair, contacts] : negative_result.contacts)
      if ((pair.first == "left_fr3_side_suction" && pair.second == "task26_cube_1") ||
          (pair.second == "left_fr3_side_suction" && pair.first == "task26_cube_1"))
      {
        tool_cube_collision = !contacts.empty();
        for (const auto &contact : contacts)
          max_penetration = std::max(max_penetration, contact.depth);
      }
    require(tool_cube_collision && penetrating_state.satisfiesBounds(model->getJointModelGroup("left_arm")),
            "penetrating endpoint not rejected by original tool/Cube geometry");

    double actual_max_joint_step = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i)
      for (std::size_t j = 0; j < 7; ++j)
        actual_max_joint_step = std::max(actual_max_joint_step, std::abs(path[i][j] - path[i - 1][j]));
    std::size_t checked = 0;
    require(validPath(*scene, model, ln, rn, path, f.right, &checked) && checked == path.size(),
            "full path check failed");
    require(!validPath(*incomplete_scene, model, ln, rn, path, f.right, nullptr),
            "missing Cube path accepted");
    auto invalid_path = path;
    invalid_path[invalid_path.size() / 2][0] = std::numeric_limits<double>::quiet_NaN();
    require(!validPath(*scene, model, ln, rn, invalid_path, f.right, nullptr),
            "invalid intermediate path state accepted");
    // Both ends are the valid goal; a collision inserted between them must still fail the path check.
    std::vector<Joints> collision_detour = path;
    double max_delta = 0;
    for (std::size_t i = 0; i < 7; ++i)
      max_delta = std::max(max_delta, std::abs(penetrating[i] - goal[i]));
    const int subdivisions = std::max(1, static_cast<int>(std::ceil(max_delta / kMaxJointStep)));
    for (int i = 1; i <= subdivisions; ++i)
    {
      Joints q;
      for (std::size_t j = 0; j < 7; ++j)
        q[j] = goal[j] + (penetrating[j] - goal[j]) * i / subdivisions;
      collision_detour.push_back(q);
    }
    for (int i = subdivisions - 1; i >= 0; --i)
    {
      Joints q;
      for (std::size_t j = 0; j < 7; ++j)
        q[j] = goal[j] + (penetrating[j] - goal[j]) * i / subdivisions;
      collision_detour.push_back(q);
    }
    std::size_t negative_checked = 0;
    require(!validPath(*scene, model, ln, rn, collision_detour, f.right, &negative_checked) &&
                negative_checked >= path.size() &&
                validState(*scene, model, ln, rn, collision_detour.back(), f.right),
            "intermediate collision with clear path endpoints was not rejected");
    std::FILE *output = std::fopen(argv[4], "wx");
    require(output != nullptr, "cannot create new trajectory file");
    bool output_ok = std::fprintf(output, "sample,phase,left_fr3_joint1,left_fr3_joint2,left_fr3_joint3,left_"
                                          "fr3_joint4,left_fr3_joint5,left_fr3_joint6,left_fr3_joint7\n") > 0;
    for (std::size_t i = 0; i < path.size(); ++i)
    {
      output_ok =
          output_ok && std::fprintf(output, "%zu,%s", i, i < approach_samples ? "APPROACH" : "ALIGN") > 0;
      for (double q : path[i])
        output_ok = output_ok && std::fprintf(output, ",%.17g", q) > 0;
      output_ok = output_ok && std::fprintf(output, "\n") > 0;
    }
    output_ok = std::fclose(output) == 0 && output_ok;
    require(output_ok, "trajectory write failed");
    std::cout << std::setprecision(10) << "PRECONTACT_PASS samples=" << checked
              << " max_joint_step_rad=" << kMaxJointStep
              << " actual_max_joint_step_rad=" << actual_max_joint_step << " ik_attempt=" << ik_attempt
              << " gap_mm=" << 1000 * actual_gap << " position_error_mm=" << 1000 * position_error
              << " orientation_error_rad=" << angle_error << " path_length_rad=" << path_length
              << " approach_samples=" << approach_samples
              << " align_samples=" << path.size() - approach_samples
              << " align_max_lateral_mm=" << 1000 * max_alignment_deviation << "\n";
    std::cout << "NEGATIVE_CONTACT max_penetration_mm=" << 1000 * max_penetration
              << " detour_reject_index=" << negative_checked << " detour_samples=" << collision_detour.size()
              << "\n";
    std::cout << "NEGATIVE_CONTROLS_PASS penetrating_endpoint intermediate_collision missing_cube "
                 "missing_joint bad_frame invalid_path missing_force unvalidated_force joint_torque "
                 "model_only force_step legacy_b0_guard\n";
    rclcpp::shutdown();
    return 0;
  }
  catch (const std::exception &e)
  {
    std::cerr << "PRECONTACT_REJECT: " << e.what() << '\n';
    if (rclcpp::ok())
      rclcpp::shutdown();
    return 1;
  }
}
