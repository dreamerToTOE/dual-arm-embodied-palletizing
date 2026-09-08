#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>

using namespace std::chrono_literals;

namespace
{
constexpr std::size_t NUM_BOXES = 3;

constexpr double TABLE_X = 0.55;
constexpr double TABLE_Y = 0.00;
constexpr double TABLE_Z = 0.025;
constexpr double TABLE_SIZE_X = 1.20;
constexpr double TABLE_SIZE_Y = 0.80;
constexpr double TABLE_SIZE_Z = 0.05;
constexpr double TABLE_TOP_Z = TABLE_Z + 0.5 * TABLE_SIZE_Z;

constexpr double BOX_SIZE = 0.030;
constexpr double BOX_HALF = 0.5 * BOX_SIZE;

// Task03 B-A-C 几何：同一目标空位，用顶部吸盘验证二指夹爪的侧向干涉是否被消除。
constexpr double TARGET_X = 0.650;
constexpr double TARGET_Y = -0.150;
constexpr double TARGET_CENTER_Z = 0.065;

// Franka 官方 cobot_pump：TCP 相对 fr3_link8 沿局部 +Z 约 0.105 m。
constexpr double COBOT_PUMP_TCP_OFFSET_Z = 0.105;
constexpr double CONTACT_TCP_CLEARANCE = 0.001;
constexpr double PLACEMENT_RELEASE_GAP = 0.001;

// 沿用 Task04 已验证高度关系。
constexpr double PRE_PICK_CLEARANCE = 0.064;
constexpr double LIFT_CLEARANCE = 0.099;
constexpr double PRE_PLACE_TCP_Z = 0.180;
constexpr double RETREAT_TCP_Z = 0.180;

constexpr double CARTESIAN_EEF_STEP = 0.002;
constexpr double CARTESIAN_MIN_FRACTION = 0.999;

bool copyRobotModelParameters(const rclcpp::Node::SharedPtr& node)
{
    RCLCPP_INFO(node->get_logger(), "等待 /move_group 参数服务...");

    auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
    if (!client->wait_for_service(10s))
    {
        RCLCPP_ERROR(node->get_logger(), "无法连接 /move_group，请先启动 MoveIt。");
        return false;
    }

    const auto params = client->get_parameters({
        "robot_description",
        "robot_description_semantic"
    });

    if (params.size() != 2 ||
        params[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
        params[1].get_type() != rclcpp::ParameterType::PARAMETER_STRING)
    {
        RCLCPP_ERROR(node->get_logger(), "robot_description / semantic 参数无效。");
        return false;
    }

    node->declare_parameter<std::string>("robot_description", params[0].as_string());
    node->declare_parameter<std::string>("robot_description_semantic", params[1].as_string());
    return true;
}

double pointTime(const trajectory_msgs::msg::JointTrajectoryPoint& point)
{
    return static_cast<double>(point.time_from_start.sec) +
           static_cast<double>(point.time_from_start.nanosec) * 1e-9;
}

void setPointTime(trajectory_msgs::msg::JointTrajectoryPoint& point, double seconds)
{
    int32_t sec = static_cast<int32_t>(std::floor(seconds));
    int64_t nsec = static_cast<int64_t>(
        std::llround((seconds - static_cast<double>(sec)) * 1e9));

    if (nsec >= 1000000000LL)
    {
        ++sec;
        nsec -= 1000000000LL;
    }

    point.time_from_start.sec = sec;
    point.time_from_start.nanosec = static_cast<uint32_t>(std::max<int64_t>(0, nsec));
}

void ensureTrajectoryTiming(trajectory_msgs::msg::JointTrajectory& trajectory)
{
    if (trajectory.points.empty() || pointTime(trajectory.points.back()) > 1e-6)
    {
        return;
    }

    constexpr double FALLBACK_DT = 0.03;
    for (std::size_t i = 0; i < trajectory.points.size(); ++i)
    {
        setPointTime(trajectory.points[i], static_cast<double>(i) * FALLBACK_DT);
    }
}

geometry_msgs::msg::Pose makeTopDownPose(
    double x,
    double y,
    double suction_tcp_world_z,
    double yaw = 0.0)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;

    // Rz(yaw) * Rx(pi) 时，fr3_link8 local +Z 指向 world -Z。
    // 因此 link8 目标需要比 suction TCP 高 0.105 m。
    pose.position.z = suction_tcp_world_z + COBOT_PUMP_TCP_OFFSET_Z;

    const double half_yaw = 0.5 * yaw;
    pose.orientation.x = std::cos(half_yaw);
    pose.orientation.y = std::sin(half_yaw);
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;
    return pose;
}

}  // namespace

class Task05BACSuctionPlace
{
public:
    explicit Task05BACSuctionPlace(const rclcpp::Node::SharedPtr& node)
        : node_(node)
    {
        command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_command", 10);
        suction_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
            "/task05/suction_command", 10);

        box_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
            "/task05/box_poses",
            10,
            [this](const geometry_msgs::msg::PoseArray::SharedPtr msg)
            {
                if (msg->poses.size() < NUM_BOXES)
                {
                    return;
                }

                std::lock_guard<std::mutex> lock(box_mutex_);
                for (std::size_t i = 0; i < NUM_BOXES; ++i)
                {
                    box_poses_[i] = msg->poses[i];
                }
                have_box_poses_ = true;
            });

        suction_state_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/task05/suction_state",
            10,
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                suction_closed_.store(msg->data);
            });
    }

    bool run()
    {
        RCLCPP_INFO(node_->get_logger(),
                    "========== Task05：B-A-C 顶部吸盘高密度放置 ==========");

        if (!waitForIsaacBridge())
        {
            return false;
        }

        moveit::planning_interface::MoveGroupInterface move_group(node_, "fr3_arm");
        move_group.setPlannerId("RRTConnectkConfigDefault");
        move_group.setPlanningTime(5.0);
        move_group.setNumPlanningAttempts(5);
        move_group.setMaxVelocityScalingFactor(0.20);
        move_group.setMaxAccelerationScalingFactor(0.20);
        move_group.setPoseReferenceFrame("base");

        const std::string eef_link =
            node_->declare_parameter<std::string>("eef_link", "fr3_link8");
        if (move_group.getRobotModel()->getLinkModel(eef_link) == nullptr)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "指定 eef_link=%s 不存在。", eef_link.c_str());
            return false;
        }
        move_group.setEndEffectorLink(eef_link);

        const auto* joint_model_group =
            move_group.getRobotModel()->getJointModelGroup("fr3_arm");
        if (joint_model_group == nullptr)
        {
            RCLCPP_ERROR(node_->get_logger(), "找不到 fr3_arm JointModelGroup。");
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "MoveIt planning tip = %s", eef_link.c_str());
        RCLCPP_INFO(node_->get_logger(), "Suction TCP offset Z = %.3f m",
                    COBOT_PUMP_TCP_OFFSET_Z);

        std::array<geometry_msgs::msg::Pose, NUM_BOXES> initial_boxes;
        {
            std::lock_guard<std::mutex> lock(box_mutex_);
            initial_boxes = box_poses_;
        }

        if (!addInitialPlanningScene(initial_boxes))
        {
            return false;
        }

        const auto& box_a_pick = initial_boxes[0];
        const auto& box_b = initial_boxes[1];
        const auto& box_c = initial_boxes[2];

        RCLCPP_INFO(node_->get_logger(),
                    "BoxA PICK = (%.4f, %.4f, %.4f)",
                    box_a_pick.position.x,
                    box_a_pick.position.y,
                    box_a_pick.position.z);
        RCLCPP_INFO(node_->get_logger(),
                    "BoxA TARGET = (%.4f, %.4f, %.4f)",
                    TARGET_X, TARGET_Y, TARGET_CENTER_Z);
        RCLCPP_INFO(node_->get_logger(),
                    "BoxB = (%.4f, %.4f, %.4f), BoxC = (%.4f, %.4f, %.4f)",
                    box_b.position.x, box_b.position.y, box_b.position.z,
                    box_c.position.x, box_c.position.y, box_c.position.z);

        const std::vector<double> q_home = {
            0.0, -0.7, 0.0, -2.2, 0.0, 2.0, 0.8
        };
        std::vector<double> current_q = q_home;

        // 保证下降到 CONTACT 之前吸盘关闭，避免提前吸附/浮空抓。
        commandSuction(false);
        std::this_thread::sleep_for(100ms);

        if (!pickPlaceA(move_group,
                        joint_model_group,
                        eef_link,
                        box_a_pick,
                        current_q))
        {
            commandSuction(false);
            return false;
        }

        trajectory_msgs::msg::JointTrajectory home_traj;
        if (!planJointStage(move_group,
                            joint_model_group,
                            current_q,
                            q_home,
                            "RETREAT -> HOME",
                            home_traj))
        {
            return false;
        }
        executeTrajectory(home_traj);

        const auto settled = getBoxPose(0);
        const double ex = settled.position.x - TARGET_X;
        const double ey = settled.position.y - TARGET_Y;
        const double ez = settled.position.z - TARGET_CENTER_Z;
        const double exy = std::hypot(ex, ey);

        RCLCPP_INFO(node_->get_logger(),
                    "Task05 最终 BoxA = (%.4f, %.4f, %.4f)",
                    settled.position.x,
                    settled.position.y,
                    settled.position.z);
        RCLCPP_INFO(node_->get_logger(),
                    "Task05 放置误差：e_xy=%.2f mm, e_z=%.2f mm（本 Task 只记录，不自动纠偏）",
                    exy * 1000.0,
                    std::abs(ez) * 1000.0);
        RCLCPP_INFO(node_->get_logger(),
                    "========== Task05 SUCCESS：顶部吸盘完成 B-A-C 放置 ==========");
        return true;
    }

private:
    geometry_msgs::msg::Pose getBoxPose(std::size_t index)
    {
        std::lock_guard<std::mutex> lock(box_mutex_);
        return box_poses_[index];
    }

    bool waitForIsaacBridge()
    {
        RCLCPP_INFO(node_->get_logger(),
                    "等待 Isaac Task05 bridge：box_poses + suction_state...");

        for (int i = 0; i < 100; ++i)
        {
            bool have_poses = false;
            {
                std::lock_guard<std::mutex> lock(box_mutex_);
                have_poses = have_box_poses_;
            }

            if (have_poses &&
                command_pub_->get_subscription_count() > 0 &&
                suction_pub_->get_subscription_count() > 0)
            {
                RCLCPP_INFO(node_->get_logger(), "Isaac Task05 bridge 已连接。");
                return true;
            }
            std::this_thread::sleep_for(100ms);
        }

        RCLCPP_ERROR(node_->get_logger(),
                     "等待 Task05 bridge 超时。请确认 Isaac 已 Play 且 task05_suction_ros_bridge.py 已运行。");
        return false;
    }

    bool pickPlaceA(
        moveit::planning_interface::MoveGroupInterface& move_group,
        const moveit::core::JointModelGroup* joint_model_group,
        const std::string& eef_link,
        const geometry_msgs::msg::Pose& pick_pose,
        std::vector<double>& current_q)
    {
        const double pick_contact_tcp_z =
            pick_pose.position.z + BOX_HALF + CONTACT_TCP_CLEARANCE;
        const double pre_pick_tcp_z = pick_contact_tcp_z + PRE_PICK_CLEARANCE;
        const double lift_tcp_z = pick_contact_tcp_z + LIFT_CLEARANCE;

        const double planned_release_center_z =
            TABLE_TOP_Z + BOX_HALF + PLACEMENT_RELEASE_GAP;
        const double place_tcp_z =
            planned_release_center_z + BOX_HALF + CONTACT_TCP_CLEARANCE;

        RCLCPP_INFO(node_->get_logger(),
                    "抓取高度：CONTACT TCP Z=%.4f, PRE_PICK TCP Z=%.4f, LIFT TCP Z=%.4f",
                    pick_contact_tcp_z, pre_pick_tcp_z, lift_tcp_z);
        RCLCPP_INFO(node_->get_logger(),
                    "放置高度：planned center Z=%.4f, PLACE TCP Z=%.4f",
                    planned_release_center_z, place_tcp_z);

        // 1. HOME -> PRE_PICK
        auto pre_pick = makeTopDownPose(
            pick_pose.position.x,
            pick_pose.position.y,
            pre_pick_tcp_z);
        trajectory_msgs::msg::JointTrajectory pre_pick_traj;
        if (!planPoseStage(move_group,
                           joint_model_group,
                           eef_link,
                           current_q,
                           pre_pick,
                           "PRE_PICK",
                           pre_pick_traj))
        {
            return false;
        }
        executeTrajectory(pre_pick_traj);

        // 2. 吸盘保持 OFF，垂直到 CONTACT。
        auto contact = makeTopDownPose(
            pick_pose.position.x,
            pick_pose.position.y,
            pick_contact_tcp_z);
        trajectory_msgs::msg::JointTrajectory contact_traj;
        if (!planCartesianStage(move_group,
                                joint_model_group,
                                currentFrom(pre_pick_traj),
                                contact,
                                "PRE_PICK -> SUCTION_CONTACT",
                                contact_traj))
        {
            return false;
        }
        executeTrajectory(contact_traj);

        // 3. 到达 CONTACT 后才打开吸盘。
        commandSuction(true);
        if (!waitForSuctionClosed(true, 2.0))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "BoxA：CONTACT 到位后 Surface Gripper 未吸住。");
            return false;
        }

        removeWorldBox("box_a");
        if (!attachBoxAToTcp("box_a", eef_link))
        {
            return false;
        }

        // 4. LIFT
        auto lift = makeTopDownPose(
            pick_pose.position.x,
            pick_pose.position.y,
            lift_tcp_z);
        trajectory_msgs::msg::JointTrajectory lift_traj;
        if (!planCartesianStage(move_group,
                                joint_model_group,
                                currentFrom(contact_traj),
                                lift,
                                "SUCTION_CONTACT -> LIFT",
                                lift_traj))
        {
            return false;
        }
        executeTrajectory(lift_traj);

        // 5. 携物到 B/C 上方 PRE_PLACE。
        auto pre_place = makeTopDownPose(TARGET_X, TARGET_Y, PRE_PLACE_TCP_Z);
        trajectory_msgs::msg::JointTrajectory pre_place_traj;
        if (!planPoseStage(move_group,
                           joint_model_group,
                           eef_link,
                           currentFrom(lift_traj),
                           pre_place,
                           "LIFT -> PRE_PLACE",
                           pre_place_traj))
        {
            return false;
        }
        executeTrajectory(pre_place_traj);

        // 6. 垂直进入 B/C 中间。
        auto place = makeTopDownPose(TARGET_X, TARGET_Y, place_tcp_z);
        trajectory_msgs::msg::JointTrajectory place_traj;
        if (!planCartesianStage(move_group,
                                joint_model_group,
                                currentFrom(pre_place_traj),
                                place,
                                "PRE_PLACE -> PLACE (B-A-C INSERT)",
                                place_traj))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Task05 C_insert FAIL：顶部吸盘无法完成 B/C 中间垂直插入。");
            return false;
        }
        executeTrajectory(place_traj);

        // 7. 仍 Attached + SUCTION ON 时预先规划 RETREAT。
        //    这是 Task04 已验证的正确时序：避免 BoxA 落稳后再加入 World 时，
        //    末端紧邻 BoxA 导致 Cartesian 起点 fraction=0。
        auto retreat = makeTopDownPose(TARGET_X, TARGET_Y, RETREAT_TCP_Z);
        trajectory_msgs::msg::JointTrajectory retreat_traj;
        if (!planCartesianStage(move_group,
                                joint_model_group,
                                currentFrom(place_traj),
                                retreat,
                                "PLACE -> RETREAT (PLAN BEFORE RELEASE)",
                                retreat_traj))
        {
            return false;
        }

        // 8. 物理释放并等待 BoxA 落稳。
        commandSuction(false);
        if (!waitForSuctionClosed(false, 2.0))
        {
            RCLCPP_ERROR(node_->get_logger(), "BoxA：吸盘释放状态确认失败。");
            return false;
        }

        std::this_thread::sleep_for(300ms);
        const auto settled_pose = getBoxPose(0);
        RCLCPP_INFO(node_->get_logger(),
                    "BoxA Isaac settle pose = (%.4f, %.4f, %.4f)",
                    settled_pose.position.x,
                    settled_pose.position.y,
                    settled_pose.position.z);

        // 9. MoveIt 与 Isaac 物理状态同步。
        if (!detachBoxFromTcp("box_a", eef_link))
        {
            return false;
        }
        removeWorldBox("box_a");
        if (!addWorldBox("box_a", settled_pose))
        {
            return false;
        }

        // 10. 执行释放前已规划好的安全 RETREAT。
        executeTrajectory(retreat_traj);
        current_q = currentFrom(retreat_traj);

        RCLCPP_INFO(node_->get_logger(),
                    "Task05 C_insert = PASS，BoxA 已释放并完成垂直 RETREAT。");
        return true;
    }

    static std::vector<double> currentFrom(
        const trajectory_msgs::msg::JointTrajectory& trajectory)
    {
        if (trajectory.points.empty())
        {
            return {};
        }
        return trajectory.points.back().positions;
    }

    bool planPoseStage(
        moveit::planning_interface::MoveGroupInterface& move_group,
        const moveit::core::JointModelGroup* joint_model_group,
        const std::string& eef_link,
        const std::vector<double>& start_q,
        const geometry_msgs::msg::Pose& target_pose,
        const std::string& stage_name,
        trajectory_msgs::msg::JointTrajectory& trajectory_out)
    {
        moveit::core::RobotState start_state(move_group.getRobotModel());
        start_state.setToDefaultValues();
        start_state.setJointGroupPositions(joint_model_group, start_q);
        start_state.update();

        move_group.setStartState(start_state);
        move_group.clearPoseTargets();
        if (!move_group.setPoseTarget(target_pose, eef_link))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "%s：设置 PoseTarget 失败。", stage_name.c_str());
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "%s：RRTConnect 规划失败。", stage_name.c_str());
            return false;
        }

        trajectory_out = plan.trajectory_.joint_trajectory;
        if (trajectory_out.points.empty())
        {
            return false;
        }

        RCLCPP_INFO(node_->get_logger(),
                    "%s：规划成功，waypoints=%zu，duration=%.3f s",
                    stage_name.c_str(),
                    trajectory_out.points.size(),
                    pointTime(trajectory_out.points.back()));
        return true;
    }

    bool planJointStage(
        moveit::planning_interface::MoveGroupInterface& move_group,
        const moveit::core::JointModelGroup* joint_model_group,
        const std::vector<double>& start_q,
        const std::vector<double>& target_q,
        const std::string& stage_name,
        trajectory_msgs::msg::JointTrajectory& trajectory_out)
    {
        moveit::core::RobotState start_state(move_group.getRobotModel());
        start_state.setToDefaultValues();
        start_state.setJointGroupPositions(joint_model_group, start_q);
        start_state.update();

        move_group.setStartState(start_state);
        move_group.clearPoseTargets();
        if (!move_group.setJointValueTarget(target_q))
        {
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "%s：规划失败。", stage_name.c_str());
            return false;
        }

        trajectory_out = plan.trajectory_.joint_trajectory;
        return !trajectory_out.points.empty();
    }

    bool planCartesianStage(
        moveit::planning_interface::MoveGroupInterface& move_group,
        const moveit::core::JointModelGroup* joint_model_group,
        const std::vector<double>& start_q,
        const geometry_msgs::msg::Pose& target_pose,
        const std::string& stage_name,
        trajectory_msgs::msg::JointTrajectory& trajectory_out)
    {
        moveit::core::RobotState start_state(move_group.getRobotModel());
        start_state.setToDefaultValues();
        start_state.setJointGroupPositions(joint_model_group, start_q);
        start_state.update();
        move_group.setStartState(start_state);

        std::vector<geometry_msgs::msg::Pose> waypoints{target_pose};
        moveit_msgs::msg::RobotTrajectory robot_trajectory;
        moveit_msgs::msg::MoveItErrorCodes error_code;

        const double fraction = move_group.computeCartesianPath(
            waypoints,
            CARTESIAN_EEF_STEP,
            0.0,
            robot_trajectory,
            true,
            &error_code);

        RCLCPP_INFO(node_->get_logger(),
                    "%s：Cartesian fraction = %.4f, error_code = %d",
                    stage_name.c_str(), fraction, error_code.val);

        if (fraction < CARTESIAN_MIN_FRACTION)
        {
            return false;
        }

        trajectory_out = robot_trajectory.joint_trajectory;
        if (trajectory_out.points.empty())
        {
            return false;
        }
        ensureTrajectoryTiming(trajectory_out);
        return true;
    }

    moveit_msgs::msg::CollisionObject makeBoxObject(
        const std::string& id,
        const geometry_msgs::msg::Pose& pose,
        int operation)
    {
        moveit_msgs::msg::CollisionObject box;
        box.header.frame_id = "base";
        box.id = id;

        shape_msgs::msg::SolidPrimitive shape;
        shape.type = shape_msgs::msg::SolidPrimitive::BOX;
        shape.dimensions = {BOX_SIZE, BOX_SIZE, BOX_SIZE};

        box.primitives.push_back(shape);
        box.primitive_poses.push_back(pose);
        box.operation = operation;
        return box;
    }

    bool addInitialPlanningScene(
        const std::array<geometry_msgs::msg::Pose, NUM_BOXES>& boxes)
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        std::vector<moveit_msgs::msg::CollisionObject> objects;

        moveit_msgs::msg::CollisionObject table;
        table.header.frame_id = "base";
        table.id = "table";

        shape_msgs::msg::SolidPrimitive table_shape;
        table_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
        table_shape.dimensions = {TABLE_SIZE_X, TABLE_SIZE_Y, TABLE_SIZE_Z};

        geometry_msgs::msg::Pose table_pose;
        table_pose.orientation.w = 1.0;
        table_pose.position.x = TABLE_X;
        table_pose.position.y = TABLE_Y;
        table_pose.position.z = TABLE_Z;

        table.primitives.push_back(table_shape);
        table.primitive_poses.push_back(table_pose);
        table.operation = moveit_msgs::msg::CollisionObject::ADD;
        objects.push_back(table);

        objects.push_back(makeBoxObject(
            "box_a", boxes[0], moveit_msgs::msg::CollisionObject::ADD));
        objects.push_back(makeBoxObject(
            "box_b", boxes[1], moveit_msgs::msg::CollisionObject::ADD));
        objects.push_back(makeBoxObject(
            "box_c", boxes[2], moveit_msgs::msg::CollisionObject::ADD));

        if (!psi.applyCollisionObjects(objects))
        {
            RCLCPP_ERROR(node_->get_logger(), "Task05 初始化 Planning Scene 失败。");
            return false;
        }

        std::this_thread::sleep_for(700ms);
        return true;
    }

    void removeWorldBox(const std::string& box_id)
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        psi.removeCollisionObjects({box_id});
        std::this_thread::sleep_for(300ms);
    }

    bool addWorldBox(
        const std::string& box_id,
        const geometry_msgs::msg::Pose& pose)
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        const auto box = makeBoxObject(
            box_id, pose, moveit_msgs::msg::CollisionObject::ADD);
        const bool ok = psi.applyCollisionObject(box);
        std::this_thread::sleep_for(300ms);
        return ok;
    }

    bool attachBoxAToTcp(
        const std::string& box_id,
        const std::string& eef_link)
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        moveit_msgs::msg::AttachedCollisionObject attached;
        attached.link_name = eef_link;
        attached.touch_links = {eef_link, "fr3_cobot_pump"};
        attached.object.header.frame_id = eef_link;
        attached.object.id = box_id;

        shape_msgs::msg::SolidPrimitive shape;
        shape.type = shape_msgs::msg::SolidPrimitive::BOX;
        shape.dimensions = {BOX_SIZE, BOX_SIZE, BOX_SIZE};

        geometry_msgs::msg::Pose relative_pose;
        relative_pose.position.z =
            COBOT_PUMP_TCP_OFFSET_Z + BOX_HALF + CONTACT_TCP_CLEARANCE;
        relative_pose.orientation.x = 1.0;
        relative_pose.orientation.y = 0.0;
        relative_pose.orientation.z = 0.0;
        relative_pose.orientation.w = 0.0;

        attached.object.primitives.push_back(shape);
        attached.object.primitive_poses.push_back(relative_pose);
        attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;

        const bool ok = psi.applyAttachedCollisionObject(attached);
        std::this_thread::sleep_for(500ms);
        return ok;
    }

    bool detachBoxFromTcp(
        const std::string& box_id,
        const std::string& eef_link)
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        moveit_msgs::msg::AttachedCollisionObject attached;
        attached.link_name = eef_link;
        attached.object.id = box_id;
        attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

        const bool ok = psi.applyAttachedCollisionObject(attached);
        std::this_thread::sleep_for(400ms);
        return ok;
    }

    void commandSuction(bool on)
    {
        std_msgs::msg::Bool msg;
        msg.data = on;
        for (int i = 0; i < 20; ++i)
        {
            suction_pub_->publish(msg);
            std::this_thread::sleep_for(10ms);
        }
        RCLCPP_INFO(node_->get_logger(), "SUCTION %s", on ? "ON" : "OFF");
    }

    bool waitForSuctionClosed(bool expected, double timeout_sec)
    {
        const auto start = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - start).count() < timeout_sec)
        {
            if (suction_closed_.load() == expected)
            {
                RCLCPP_INFO(node_->get_logger(),
                            "Isaac suction_state = %s",
                            expected ? "CLOSED" : "OPEN");
                return true;
            }
            std::this_thread::sleep_for(20ms);
        }
        return false;
    }

    void executeTrajectory(const trajectory_msgs::msg::JointTrajectory& input)
    {
        if (input.points.empty())
        {
            return;
        }

        auto trajectory = input;
        ensureTrajectoryTiming(trajectory);

        const double total_time = pointTime(trajectory.points.back());
        std::size_t segment = 0;
        const auto start_time = std::chrono::steady_clock::now();

        while (true)
        {
            const double t = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start_time).count();
            if (t > total_time)
            {
                break;
            }

            while (segment + 1 < trajectory.points.size() &&
                   pointTime(trajectory.points[segment + 1]) < t)
            {
                ++segment;
            }

            std::vector<double> q_command;
            if (segment + 1 >= trajectory.points.size())
            {
                q_command = trajectory.points.back().positions;
            }
            else
            {
                const auto& p0 = trajectory.points[segment];
                const auto& p1 = trajectory.points[segment + 1];
                const double t0 = pointTime(p0);
                const double t1 = pointTime(p1);
                double alpha = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0;
                alpha = std::clamp(alpha, 0.0, 1.0);

                q_command.resize(p0.positions.size());
                for (std::size_t j = 0; j < q_command.size(); ++j)
                {
                    q_command[j] = p0.positions[j] +
                        alpha * (p1.positions[j] - p0.positions[j]);
                }
            }

            sensor_msgs::msg::JointState msg;
            msg.header.stamp = node_->now();
            msg.name = trajectory.joint_names;
            msg.position = q_command;
            command_pub_->publish(msg);
            std::this_thread::sleep_for(10ms);
        }

        for (int i = 0; i < 50; ++i)
        {
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = node_->now();
            msg.name = trajectory.joint_names;
            msg.position = trajectory.points.back().positions;
            command_pub_->publish(msg);
            std::this_thread::sleep_for(10ms);
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr suction_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr box_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr suction_state_sub_;

    std::mutex box_mutex_;
    std::array<geometry_msgs::msg::Pose, NUM_BOXES> box_poses_{};
    bool have_box_poses_ = false;
    std::atomic_bool suction_closed_{false};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("task05_bac_suction_place");

    if (!copyRobotModelParameters(node))
    {
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    bool success = false;
    try
    {
        Task05BACSuctionPlace demo(node);
        success = demo.run();
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(node->get_logger(), "程序异常：%s", e.what());
    }

    executor.cancel();
    if (spin_thread.joinable())
    {
        spin_thread.join();
    }

    rclcpp::shutdown();
    return success ? 0 : 1;
}
