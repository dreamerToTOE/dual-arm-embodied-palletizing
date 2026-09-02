// Task 03：B-A-C 放置可执行性测试
//
// 设计原则：不复制、不改动 Task 02 已验证的 Placement Skill 主体。
// 本节点先把 BoxB / BoxC 注入 MoveIt Planning Scene，随后直接复用
// placement_skill_demo.cpp 的完整抓取、C_reach、C_insert、C_release、C_retreat 流程。

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace std::chrono_literals;

// 让被 include 的 Task 02 源码中的 main() 改名，避免和本文件 main() 冲突。
#define main placement_skill_demo_main
#include "placement_skill_demo.cpp"
#undef main

namespace
{
constexpr double BOX_SIZE = 0.030;

// A 的目标位置保持 Task 02 默认值：
// A center = (0.65, -0.15, 0.065)
// B/C 沿世界 Y 方向分布在 A 两侧。
constexpr double BOX_B_X = 0.65;
constexpr double BOX_B_Y = -0.115;
constexpr double BOX_B_Z = 0.065;

constexpr double BOX_C_X = 0.65;
constexpr double BOX_C_Y = -0.185;
constexpr double BOX_C_Z = 0.065;

moveit_msgs::msg::CollisionObject makeStaticBox(
    const std::string& id,
    double x,
    double y,
    double z)
{
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = "base";
    object.id = id;

    shape_msgs::msg::SolidPrimitive shape;
    shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    shape.dimensions = {BOX_SIZE, BOX_SIZE, BOX_SIZE};

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    object.primitives.push_back(shape);
    object.primitive_poses.push_back(pose);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;

    return object;
}

bool addBacObstaclesToMoveIt()
{
    auto node = std::make_shared<rclcpp::Node>("bac_scene_setup");
    moveit::planning_interface::PlanningSceneInterface psi;

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.push_back(makeStaticBox("box_b", BOX_B_X, BOX_B_Y, BOX_B_Z));
    objects.push_back(makeStaticBox("box_c", BOX_C_X, BOX_C_Y, BOX_C_Z));

    RCLCPP_INFO(node->get_logger(),
                "Task 03：向 MoveIt Planning Scene 加入 BoxB / BoxC...");

    if (!psi.applyCollisionObjects(objects))
    {
        RCLCPP_ERROR(node->get_logger(),
                     "Task 03：BoxB / BoxC 加入 MoveIt Planning Scene 失败。");
        return false;
    }

    rclcpp::sleep_for(1s);

    const double inner_gap = (BOX_B_Y - 0.5 * BOX_SIZE) -
                             (BOX_C_Y + 0.5 * BOX_SIZE);

    RCLCPP_INFO(node->get_logger(),
                "Task 03 B-A-C 场景已加入 MoveIt：");
    RCLCPP_INFO(node->get_logger(),
                "BoxB center = (%.3f, %.3f, %.3f)",
                BOX_B_X, BOX_B_Y, BOX_B_Z);
    RCLCPP_INFO(node->get_logger(),
                "BoxC center = (%.3f, %.3f, %.3f)",
                BOX_C_X, BOX_C_Y, BOX_C_Z);
    RCLCPP_INFO(node->get_logger(),
                "B/C 内侧净间距 = %.1f mm；A 宽度 = 30.0 mm",
                inner_gap * 1000.0);
    RCLCPP_INFO(node->get_logger(),
                "几何条件：d_BC > w_A，因此 A 本体可以放入；下面验证夹爪是否可执行插入。");

    return true;
}
}  // namespace

int main(int argc, char** argv)
{
    // 第一阶段：只负责把 B/C 写入 MoveIt Planning Scene。
    rclcpp::init(argc, argv);

    const bool scene_ok = addBacObstaclesToMoveIt();

    rclcpp::shutdown();

    if (!scene_ok)
    {
        return 1;
    }

    // 第二阶段：直接运行 Task 02 已验证的 Placement Skill。
    // 第一轮预期：
    // C_reach = PASS
    // C_insert = FAIL
    // RESULT = INSERT_FAILED
    return placement_skill_demo_main(argc, argv);
}
