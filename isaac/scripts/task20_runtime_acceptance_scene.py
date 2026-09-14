# Task20-C/D：可重复的 Isaac 物理执行验收场景。
#
# 在新开的 Isaac Sim 4.5 Script Editor 中、Timeline Stop 时运行：
#
#   exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
#             "task20_runtime_acceptance_scene.py").read())
#
# 本入口不要求此前已打开 Task06 或 Task18。它复用 Task18 的自包含场景构造：
# Task18 会在空 Stage 上创建双 FR3、compact suction、Table、Physics Scene 与
# Task06 ROS ActionGraph。这里固定一个已通过 MoveIt/SharedBox private-FCL 回归的
# episode seed，只用于第一次真实执行验收；不改变通用 Task18 随机生成器。

import builtins
import os


PROJECT_ROOT = "/home/ubuntu2004/lmy/dual-arm-embodied-palletizing"
TASK18_SCENE_SCRIPT = f"{PROJECT_ROOT}/isaac/scripts/task18_randomized_scene.py"
ACCEPTANCE_SEED = "20260922"


def _shutdown_old_runtime_bridges():
    """场景重建前先停掉只会引用旧 Prim 的 Task18/Task20 bridge。"""
    for name in (
        "_task20_runtime_suction_bridge",
        "_task18_ground_truth_bridge",
    ):
        bridge = getattr(builtins, name, None)
        if bridge is None:
            continue
        try:
            bridge.shutdown()
        except Exception as exc:
            print(f"Task20-C 清理旧 bridge {name} 时出现非致命异常：{exc}")


def _run_task18_scene_with_acceptance_seed():
    try:
        with open(TASK18_SCENE_SCRIPT, "r", encoding="utf-8") as source_file:
            source = source_file.read()
    except OSError as exc:
        raise RuntimeError(f"无法读取 Task18 场景脚本：{TASK18_SCENE_SCRIPT}") from exc

    # 只在执行 Task18 场景构造的期间覆盖环境变量。USD metadata 会记录实际 seed，
    # 之后 Ground Truth bridge 不依赖该环境变量，更不会使用固定源坐标。
    previous_seed = os.environ.get("TASK18_SEED")
    os.environ["TASK18_SEED"] = ACCEPTANCE_SEED
    try:
        exec(compile(source, TASK18_SCENE_SCRIPT, "exec"), {
            "__name__": "__task20_runtime_acceptance_scene__",
        })
    finally:
        if previous_seed is None:
            os.environ.pop("TASK18_SEED", None)
        else:
            os.environ["TASK18_SEED"] = previous_seed


_shutdown_old_runtime_bridges()
_run_task18_scene_with_acceptance_seed()

print("")
print("====================================================")
print("Task20-C/D Isaac runtime acceptance scene ready")
print(f"fixed replay seed = {ACCEPTANCE_SEED}")
print("Runtime execution policy: Task20-C executes largest Box only (max_tasks=1)")
print("                          Task20-D executes 3 Boxes with settle/replan (max_tasks=3)")
print("TIGHT target window: x=[0.535, 0.765], y=[-0.165, 0.165]")
print("Next: Timeline Play -> task18_ground_truth_bridge.py")
print("      -> task20_runtime_suction_bridge.py")
print("      -> ros2 launch fr3_dual_palletize task20_runtime_acceptance.launch.py")
print("         or task20_continuous_runtime_acceptance.launch.py")
print("====================================================")
