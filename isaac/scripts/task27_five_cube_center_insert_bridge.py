# Task27：第一层五 Cube 的 Isaac ROS 2 物理桥入口。
#
# 必须先运行 task27_five_cube_center_insert_scene.py、再点击 Play，随后在同一
# Script Editor 执行：
# exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
#           "task27_five_cube_center_insert_bridge.py").read())
#
# Bridge 复用 Task26 已验收的真实 Surface Gripper、共同 command 原子写入、
# Ground Truth、逐件到料与导轨安全互锁；本入口只隔离到 /task27 namespace。
_SIDE_SUCTION_SCENARIO = "task27"
_task26_bridge_path = (
    "/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
    "task26_truck_box_bridge.py"
)
try:
    exec(compile(open(_task26_bridge_path, encoding="utf-8").read(),
                 _task26_bridge_path, "exec"), globals(), globals())
finally:
    globals().pop("_SIDE_SUCTION_SCENARIO", None)
