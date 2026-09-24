# Task27：第一层五 Cube（四件边侧基准 + 中心插入）场景入口。
#
# Isaac Sim 4.5 Script Editor：先停止 Timeline，再运行：
# exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
#           "task27_five_cube_center_insert_scene.py").read())
#
# 本入口只选择 Task27 布局；FR3、L 型侧吸工具、桌面、围墙、关节量程对齐和
# ROS ActionGraph 均复用 Task26 的已验证构造，避免出现两套漂移的场景资产。
_SIDE_SUCTION_SCENARIO = "task27"
_task26_scene_path = (
    "/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
    "task26_truck_box_scene.py"
)
try:
    exec(compile(open(_task26_scene_path, encoding="utf-8").read(),
                 _task26_scene_path, "exec"), globals(), globals())
finally:
    globals().pop("_SIDE_SUCTION_SCENARIO", None)
