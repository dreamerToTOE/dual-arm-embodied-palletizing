#!/usr/bin/env python3
# Task24-E-B：纯竖直顶吸阵列的 Isaac 外观对照入口。
#
# 在 Isaac Sim 4.5 Script Editor 中、Timeline 停止时运行：
# exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
#           "task24_vertical_array_suction_scene.py").read())
#
# 与 task24_side_suction_tight_scene.py 使用完全相同的 FR3、桌面、Cube 与墙优先
# 垛型，仅将末端切换为：竖直杆 + 水平 2 x 2 阵列面板 + 垂直向下 Cup。
# 本文件是外观/空间对照，不可与 Task24 侧吸 MoveIt、bridge 或执行器混用。

_BASE_SCENE = (
    "/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
    "task24_side_suction_tight_scene.py"
)

_scene_globals = {
    "__name__": "__main__",
    "__file__": _BASE_SCENE,
    "TASK24_TOOL_VARIANT": "vertical_down_array",
}
exec(
    compile(open(_BASE_SCENE, "rb").read(), _BASE_SCENE, "exec"),
    _scene_globals,
)
