#!/usr/bin/env python3
# Task24-E-C：80 mm 竖直杆 + 2 x 2 顶吸阵列的 Isaac 外观对照入口。
#
# 在 Isaac Sim Script Editor 中、Timeline 停止时运行：
# exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
#           "task24_vertical_array_suction_80mm_scene.py").read())
#
# 此处 80 mm 指 link8 到阵列板中心的固定竖直支撑杆长度；加上阵列板、Cup
# 与 TCP 后，link8 到接触 TCP 的总延伸约为 97 mm。
# 本文件只用于外观和工作空间对照，不能运行 Task24 的侧吸 bridge 或执行器。

_BASE_SCENE = (
    "/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
    "task24_side_suction_tight_scene.py"
)

_scene_globals = {
    "__name__": "__main__",
    "__file__": _BASE_SCENE,
    "TASK24_TOOL_VARIANT": "vertical_down_array",
    "TASK24_VERTICAL_ARRAY_SUPPORT_LENGTH": 0.080,
}
exec(
    compile(open(_BASE_SCENE, "rb").read(), _BASE_SCENE, "exec"),
    _scene_globals,
)
