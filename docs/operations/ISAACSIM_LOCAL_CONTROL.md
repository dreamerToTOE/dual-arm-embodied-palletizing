# Isaac Sim 本机控制入口

本项目使用 Isaac Sim 4.5 自带的 `isaacsim.code_editor.vscode` 扩展，
让本机 Codex 通过回环地址执行 Isaac Kit 进程内的 Python。它可以读取 USD
Stage、控制 Timeline、截取 Viewport，以及执行已审查的场景脚本；不会暴露给
局域网或公网。

## 1. 从带 ROS 2 环境的终端启动 Isaac Sim

必须先关闭旧 Isaac Sim GUI，再在一个干净终端执行：

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/home/ubuntu2004/isaacsim-4.5.0/exts/isaacsim.ros2.bridge/humble/lib"
/home/ubuntu2004/isaacsim-4.5.0/isaac-sim.sh
```

这一步保证 `isaacsim.ros2.bridge` 能加载。仅启动 GUI 而未加载上述环境时，
Scene 虽可显示，但 ROS2 Bridge 会启动失败。

## 2. 在 Isaac Script Editor 中执行一次

在 `Window -> Script Editor` 新建 Python 标签页，粘贴并运行以下代码：

```python
import carb
import omni.kit.app

HOST = "127.0.0.1"
PORT = 8226
EXTENSION = "isaacsim.code_editor.vscode"

settings = carb.settings.get_settings()
settings.set("/exts/isaacsim.code_editor.vscode/host", HOST)
settings.set("/exts/isaacsim.code_editor.vscode/port", PORT)

manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate(EXTENSION, True)

print(f"[Project] Isaac local executor ready at {HOST}:{PORT}")
```

每次新开 Isaac Sim GUI 后都要执行一次。这里故意绑定 `127.0.0.1`，避免
控制端口监听到其他机器。

## 3. 终端验证

```bash
ss -ltn | grep ':8226'
```

预期有一行 `127.0.0.1:8226` 的 `LISTEN`。出现后，已安装的
`isaacsim-local-bridge` 可在本机检查 Stage、执行场景脚本、控制 Timeline 和
抓取 Viewport。

## Task24 例子

在启用 executor 后，可照常从 Script Editor 重建 Task24 场景与启动 bridge：

```python
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task24_side_suction_tight_scene.py").read())
exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task24_side_suction_tight_bridge.py").read())
```

先运行 Scene，确认 Stage 已重建后点击 Timeline Play，再运行 Bridge。
Bridge 启动日志中应包含 `/task24/cube_poses` 与左右吸盘、关节命令主题。
