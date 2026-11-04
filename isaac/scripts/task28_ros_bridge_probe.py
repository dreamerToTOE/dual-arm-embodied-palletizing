#!/usr/bin/env python3
"""Check the real bridge and its ROS2Context node in a clean Kit process."""
import ctypes
import json
import os
import traceback

for name in ('librosidl_runtime_c.so', 'librmw_fastrtps_cpp.so'):
    ctypes.CDLL(name)
print('ROS_ENV', json.dumps({k: os.environ.get(k) for k in
      ('ROS_DISTRO', 'RMW_IMPLEMENTATION', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH')}), flush=True)
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'create_new_stage': True, 'multi_gpu': False, 'fast_shutdown': True})
try:
    import omni.kit.app
    import omni.graph.core as og
    manager = omni.kit.app.get_app().get_extension_manager()
    if not manager.set_extension_enabled_immediate('isaacsim.ros2.bridge', True):
        raise RuntimeError('BRIDGE_ENABLE_FAILED')
    from isaacsim.ros2.bridge import _ros2_bridge
    bridge = _ros2_bridge.acquire_ros2_bridge_interface()
    if not manager.is_extension_enabled('isaacsim.ros2.bridge') or not bridge.get_startup_status():
        raise RuntimeError('BRIDGE_STARTUP_FAILED')
    og.Controller.edit({'graph_path': '/ROSProbe', 'evaluator_name': 'execution'},
                       {og.Controller.Keys.CREATE_NODES: [('Context', 'isaacsim.ros2.bridge.ROS2Context'),
                                                        ('Tick', 'omni.graph.action.OnPlaybackTick'),
                                                        ('Clock', 'isaacsim.ros2.bridge.ROS2PublishClock')],
                        og.Controller.Keys.CONNECT: [('Tick.outputs:tick', 'Clock.inputs:execIn'),
                                                     ('Context.outputs:context', 'Clock.inputs:context')],
                        og.Controller.Keys.SET_VALUES: [('Context.inputs:domain_id', 167),
                                                        ('Clock.inputs:topicName', '/task28_probe_clock')]})
    node = og.Controller.node('/ROSProbe/Context')
    if not node.is_valid():
        raise RuntimeError('ROS2_CONTEXT_NODE_INVALID')
    mapped = sorted({line.split()[-1] for line in open('/proc/self/maps')
                     if '/librosidl_' in line or '/librmw_' in line or '/librcl' in line})
    if not mapped or any('/opt/ros/' in path for path in mapped):
        raise RuntimeError(f'ROS_LIBRARY_PROVENANCE_INVALID:{mapped}')
    print('ROS_MAPPED_LIBRARIES', json.dumps(mapped), flush=True)
    import omni.timeline
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(10):
        app.update()
    context = og.Controller.get(og.Controller.attribute('/ROSProbe/Context.outputs:context'))
    print('ROS_CONTEXT_HANDLE', int(context), flush=True)
    if not context:
        raise RuntimeError('ROS_CONTEXT_NOT_INITIALIZED')
    timeline.stop()
    print('ROS_BRIDGE_PROBE_OK bridge_started=true context_node_valid=true', flush=True)
except Exception:
    traceback.print_exc()
    os._exit(1)
app.close()
