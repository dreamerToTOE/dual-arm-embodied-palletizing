"""Isolated Task27 scene builder: omit ROS and restore the original material."""
import ast


def diagnostic_scene_code(source, filename):
    tree = ast.parse(source, filename)
    counts = {'bridge': 0, 'graph': 0, 'objects': 0}
    body = []
    for node in tree.body:
        if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call):
            call = node.value
            if (isinstance(call.func, ast.Attribute) and call.func.attr == 'set_extension_enabled_immediate'
                    and call.args and isinstance(call.args[0], ast.Constant)
                    and call.args[0].value == 'isaacsim.ros2.bridge'):
                counts['bridge'] += 1
                continue
            if isinstance(call.func, ast.Name) and call.func.id == '_build_ros_graph':
                counts['graph'] += 1
                continue
            if isinstance(call.func, ast.Name) and call.func.id == '_build_objects':
                counts['objects'] += 1
                body.append(node)
                body.extend(ast.parse('_MATERIAL = _make_material()').body)
                continue
        body.append(node)
    if counts != {'bridge': 1, 'graph': 1, 'objects': 1}:
        raise ValueError(f'UNRECOGNIZED_SCENE_CONSTRUCTION:{counts}')
    tree.body = body
    return compile(ast.fix_missing_locations(tree), filename, 'exec')
