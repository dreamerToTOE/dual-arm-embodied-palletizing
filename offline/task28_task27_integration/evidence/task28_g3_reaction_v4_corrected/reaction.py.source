"""Read-only articulation transmission recorder; never a suction-force sensor."""
import numpy as np


def checked_reactions(values, body_names, joint_indices):
    values = np.asarray(values, dtype=float)
    if values.shape != (len(body_names), 6) or not np.isfinite(values).all():
        raise ValueError('INCOMING_REACTION_SHAPE_OR_FINITE')
    rows = {name: int(index) + 1 for name, index in joint_indices.items()}
    if any(index < 1 or index >= len(body_names) for index in rows.values()):
        raise ValueError('INCOMING_REACTION_INDEX')
    return values.copy(), rows


class ReactionRecorder:
    def __init__(self, articulations, stage):
        from pxr import Usd, UsdPhysics
        self.articulations = articulations
        try:
            from omni.isaac.dynamic_control import _dynamic_control
            self.dc = _dynamic_control.acquire_dynamic_control_interface()
        except Exception:
            self.dc = None
        self.metadata = {}
        for side, articulation in articulations.items():
            view = articulation._articulation_view
            names = list(view.body_names)
            indices = dict(view._metadata.joint_indices)
            joints = {}
            for prim in Usd.PrimRange(stage.GetPrimAtPath(f'/World/{side}_fr3')):
                if not prim.IsA(UsdPhysics.Joint):
                    continue
                joint = UsdPhysics.Joint(prim)
                rotation = joint.GetLocalRot1Attr().Get()
                joints[prim.GetName()] = {
                    'path': str(prim.GetPath()),
                    'axis': str(prim.GetAttribute('physics:axis').Get()),
                    'body0': [str(p) for p in joint.GetBody0Rel().GetTargets()],
                    'body1': [str(p) for p in joint.GetBody1Rel().GetTargets()],
                    'local_pos1_m': list(joint.GetLocalPos1Attr().Get()),
                    'local_rot1_wxyz': [float(rotation.GetReal()), *map(float, rotation.GetImaginary())],
                }
            inertial = {}
            for method in ('get_masses', 'get_coms', 'get_inertias'):
                try:
                    inertial[method] = np.asarray(getattr(view._physics_view, method)(), dtype=float).tolist()
                except Exception as exc:
                    inertial[method] = {'unavailable': repr(exc)}
            self.metadata[side] = {
                'body_names_by_row': names,
                'joint_indices': {name: int(index) for name, index in indices.items()},
                'joint_rows': {name: int(index) + 1 for name, index in indices.items()},
                'usd_joints': joints, 'link_inertial_runtime': inertial,
            }

    def read(self):
        result = {}
        for side, articulation in self.articulations.items():
            try:
                meta = self.metadata[side]
                values, _ = checked_reactions(articulation.get_measured_joint_forces(),
                                              meta['body_names_by_row'], meta['joint_indices'])
                physics = articulation._articulation_view._physics_view
                state = {'valid': True, 'incoming_child_joint_N_Nm': values.tolist()}
                if self.dc is not None:
                    try:
                        path = f'/World/{side}_fr3/fr3_link8/task26_side_surface_gripper_joint'
                        handle = self.dc.get_d6_joint(path)
                        state['d6'] = {'handle': int(handle), 'reaction_getter': None,
                                       'available_methods': [name for name in dir(self.dc) if 'd6' in name.lower()]}
                        if handle:
                            broken = getattr(self.dc, 'get_d6_joint_constraint_is_broken', None)
                            state['d6']['broken'] = bool(broken(handle)) if broken else None
                            properties = getattr(self.dc, 'get_d6_joint_properties', None)
                            if properties:
                                props = properties(handle)
                                state['d6']['bodies'] = [str(self.dc.get_rigid_body_path(body))
                                                          for body in (props.body0, props.body1)]
                    except Exception as exc:
                        state['d6'] = {'reaction_getter': None, 'error': repr(exc)}
                for method in ('get_link_transforms', 'get_link_velocities', 'get_link_accelerations'):
                    state[method] = np.asarray(getattr(physics, method)(), dtype=float).tolist()
                result[side] = state
            except Exception as exc:
                result[side] = {'valid': False, 'error': repr(exc)}
        return result
