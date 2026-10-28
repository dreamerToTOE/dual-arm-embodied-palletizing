from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from offline.task28_p5_qp_5.controller import sample_reference, interpolate_pose
@dataclass(frozen=True)
class TemporalSample:
    time_s: float
    q_ref_14: np.ndarray
    qdot_ref_14: np.ndarray
    object_pose_world: np.ndarray
    source_step_index: int
    phase: str

def sample(refs, initial_pose, t):
    q,qd,k,phase=sample_reference(refs,max(0.,float(t)))
    return TemporalSample(float(t),np.asarray(q),np.asarray(qd),np.asarray(interpolate_pose(refs,initial_pose,max(0.,float(t)))),k,phase)

def check_time_pair(pre,post,dt):
    if not (np.isfinite(pre) and np.isfinite(post) and abs((post-pre)-dt)<=1e-7):
        raise ValueError('TEMPORAL_STEP_MISMATCH')
