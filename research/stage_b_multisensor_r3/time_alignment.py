"""Bounded observable-only robot history; interpolate q, then recompute FK."""
from collections import deque
import bisect
import copy
import math
import numpy as np


class RobotHistory:
    def __init__(self, fk, max_samples=240, max_bracket_s=0.034):
        self.fk = fk
        self.rows = deque(maxlen=max_samples)
        self.max_bracket_s = max_bracket_s

    def append(self, *, time_s, physics_step, q, dq, suction_observation, suction_command, controller_command):
        if not math.isfinite(time_s) or (self.rows and time_s <= self.rows[-1]['time_s']):
            raise ValueError('nonmonotonic physics time')
        for values in (q, dq):
            if set(values) != {'left', 'right'} or any(np.asarray(v).shape != (7,) or not np.isfinite(v).all() for v in values.values()):
                raise ValueError('expected seven finite joints per arm')
        self.rows.append(copy.deepcopy(dict(time_s=time_s, physics_step=physics_step, q=q, dq=dq,
            suction_observation=suction_observation, suction_command=suction_command, controller_command=controller_command)))

    def at(self, time_s):
        if not math.isfinite(time_s):
            return {'status': 'UNKNOWN', 'reason': 'missing_or_invalid_render_time'}
        rows = list(self.rows)
        if not rows or time_s < rows[0]['time_s']-1e-9 or time_s > rows[-1]['time_s']+1e-9:
            return {'status': 'UNKNOWN', 'reason': 'outside_retained_history'}
        i = min(bisect.bisect_left([r['time_s'] for r in rows], time_s), len(rows)-1)
        right = rows[i]
        left = right if abs(right['time_s']-time_s) < 1e-9 else rows[max(0, i-1)]
        dt = right['time_s']-left['time_s']
        if dt > self.max_bracket_s:
            return {'status': 'UNKNOWN', 'reason': 'physics_history_gap', 'bracket_s': dt}
        alpha = (time_s-left['time_s'])/dt if dt else 0.
        q = {s: ((1-alpha)*np.array(left['q'][s])+alpha*np.array(right['q'][s])).tolist() for s in ('left','right')}
        dq = {s: ((1-alpha)*np.array(left['dq'][s])+alpha*np.array(right['dq'][s])).tolist() for s in ('left','right')}
        # Discrete observations and commands are held from the past, never interpolated.
        return dict(status='ALIGNED', time_s=time_s, bracket_times_s=[left['time_s'],right['time_s']],
            bracket_steps=[left['physics_step'],right['physics_step']], alpha=alpha, q=q, dq=dq,
            tcp_fk={s: np.asarray(self.fk(s,q[s])).tolist() for s in ('left','right')},
            **{k: copy.deepcopy(left[k]) for k in ('suction_observation','suction_command','controller_command')})
