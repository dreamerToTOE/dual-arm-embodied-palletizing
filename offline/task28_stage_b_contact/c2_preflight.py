"""Stationary support gate: no tool motion until force and momentum agree."""
import math

import numpy as np


class SupportPreflight:
    def __init__(self, mass, gravity):
        self.mass = mass
        self.gravity = np.asarray(gravity, dtype=float)
        if not math.isfinite(mass) or mass <= 0 or self.gravity.shape != (3,) or not np.isfinite(self.gravity).all():
            raise ValueError('invalid runtime mass or gravity')
        self.expected = mass * float(np.linalg.norm(self.gravity))
        self.forces = []
        self.impulses = np.zeros(3)
        self.total_dt = 0.0
        self.start_velocity = None
        self.previous_step = 0
        self.previous_time = 0.0
        self.result = 'PENDING'
        self.reason = None

    def update(self, obs, velocity, contact_impulse):
        try:
            v = np.asarray(velocity, dtype=float)
            j = np.asarray(contact_impulse, dtype=float)
            if (v.shape != (3,) or j.shape != (3,) or not np.isfinite(v).all() or not np.isfinite(j).all() or
                    not all(math.isfinite(obs[k]) for k in ('time_s', 'dt_s', 'support_force_z', 'support_event_z',
                                                              'force_norm', 'age_ns'))):
                return self.fail('PREFLIGHT_NONFINITE')
            if (not obs['valid'] or obs['frame'] != 'world' or obs['sensor_step'] != obs['step'] or
                    obs['step'] != self.previous_step + 1 or obs['time_s'] <= self.previous_time or
                    not .005 <= obs['dt_s'] <= .03 or not 0 <= obs['age_ns'] <= 50_000_000):
                return self.fail('PREFLIGHT_STALE_OR_OUT_OF_ORDER')
            if obs['unexpected_pairs'] or obs['force_norm'] > 1e-5 or np.linalg.norm(v) > .005:
                return self.fail('PREFLIGHT_UNSAFE_OR_NOT_STATIONARY')
            if abs(obs['support_force_z'] - obs['support_event_z']) > .25:
                return self.fail('PREFLIGHT_PAIR_REPORT_MISMATCH')
            if obs['support_force_z'] < 1.0 or self.expected <= 0:
                return self.fail('PREFLIGHT_SUPPORT_MISSING')
            if self.start_velocity is None:
                self.start_velocity = v
            else:
                self.impulses += j
                self.total_dt += obs['dt_s']
            self.forces.append(obs['support_force_z'])
            self.previous_step, self.previous_time = obs['step'], obs['time_s']
            residual = self.mass * (v - self.start_velocity) - self.impulses - self.mass * self.gravity * self.total_dt
            if len(self.forces) >= 12:
                force_error = abs(float(np.mean(self.forces)) - self.expected) / self.expected
                limit = max(.01, .20 * self.expected * self.total_dt)
                if force_error > .20 or np.linalg.norm(residual) > limit:
                    return self.fail('PREFLIGHT_SUPPORT_OR_MOMENTUM_MISMATCH', residual, force_error)
                if len(self.forces) >= 30:
                    self.result = 'PASS'
            return {'status': self.result, 'reason': self.reason, 'samples': len(self.forces),
                    'support_mean_N': float(np.mean(self.forces)), 'expected_weight_N': self.expected,
                    'momentum_residual_Ns': residual.tolist()}
        except (KeyError, TypeError, ValueError):
            return self.fail('PREFLIGHT_MISSING_OBSERVATION')

    def fail(self, reason, residual=None, force_error=None):
        self.result, self.reason = 'FAILED', reason
        return {'status': self.result, 'reason': reason, 'samples': len(self.forces),
                'expected_weight_N': self.expected,
                'support_mean_N': float(np.mean(self.forces)) if self.forces else None,
                'momentum_residual_Ns': residual.tolist() if residual is not None else None,
                'relative_force_error': force_error}
