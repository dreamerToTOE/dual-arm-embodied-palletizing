"""Bounded, single-axis force probe; no robot or attachment controller."""
import math
import json

import numpy as np


def json_record(value):
    def convert(item):
        if isinstance(item, np.generic):
            return item.item()
        raise TypeError(f'unsupported JSON value: {type(item).__name__}')
    return json.dumps(value, allow_nan=False, default=convert)


def clamp(value, limit):
    return max(-limit, min(limit, value))


class Controller:
    def __init__(self):
        self.state = 'APPROACH'
        self.previous_step = 0
        self.previous_time = -1.0
        self.enter_time = 0.0
        self.contact_count = 0
        self.clear_count = 0
        self.exit_y = None
        self.stopping = False
        self.reason = None

    def fail(self, reason):
        self.state, self.reason = 'FAILED', reason
        return {'state': self.state, 'reference_N': 0.0, 'command_N': 0.0, 'reason': reason}

    def update(self, o):
        if self.state in ('DONE', 'FAILED'):
            return {'state': self.state, 'reference_N': 0.0, 'command_N': 0.0, 'reason': self.reason}
        try:
            nums = [o[k] for k in ('time_s', 'dt_s', 'sensor_time_s', 'force_y', 'force_norm',
                                   'event_force_y', 'tool_y', 'cube_y', 'tool_velocity', 'cube_velocity',
                                   'penetration_m', 'support_force_z', 'support_event_z', 'tool_displacement', 'cube_displacement',
                                   'tool_speed', 'cube_speed', 'expected_weight_N')]
            if not all(math.isfinite(x) for x in nums):
                return self.fail('nonfinite_observation')
            if (o['frame'] != 'world' or o['source'] != 'physx_pair' or not o['valid'] or
                o['step'] != self.previous_step + 1 or o['sensor_step'] != o['step'] or
                o['time_s'] <= self.previous_time or abs(o['sensor_time_s'] - o['time_s']) > 1e-6 or
                not 0.005 <= o['dt_s'] <= 0.03 or not 0 <= o['age_ns'] <= 50_000_000):
                return self.fail('stale_or_inconsistent_observation')
            if o['unexpected_pairs']:
                return self.fail('unexpected_contact_pair')
            if o['support_force_z'] < 1.0:
                return self.fail('cube_support_missing')
            if abs(o['support_force_z'] - o['support_event_z']) > 0.25:
                return self.fail('support_force_disagreement')
            if (self.state == 'APPROACH' and o['time_s'] > .2 and o['force_norm'] < .012 and
                    (o['expected_weight_N'] <= 0 or o['support_force_z'] < .5 * o['expected_weight_N'])):
                return self.fail('support_force_weight_inconsistent')
            if abs(o['force_y'] - o['event_force_y']) > 0.02:
                return self.fail('pair_force_disagreement')
            if o['force_norm'] > 1.0:
                return self.fail('excessive_force')
            if o['penetration_m'] > 0.002:
                return self.fail('excessive_penetration')
            if (o['tool_displacement'] > 0.015 or o['cube_displacement'] > 0.010 or
                o['tool_speed'] > 0.04 or o['cube_speed'] > 0.04):
                return self.fail('motion_limit')
            if o['force_norm'] > 0.012 and (o['force_y'] <= 0 or o['force_y'] / o['force_norm'] < 0.95 or not o['tool_pair']):
                return self.fail('contact_identity_or_direction')
        except (KeyError, TypeError, ValueError):
            return self.fail('missing_or_invalid_observation')
        self.previous_step, self.previous_time = o['step'], o['time_s']
        t, force, velocity = o['time_s'], o['force_y'], o['tool_velocity']
        if self.state == 'APPROACH':
            if t > 3.0:
                return self.fail('contact_timeout')
            if force > 0.012:
                self.state, self.enter_time, self.contact_count = 'CONTACT', t, 1
                command = 0.03
            else:
                command = clamp(12.0 * ((0.0 if t < 0.3 else 0.003) - velocity), 0.2)
            ref = 0.0
        elif self.state == 'CONTACT':
            if t - self.enter_time > 0.5:
                return self.fail('contact_unstable')
            if force <= 0.012:
                return self.fail('contact_lost_during_confirmation')
            self.contact_count += 1
            command, ref = 0.03, 0.03
            if self.contact_count >= 3:
                self.state, self.enter_time = 'FORCE_RAMP', t
        elif self.state == 'FORCE_RAMP':
            elapsed = t - self.enter_time
            if force <= 0.005:
                return self.fail('contact_lost_during_ramp')
            ref = 0.03 + 0.12 * min(elapsed / 1.2, 1.0)
            command = max(0.0, min(0.35, ref + 0.2 * (ref - force)))
            if elapsed >= 1.8:
                self.state, self.enter_time, self.exit_y = 'EXIT', t, o['tool_y']
        else:
            elapsed = t - self.enter_time
            if elapsed > 3.0:
                return self.fail('exit_timeout')
            ref = max(0.0, 0.15 * (1.0 - elapsed / 0.3))
            if elapsed < 0.3:
                command = max(0.0, min(0.35, ref + 0.2 * (ref - force)))
            else:
                self.clear_count = self.clear_count + 1 if o['force_norm'] < 1e-5 else 0
                if self.clear_count >= 3 and o['tool_y'] <= self.exit_y - 0.004:
                    self.stopping = True
                command = clamp(12.0 * ((0.0 if self.stopping else -0.004) - velocity), 0.2)
                if self.stopping and abs(velocity) < 0.001 and self.clear_count >= 3:
                    self.state, command = 'DONE', 0.0
        return {'state': self.state, 'reference_N': ref, 'command_N': command, 'reason': self.reason}
