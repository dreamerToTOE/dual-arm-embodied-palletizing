"""Synthetic 6D bilateral grasp wrench mapping and weighted decomposition.

Column convention: T_A_B takes B coordinates to A. Each contact wrench is
[f_x,f_y,f_z,tau_x,tau_y,tau_z], exerted ON Cube_01, about that contact TCP
origin. Both local inputs use the G1-1 object-to-TCP grasp convention. Net
wrenches are ABOUT the Cube center, in world or Cube-axis coordinates.

The TCP origin (at +/-61 mm) is a kinematic nominal grasp abstraction with a
1 mm gap, not an identified physical suction patch or measured wrench point.
No real contact capacity, sensor or control authority is inferred.
"""
from dataclasses import dataclass
import numpy as np
from offline.task28_g1_1.task28_validator.closed_chain import nominal_grasps
from offline.task28_g1_1.task28_validator.se3 import checked_transform


def _array(value, shape, name):
    raw = np.asarray(value)
    if raw.shape != shape or not np.issubdtype(raw.dtype, np.number) or np.issubdtype(raw.dtype, np.complexfloating) or np.issubdtype(raw.dtype, np.bool_):
        raise ValueError('INVALID_' + name)
    out = raw.astype(float)
    if not np.isfinite(out).all():
        raise ValueError('INVALID_' + name)
    return out


def _skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def metric_matrix(force_scale_n=1., torque_scale_nm=0.061):
    """Positive definite inverse-square scales, not finite force limits.

    The 0.061 m reference-length equivalent is a mathematical normalization;
    different metrics generally produce different external/internal components.
    """
    values = _array([force_scale_n, torque_scale_nm], (2,), 'SCALES')
    if np.any(values <= 0):
        raise ValueError('INVALID_SCALES')
    return np.diag(np.tile([1 / values[0]**2] * 3 + [1 / values[1]**2] * 3, 2))


def _metric(value):
    w = _array(value, (12, 12), 'METRIC')
    if not np.allclose(w, w.T, rtol=0, atol=1e-12):
        raise ValueError('METRIC_NOT_SYMMETRIC')
    try:
        factor = np.linalg.cholesky(w)
    except np.linalg.LinAlgError as exc:
        raise ValueError('METRIC_NOT_POSITIVE_DEFINITE') from exc
    return w, factor


@dataclass(frozen=True)
class ContactObservation:
    """A labeled numerical wrench at a contact TCP; no implicit provenance."""
    side: str
    body_id: str
    frame_id: str
    provenance: str
    ordering: str
    values: tuple

    def __post_init__(self):
        if self.side not in ('left', 'right'):
            raise ValueError('INVALID_CONTACT_SIDE')
        if self.body_id != 'Cube_01':
            raise ValueError('INVALID_CONTACT_BODY_ID')
        if self.frame_id != self.side + '_fr3_side_suction_tcp':
            raise ValueError('INVALID_CONTACT_FRAME_ID')
        if self.provenance != 'SYNTHETIC_OFFLINE':
            raise ValueError('MISSING_OR_UNSUPPORTED_PROVENANCE')
        if self.ordering != 'force_then_torque_on_cube':
            raise ValueError('INVALID_WRENCH_CONVENTION')
        value = _array(self.values, (6,), 'CONTACT_WRENCH')
        object.__setattr__(self, 'values', tuple(value))


def build_observation(side, body_id, frame_id, provenance, ordering, values):
    return ContactObservation(side, body_id, frame_id, provenance, ordering, values)


def request_physical_estimate(*_args, **_kwargs):
    """Physical wrench observability is not established by the present sources."""
    return {'status': 'BLOCKED', 'reason': 'CALIBRATED_BILATERAL_CONTACT_WRENCH_MEASUREMENTS_UNAVAILABLE',
            'physical_estimate': None, 'execution_authorized': False}


@dataclass(frozen=True)
class Decomposition:
    external: np.ndarray
    internal: np.ndarray
    net_wrench: np.ndarray
    rank: int
    singular_values: np.ndarray
    null_residual: float
    reconstruction_residual: float
    metric_orthogonality: float


@dataclass(frozen=True)
class Allocation:
    contacts: np.ndarray
    required: np.ndarray
    achieved: np.ndarray
    rank: int
    residual: float
    objective: float


class GraspModel:
    """Full-six-dimensional aggregate wrench, without unilateral/friction limits."""

    def __init__(self, world_object, object_left, object_right):
        self.geometry_provenance = 'ABSTRACT_CONTROL'
        self.world_object = checked_transform(world_object)
        self.object_left = checked_transform(object_left)
        self.object_right = checked_transform(object_right)
        # Do not let callers change a validated model in-place.
        for value in (self.world_object, self.object_left, self.object_right):
            value.setflags(write=False)

    @classmethod
    def frozen(cls, world_object):
        pair = nominal_grasps()
        model = cls(world_object, pair.left, pair.right)
        model.geometry_provenance = 'G1_1_NOMINAL_FROZEN_CONTRACT'
        return model

    def matrix(self, output_frame='world'):
        if output_frame not in ('world', 'object'):
            raise ValueError('UNSUPPORTED_OUTPUT_FRAME')
        ro = self.world_object[:3, :3]
        blocks = []
        for t in (self.object_left, self.object_right):
            r = ro @ t[:3, 3]  # center -> TCP, world coordinates
            rotation = ro @ t[:3, :3]
            blocks.append(np.block([[rotation, np.zeros((3, 3))],
                                    [_skew(r) @ rotation, rotation]]))
        g = np.hstack(blocks)
        if output_frame == 'object':
            axes = np.zeros((6, 6))
            axes[:3, :3] = axes[3:, 3:] = ro.T
            g = axes @ g
        return g

    def contact_to_world(self, side, local_wrench):
        """Re-express a wrench at its TCP without changing its reference point."""
        if side not in ('left', 'right'):
            raise ValueError('INVALID_CONTACT_SIDE')
        t = self.object_left if side == 'left' else self.object_right
        r = self.world_object[:3, :3] @ t[:3, :3]
        v = _array(local_wrench, (6,), 'CONTACT_WRENCH')
        return np.r_[r @ v[:3], r @ v[3:]]

    def world_to_contact(self, side, world_wrench):
        if side not in ('left', 'right'):
            raise ValueError('INVALID_CONTACT_SIDE')
        t = self.object_left if side == 'left' else self.object_right
        r = self.world_object[:3, :3] @ t[:3, :3]
        v = _array(world_wrench, (6,), 'CONTACT_WRENCH')
        return np.r_[r.T @ v[:3], r.T @ v[3:]]

    def contact_to_center(self, side, local_wrench, output_frame='world'):
        """Move a local TCP wrench to the Cube center and chosen axes."""
        if side not in ('left', 'right'):
            raise ValueError('INVALID_CONTACT_SIDE')
        block = self.matrix(output_frame)[:, :6] if side == 'left' else self.matrix(output_frame)[:, 6:]
        return block @ _array(local_wrench, (6,), 'CONTACT_WRENCH')

    def center_to_contact(self, side, center_wrench, input_frame='world'):
        """Inverse of contact_to_center; preserves the same physical wrench."""
        if side not in ('left', 'right'):
            raise ValueError('INVALID_CONTACT_SIDE')
        block = self.matrix(input_frame)[:, :6] if side == 'left' else self.matrix(input_frame)[:, 6:]
        return np.linalg.solve(block, _array(center_wrench, (6,), 'CENTER_WRENCH'))

    def net(self, local_wrenches, output_frame='world'):
        return self.matrix(output_frame) @ _array(local_wrenches, (12,), 'CONTACT_WRENCHES')

    def net_observations(self, left, right, output_frame='world'):
        if not isinstance(left, ContactObservation) or not isinstance(right, ContactObservation) or left.side != 'left' or right.side != 'right':
            raise ValueError('INVALID_OBSERVATION_PAIR')
        return self.net(np.r_[left.values, right.values], output_frame)

    def _generalized_inverse(self, metric, output_frame, rcond):
        if not isinstance(rcond, (float, int)) or isinstance(rcond, bool) or not np.isfinite(rcond) or not 0 < rcond < 1:
            raise ValueError('INVALID_RCOND')
        w, l = _metric(metric)
        g = self.matrix(output_frame)
        # W = L L^T and x = L^-T y. Minimum ||y|| gives min x^T W x.
        d = np.linalg.solve(l.T, np.eye(12))
        a = g @ d
        u, s, vh = np.linalg.svd(a, full_matrices=False)
        keep = s > rcond * s[0] if s.size and s[0] else np.zeros_like(s, dtype=bool)
        inv = np.zeros_like(s)
        inv[keep] = 1. / s[keep]
        pinv = d @ ((vh.T * inv) @ u.T)
        return g, w, pinv, int(np.count_nonzero(keep)), s

    def decompose(self, local_wrenches, metric=None, output_frame='world', rcond=1e-12):
        x = _array(local_wrenches, (12,), 'CONTACT_WRENCHES')
        g, w, pinv, rank, s = self._generalized_inverse(metric_matrix() if metric is None else metric, output_frame, rcond)
        net = g @ x
        external = pinv @ net
        internal = x - external
        null_residual = float(np.linalg.norm(g @ internal))
        # A deliberately aggressive SVD cutoff can discard a real map direction.
        # Do not label the discarded load as a valid internal wrench.
        if null_residual > 1e-10 * max(1., float(np.linalg.norm(net))):
            raise ValueError('TRUNCATED_DECOMPOSITION_NOT_IN_NULL_SPACE')
        return Decomposition(external, internal, net, rank, s, null_residual,
                             float(np.linalg.norm(x - external - internal)),
                             float(external @ w @ internal))

    def allocate(self, required, metric=None, output_frame='world', rcond=1e-12):
        """Minimize 1/2 x^T W x subject to G x = required; no capacity limits."""
        target = _array(required, (6,), 'REQUIRED_WRENCH')
        g, w, pinv, rank, _ = self._generalized_inverse(metric_matrix() if metric is None else metric, output_frame, rcond)
        x = pinv @ target
        achieved = g @ x
        residual = float(np.linalg.norm(achieved - target))
        if residual > 1e-10 * max(1., float(np.linalg.norm(target))):
            raise ValueError('UNSUPPORTED_REQUIRED_WRENCH')
        return Allocation(x, target, achieved, rank, residual, float(0.5 * x @ w @ x))
