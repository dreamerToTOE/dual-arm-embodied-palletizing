"""Frozen rigid grasp driven by planned TCPs, never by the measured moving Cube."""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


class RigidObjectReference:
    def __init__(self, object_pose, tcps):
        self.offsets = {s: np.linalg.inv(object_pose) @ np.asarray(T) for s,T in tcps.items()}
        self.previous_pose = None
        self.previous_velocity = np.zeros(6)
        self.previous_q = None
        self.previous_dq = {s: np.zeros(7) for s in tcps}

    def sample(self, planned_tcps, qref, dt, model=None):
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('OBJECT_REFERENCE_DT')
        implied = {s: np.asarray(T) @ np.linalg.inv(self.offsets[s]) for s,T in planned_tcps.items()}
        pose = np.eye(4)
        pose[:3,3] = np.mean([T[:3,3] for T in implied.values()],axis=0)
        pose[:3,:3] = Slerp([0,1],Rotation.from_matrix([implied['left'][:3,:3],implied['right'][:3,:3]]))(.5).as_matrix()
        velocity = np.zeros(6)
        if self.previous_pose is not None:
            velocity[:3] = (pose[:3,3]-self.previous_pose[:3,3])/dt
            velocity[3:] = Rotation.from_matrix(pose[:3,:3] @ self.previous_pose[:3,:3].T).as_rotvec()/dt
        acceleration = (velocity-self.previous_velocity)/dt
        tcps = {s: pose @ offset for s,offset in self.offsets.items()}
        if model is not None:
            projected = {}
            for side,T in tcps.items():
                q = qref[side].copy()
                for _ in range(5):
                    actual = model.fk(side,q)
                    error = np.r_[T[:3,3]-actual[:3,3],
                        Rotation.from_matrix(T[:3,:3]@actual[:3,:3].T).as_rotvec()]
                    if np.linalg.norm(error[:3])<1e-7 and np.linalg.norm(error[3:])<1e-6: break
                    delta = np.linalg.lstsq(model.jacobian(side,q),error,rcond=1e-8)[0]
                    q += np.clip(delta,-.01,.01)
                actual = model.fk(side,q)
                if (not model.within_limits(q) or np.linalg.norm(T[:3,3]-actual[:3,3])>1e-6
                    or np.linalg.norm(Rotation.from_matrix(T[:3,:3]@actual[:3,:3].T).as_rotvec())>1e-5):
                    raise RuntimeError('OBJECT_POSTURE_PROJECTION:' + side)
                projected[side] = q
            qref = projected
        twists, accels, dq, ddq = {}, {}, {}, {}
        for side,T in tcps.items():
            r = T[:3,3]-pose[:3,3]
            twists[side] = np.r_[velocity[:3]+np.cross(velocity[3:],r),velocity[3:]]
            accels[side] = np.r_[acceleration[:3]+np.cross(acceleration[3:],r)
                +np.cross(velocity[3:],np.cross(velocity[3:],r)),acceleration[3:]]
            dq[side] = np.zeros(7) if self.previous_q is None else (qref[side]-self.previous_q[side])/dt
            ddq[side] = (dq[side]-self.previous_dq[side])/dt
        self.previous_pose = pose.copy(); self.previous_velocity = velocity.copy()
        self.previous_q = {s:q.copy() for s,q in qref.items()}; self.previous_dq = dq
        return pose, tcps, twists, accels, dq, ddq, qref
