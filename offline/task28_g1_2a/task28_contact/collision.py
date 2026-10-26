"""Phase-local support classification around the unchanged G1-1 FCL query."""
from dataclasses import dataclass
import itertools
import numpy as np

from task28_validator.collision import CollisionValidator,CollisionResult
from task28_validator.se3 import checked_transform
from .modes import OperationMode,SupportSite
from .support import SupportPolicy


@dataclass(frozen=True)
class PhaseCollisionResult:
    valid: bool
    reasons: tuple
    reference: CollisionResult
    support: object = None
    accepted_support_contacts: tuple = ()
    remaining_collisions: tuple = ()
    airborne_clearance_m: float | None = None


class PhaseCollisionValidator:
    """No ACM modification: only a verified Cube/table record can be reclassified.

    All other collision and backend errors remain rejection reasons. Airborne
    states retain the reference gate and additionally require >1 um clearance
    above the table plane, excluding the numerical-zero band from that mode.
    """
    def __init__(self,reference,support):
        if type(reference) is not CollisionValidator or type(support) is not SupportPolicy:
            raise ValueError('FIXED_COLLISION_AND_SUPPORT_POLICY_REQUIRED')
        if reference.fixture_dir != support.fixture_dir:
            raise ValueError('COLLISION_SUPPORT_FIXTURE_MISMATCH')
        self.reference,self.support=reference,support

    def validate(self,frames,object_pose,mode,site):
        if type(mode) is not OperationMode or (site is not None and type(site) is not SupportSite):
            raise ValueError('INVALID_PHASE_COLLISION_MODE_OR_SITE')
        bilateral=mode in (OperationMode.BILATERAL_SUPPORTED,OperationMode.BILATERAL_AIRBORNE)
        raw=self.reference.validate(frames,object_pose,attachment_mode='bilateral' if bilateral else 'detached')
        reasons=list(raw.reasons)
        if not self.support._configuration_intact():
            return PhaseCollisionResult(False,tuple(reasons+['SUPPORT_POLICY_CONFIGURATION_CHANGED']),raw,remaining_collisions=raw.collisions)
        if not raw.valid and not raw.collisions and not reasons:
            reasons.append('REFERENCE_REJECTED_WITHOUT_REASON')
        if mode is OperationMode.BILATERAL_AIRBORNE:
            if site is not None:reasons.append('AIRBORNE_SUPPORT_SITE_PRESENT')
            try:
                pose=checked_transform(object_pose)
                vertices=np.array(list(itertools.product(*[(-h,h) for h in self.support.half_size])))
                world=(pose[:3,:3]@vertices.T).T+pose[:3,3]
                clearance=float(world[:,2].min()-self.support.table_top_z_m)
                if clearance <= self.support.PLANE_TOLERANCE_M:
                    reasons.append('AIRBORNE_CLEARANCE_NOT_ESTABLISHED')
            except (ValueError,TypeError,OverflowError) as error:
                clearance=None;reasons.append('INVALID_AIRBORNE_POSE:'+str(error))
            return PhaseCollisionResult(not reasons,tuple(reasons),raw,remaining_collisions=raw.collisions,airborne_clearance_m=clearance)
        geometry=self.support.geometry(object_pose,site.value if type(site) is SupportSite else None)
        reasons.extend(geometry.reasons)
        accepted=[];remaining=[]
        for record in raw.collisions:
            owners=(record['owner_a'],record['owner_b'])
            if owners in (('Cube_01','task28_table'),('task28_table','Cube_01')):
                decision=self.support.accept_contact(record,object_pose,site.value if type(site) is SupportSite else None)
                if decision.valid:
                    code=f"COLLISION:{record['shape_a']}|{record['shape_b']}"
                    if code in reasons:reasons.remove(code)
                    else:reasons.append('SUPPORT_REFERENCE_REASON_MISMATCH')
                    accepted.append({'record':record,'decision':decision})
                    continue
                reasons.extend(decision.reasons)
            remaining.append(record)
        return PhaseCollisionResult(not reasons,tuple(reasons),raw,geometry,tuple(accepted),tuple(remaining))
