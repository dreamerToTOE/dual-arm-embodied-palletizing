"""Sufficient interval certificates for the frozen mathematical FK path.

No sampled pass is a proof. No AABB overlap is a collision witness. Contact
record semantics not proved here remain unresolved, including local support.
All proof arithmetic is rational or outward integer/dyadic arithmetic.
"""
from __future__ import annotations
from fractions import Fraction as F
from pathlib import Path
import itertools
import math
import struct
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
for _path in (REPO/'offline/task28_g1_1', REPO/'offline/task28_g1_2a', REPO/'offline/task28_g1_2b_0'):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))
from interval_math import I, SCALE
from task28_edge.checker import SampledEdgeValidator, contract
from task28_edge.interpolation import span_coefficients, reconstruct
from task28_edge.types import SamplingPolicy, Motion
from task28_contact.modes import OperationMode as M
from task28_validator.closed_chain import nominal_grasps

VERIFIED = 'CONTINUOUSLY_VERIFIED'
REJECTED = 'CONTINUOUSLY_REJECTED'
INCONCLUSIVE = 'CONTINUOUS_VERIFICATION_INCONCLUSIVE'
BILATERAL = (M.BILATERAL_SUPPORTED, M.BILATERAL_TRANSITIONING, M.BILATERAL_AIRBORNE)


def fraction(x):
    """Interpret stored binary floating inputs exactly, including URDF parsing."""
    return F.from_float(float(x))


def encode_interval(x):
    return [x._lower, x._upper]  # exact integers; divide by 2**80


def identity():
    return [[I(int(r == c)) for c in range(4)] for r in range(4)]


def matmul(a, b):
    return [[sum((a[r][k]*b[k][c] for k in range(4)), I(0)) for c in range(4)] for r in range(4)]


def rz(theta):
    c, s = theta.cos(), theta.sin()
    out = identity()
    out[0][:2], out[1][:2] = [c, -s], [s, c]
    return out


def origin(element):
    out = identity()
    if element is None:
        return out
    roll, pitch, yaw = (I(float(x)) for x in element.get('rpy', '0 0 0').split())
    cr, sr, cp, sp, cy, sy = roll.cos(), roll.sin(), pitch.cos(), pitch.sin(), yaw.cos(), yaw.sin()
    out[:3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, I(0)],
               [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, I(0)],
               [-sp, cp*sr, cp*cr, I(0)]]
    for k, x in enumerate(element.get('xyz', '0 0 0').split()):
        out[k][3] = I(float(x))
    return out


def quaternion_matrix(q):
    # Normalization cancels sqrt in all matrix entries: exact real rational R.
    x, y, z, w = map(fraction, q)
    n = x*x+y*y+z*z+w*w
    if not n:
        raise ValueError('ZERO_QUATERNION')
    out = identity()
    rows = [[1-2*(y*y+z*z)/n, 2*(x*y-z*w)/n, 2*(x*z+y*w)/n],
            [2*(x*y+z*w)/n, 1-2*(x*x+z*z)/n, 2*(y*z-x*w)/n],
            [2*(x*z-y*w)/n, 2*(y*z+x*w)/n, 1-2*(x*x+y*y)/n]]
    for r in range(3):
        out[r][:3] = list(map(I, rows[r]))
    return out


def same_rotation(a, b):
    # Proportional exact quaternions describe the same normalized rotation.
    return all(fraction(a[i])*fraction(b[j]) == fraction(a[j])*fraction(b[i])
               for i in range(4) for j in range(4))


def bernstein(coefficients):
    """Cubic power coefficients (stored doubles) -> exact Bernstein controls."""
    a, b, c, d = map(fraction, coefficients)
    return (d, d+c/3, d+2*c/3+b/3, a+b+c+d)


def split_controls(p, t):
    rows = [list(p)]
    for _ in range(3):
        rows.append([(1-t)*a+t*b for a,b in zip(rows[-1], rows[-1][1:])])
    return tuple(row[0] for row in rows), tuple(row[-1] for row in rows[::-1])


def polynomial_range(p, lo, hi):
    """Exact de Casteljau restriction, then convex-hull bound."""
    if lo == hi:
        return I(sum(p[k]*math.comb(3,k)*lo**k*(1-lo)**(3-k) for k in range(4)))
    left = p if hi == 1 else split_controls(p, hi)[0]
    restricted = left if lo == 0 else split_controls(left, lo/hi)[1]
    return I.from_bounds(min(restricted), max(restricted))


class ProofModel:
    def __init__(self, checker):
        self.checker = checker
        reference = checker.contact.collision.reference
        self.reference = reference
        self.joints = []
        for node in checker.model.root.findall('joint'):
            if node.get('type') == 'revolute' and node.find('axis').get('xyz') != '0 0 1':
                raise ValueError('UNSUPPORTED_JOINT_AXIS')
            self.joints.append((node.get('name'), node.get('type'), node.find('parent').get('link'),
                                node.find('child').get('link'), origin(node.find('origin'))))
        self.shapes = (*reference.shapes, *reference.environment)
        self.shape_origins = []
        self.local_bounds = []
        collision_nodes = {f'{link.get("name")}/{i}': node for link in checker.model.root.findall('link')
                           for i,node in enumerate(link.findall('collision'))}
        for shape in self.shapes:
            if shape.name in collision_nodes:
                transform = origin(collision_nodes[shape.name].find('origin'))
            else:
                transform = [[I(float(x)) for x in row] for row in shape.origin]
            self.shape_origins.append(transform)
            if shape.kind == 'B':
                halves = [fraction(x)/2 for x in shape.params]
                bounds = [I.from_bounds(-h,h) for h in halves]
            elif shape.kind == 'C':
                r, length = map(fraction, shape.params)
                bounds = [I.from_bounds(-r,r), I.from_bounds(-r,r), I.from_bounds(-length/2,length/2)]
            elif shape.kind == 'M':
                data = Path(shape.params[0]).read_bytes()
                count, = struct.unpack_from('<I', data, 80)
                if len(data) != 84+50*count:
                    raise ValueError('UNSUPPORTED_STL_ENCODING')
                vertices = [struct.unpack_from('<3f',data,84+50*i+12+12*j)
                            for i in range(count) for j in range(3)]
                bounds = [I.from_bounds(min(v[k] for v in vertices),max(v[k] for v in vertices)) for k in range(3)]
            else:
                raise ValueError('UNSUPPORTED_GEOMETRY')
            self.local_bounds.append(bounds)
        self.pairs, skipped = [], 0
        for a,b in itertools.combinations(range(len(self.shapes)),2):
            if self.shapes[a].owner == self.shapes[b].owner:
                continue
            if frozenset((self.shapes[a].owner,self.shapes[b].owner)) in reference.allowed_pairs:
                skipped += 1
            else:
                self.pairs.append((a,b))
        if (len(self.pairs),skipped) != (414,40):
            raise ValueError('FROZEN_PAIR_COVERAGE_MISMATCH')
        self.grasps = []
        for grasp in (nominal_grasps().left,nominal_grasps().right):
            inv = np.eye(4)
            inv[:3,:3] = grasp[:3,:3].T
            inv[:3,3] = -grasp[:3,:3].T @ grasp[:3,3]
            self.grasps.append([[I(float(x)) for x in row] for row in inv])

    def forward(self, left, right):
        values = {f'{side}_fr3_joint{k+1}': q[k] for side,q in (('left',left),('right',right)) for k in range(7)}
        frames, pending = {'world':identity()}, list(self.joints)
        while pending:
            progress = False
            for row in pending[:]:
                name,kind,parent,child,offset = row
                if parent not in frames:
                    continue
                current = matmul(frames[parent],offset)
                if kind == 'revolute':
                    current = matmul(current,rz(values[name]))
                frames[child] = current
                pending.remove(row)
                progress = True
            if not progress:
                raise ValueError('DISCONNECTED_MODEL')
        return frames

    def boxes(self, frames, obj, object_position):
        result = []
        for shape,offset,local in zip(self.shapes,self.shape_origins,self.local_bounds):
            if shape.owner == 'Cube_01' and obj is None:
                # Any SO(3) rotation fits the L1-radius cube (no motion assumption).
                radius = sum((x.max_abs for x in local),F(0))
                result.append([p+I.from_bounds(-radius,radius) for p in object_position])
                continue
            transform = (obj if shape.owner == 'Cube_01' else offset if shape.owner == 'task28_table'
                         else matmul(frames[shape.owner],offset))
            result.append([sum((transform[r][k]*local[k] for k in range(3)),transform[r][3]) for r in range(3)])
        return result


def separation(a, b):
    """A positive exact grid gap is sufficient, never necessary, for separation."""
    for axis in range(3):
        for direction,gap in ((1,b[axis]._lower-a[axis]._upper),(-1,a[axis]._lower-b[axis]._upper)):
            if gap > 0:
                return {'axis':axis,'direction':direction,'gap_lower_scaled':gap}
    return None


def closure_bounds(left, right, obj):
    translation_threshold = I(1e-4).squared()
    angular_threshold = 8*I(1e-3/2).sin().squared()
    rows = []
    for name,a,b in (('left_right',left,right),('left_object',left,obj),('right_object',right,obj)):
        translation = sum(((a[k][3]-b[k][3]).squared() for k in range(3)),I(0))
        frobenius = sum(((a[r][c]-b[r][c]).squared() for r in range(3) for c in range(3)),I(0))
        verified = translation.hi <= translation_threshold.lo and frobenius.hi <= angular_threshold.lo
        rejected = translation.lo > translation_threshold.hi or frobenius.lo > angular_threshold.hi
        rows.append({'pair':name,'translation_squared_m2':encode_interval(translation),
                     'rotation_frobenius_squared':encode_interval(frobenius),
                     'translation_squared_threshold':encode_interval(translation_threshold),
                     'rotation_frobenius_squared_threshold':encode_interval(angular_threshold),
                     'status': REJECTED if rejected else VERIFIED if verified else INCONCLUSIVE})
    return rows


class ContinuousVerifier:
    def __init__(self, checker=None, max_depth=2):
        if type(max_depth) is not int or not 0 <= max_depth <= 8:
            raise ValueError('INVALID_INTERVAL_BUDGET')
        self.checker = checker or SampledEdgeValidator()
        self.model = ProofModel(self.checker)
        self.max_depth = max_depth

    def interval(self, trajectory, index, lo, hi, polys, endpoint=None):
        a,b = trajectory.waypoints[index:index+2]
        if endpoint is None:
            ql,qr,position = [[polynomial_range(p,lo,hi) for p in group] for group in polys]
        else:
            a=b=endpoint
            ql,qr,position = [list(map(I,group)) for group in (a.left_q,a.right_q,a.object_position_m)]
        frames = self.model.forward(ql,qr)
        obj = quaternion_matrix(a.object_quaternion_xyzw) if same_rotation(a.object_quaternion_xyzw,b.object_quaternion_xyzw) else None
        if obj is not None:
            for k in range(3):
                obj[k][3] = position[k]
        required = a.mode in BILATERAL
        rows = closure_bounds(matmul(frames['left_fr3_side_suction_tcp'],self.model.grasps[0]),
                              matmul(frames['right_fr3_side_suction_tcp'],self.model.grasps[1]),obj) if required and obj is not None else []
        closure = (INCONCLUSIVE if required and obj is None else
                   REJECTED if any(row['status']==REJECTED for row in rows) else
                   VERIFIED if all(row['status']==VERIFIED for row in rows) else INCONCLUSIVE)
        boxes = self.model.boxes(frames,obj,position)
        proofs,unresolved = [],[]
        for pair_index,(x,y) in enumerate(self.model.pairs):
            proof = separation(boxes[x],boxes[y])
            if proof is None:
                unresolved.append(pair_index)
            else:
                proofs.append({'pair':pair_index,**proof})
        cube = boxes[-2]
        table_top = fraction(self.checker.contact.collision.support.table_top_z_m)
        vertices = None if obj is None else [[sum((obj[r][k]*v[k] for k in range(3)),obj[r][3]) for r in range(3)]
                    for v in itertools.product(*[(x.lo,x.hi) for x in self.model.local_bounds[-2]])]
        minimum_z = I.minimum(*(v[2] for v in vertices)) if vertices else cube[2]
        penetration = I(table_top)-minimum_z
        table_bounds = self.checker.contact.collision.support.table_bounds_xy
        footprint_verified = all(fraction(table_bounds[0][k]) <= cube[k].lo and cube[k].hi <= fraction(table_bounds[1][k]) for k in range(2))
        # Exact constant-orientation Cube minimum z, not an arbitrary mesh AABB.
        support_rejected = obj is not None and footprint_verified and penetration.lo > fraction(1e-6)
        clearance_verified = (a.mode not in (M.BILATERAL_TRANSITIONING,M.BILATERAL_AIRBORNE)
                              or minimum_z.lo-table_top > fraction(1e-6))
        local_support_unverified = (a.mode not in (M.BILATERAL_TRANSITIONING,M.BILATERAL_AIRBORNE)
                                    or (endpoint is None and trajectory.motion in (Motion.LIFT,Motion.PLACE)))
        return {'span':index,'u':[str(lo),str(hi)],'closure_status':closure,'closure_bounds':rows,
                'collision_status':REJECTED if support_rejected else VERIFIED if not unresolved and clearance_verified and footprint_verified and not local_support_unverified else INCONCLUSIVE,
                'pair_separation_proofs':proofs,'unresolved_pairs':unresolved,
                'shape_world_aabbs':[[encode_interval(x) for x in box] for box in boxes],
                'cube_table_penetration_bound_m':encode_interval(penetration),
                'airborne_clearance_verified':clearance_verified,'support_penetration_rejected':support_rejected,
                'table_footprint_verified':footprint_verified,'local_support_rules_unverified':local_support_unverified,
                'object_rotation_supported':obj is not None}

    def verify(self, trajectory, sampled_evidence=None):
        try:
            with np.errstate(over='raise',invalid='raise',divide='raise'):
                return self._verify(trajectory,sampled_evidence)
        except (ValueError,OverflowError,FloatingPointError,np.linalg.LinAlgError) as error:
            return {'combined_status':INCONCLUSIVE,'reasons':['PROOF_INPUT_OR_ARITHMETIC_UNSUPPORTED:'+str(error)],
                    **{k:{'status':INCONCLUSIVE} for k in ('joint_limits','closed_chain','collision')},'witnesses':[]}

    def _verify(self, trajectory, sampled_evidence=None):
        start = time.perf_counter()
        invalid = contract(trajectory,SamplingPolicy())
        if invalid:
            return {'combined_status':INCONCLUSIVE,'reasons':list(invalid),
                    **{k:{'status':INCONCLUSIVE} for k in ('joint_limits','closed_chain','collision')},'witnesses':[]}
        # The frozen gate remains authoritative for endpoint/mode/temporal checks.
        # External evidence is descriptive only; it cannot spoof a proof prerequisite.
        sampled = self.checker.validate(trajectory)
        self.last_sampled_result = sampled
        def field(key):
            return sampled[key] if isinstance(sampled,dict) else getattr(sampled,key)
        spans,limits,witnesses = [],[],[]
        for endpoint_index,w in enumerate(trajectory.waypoints):
            row=self.interval(trajectory,min(endpoint_index,len(trajectory.waypoints)-2),F(0),F(0),None,endpoint=w)
            row.update({'kind':'explicit_endpoint','waypoint_index':endpoint_index,'depth':0})
            spans.append(row)
        for i,(a,b) in enumerate(zip(trajectory.waypoints,trajectory.waypoints[1:])):
            power = span_coefficients(a,b,trajectory.interpolation)
            polys = [[bernstein(values) for values in zip(*group)] for group in power]
            for side,group in zip(('left','right'),polys[:2]):
                for j,p in enumerate(group):
                    bound = polynomial_range(p,F(0),F(1))
                    lower,upper = map(fraction,(self.checker.model.joint_lower[j],self.checker.model.joint_upper[j]))
                    endpoint_ok = all(lower <= fraction(getattr(w,side+'_q')[j]) <= upper for w in (a,b))
                    limits.append({'span':i,'side':side,'joint':j+1,'range_rad':encode_interval(bound),
                                   'limits_rad':[str(lower),str(upper)],'status':VERIFIED if endpoint_ok and lower<=bound.lo and bound.hi<=upper else INCONCLUSIVE})
            stack = [(F(0),F(1),0)]
            while stack:
                lo,hi,depth = stack.pop()
                row = self.interval(trajectory,i,lo,hi,polys)
                if (depth < self.max_depth and (row['closure_status']==INCONCLUSIVE or row['collision_status']==INCONCLUSIVE)
                        and any(len(set(p))>1 for group in polys for p in group)):
                    mid=(lo+hi)/2
                    stack.extend(((mid,hi,depth+1),(lo,mid,depth+1)))
                else:
                    row['depth']=depth
                    spans.append(row)
            # Discovery only: numerical derivative roots choose candidates. Every
            # claimed rejection is rechecked with an outward POINT enclosure.
            candidates = {0.,.5,1.}
            for group in power:
                for aa,bb,cc,_ in zip(*group):
                    if aa != 0:
                        roots=np.roots([3*aa,2*bb,cc])
                    elif bb != 0:
                        roots=[-cc/(2*bb)]
                    else:
                        roots=[]
                    candidates.update(float(r.real) for r in roots if abs(r.imag if hasattr(r,'imag') else 0)<1e-12 and 0<float(r.real)<1)
            for u in sorted(candidates):
                if u in (0.,1.):
                    continue # endpoints are separately evaluated by frozen gate
                point = self.interval(trajectory,i,fraction(u),fraction(u),polys)
                reasons=[]
                if point['closure_status']==REJECTED and a.mode in BILATERAL:
                    reasons.append('CERTIFIED_CLOSURE_THRESHOLD_VIOLATION')
                if point['support_penetration_rejected']:
                    reasons.append('CERTIFIED_CUBE_TABLE_PENETRATION')
                for side,group in zip(('left','right'),polys[:2]):
                    for j,p in enumerate(group):
                        q=polynomial_range(p,fraction(u),fraction(u))
                        if q.hi < fraction(self.checker.model.joint_lower[j]) or q.lo > fraction(self.checker.model.joint_upper[j]):
                            reasons.append('CERTIFIED_JOINT_LIMIT_VIOLATION')
                if reasons:
                    w=reconstruct(a,b,u,trajectory.interpolation,trajectory.motion,self.checker.contact.collision.support,trajectory.identity)
                    witness={'span':i,'u':u,'time_s':w.time_s,'left_q':w.left_q,'right_q':w.right_q,
                             'object_position_m':w.object_position_m,'object_quaternion_xyzw':w.object_quaternion_xyzw,
                             'certified_reasons':reasons,'point_enclosure':point,'frozen_point_gate':self.checker.evaluate(w)}
                    witnesses.append(witness)
        def status(rows,key):
            return REJECTED if any(row[key]==REJECTED for row in rows) else VERIFIED if all(row[key]==VERIFIED for row in rows) else INCONCLUSIVE
        limit_status = VERIFIED if all(row['status']==VERIFIED for row in limits) else INCONCLUSIVE
        closure_status,collision_status = status(spans,'closure_status'),status(spans,'collision_status')
        if any('CERTIFIED_JOINT_LIMIT_VIOLATION' in w['certified_reasons'] for w in witnesses): limit_status=REJECTED
        if any('CERTIFIED_CLOSURE_THRESHOLD_VIOLATION' in w['certified_reasons'] for w in witnesses): closure_status=REJECTED
        if any('CERTIFIED_CUBE_TABLE_PENETRATION' in w['certified_reasons'] for w in witnesses): collision_status=REJECTED
        sampled_status=getattr(field('status'),'value',field('status'))
        prerequisites=bool(field('endpoints_valid')) and sampled_status=='sampled_edge_valid'
        dimensions=(limit_status,closure_status,collision_status)
        combined=REJECTED if REJECTED in dimensions else VERIFIED if all(s==VERIFIED for s in dimensions) and prerequisites else INCONCLUSIVE
        return {'schema':'task28.g1_2b_1.continuous.v1','trajectory':trajectory.identity,
                'combined_status':combined,'endpoint_validity':bool(field('endpoints_valid')),
                'sampled_edge_validity':sampled_status,'sampled_evidence_used_as_proof':False,
                'joint_limits':{'status':limit_status,'bounds':limits},
                'closed_chain':{'status':closure_status,'rotation_scope':'constant normalized endpoint quaternion only'},
                'collision':{'status':collision_status,'checked_pairs':len(self.model.pairs),'skipped_acm_pairs':40,
                             'unproved_contact_record_rules':'unchanged suction/support normal, location, depth and count policies'},
                'intervals':spans,'witnesses':witnesses,'maximum_subdivision_depth':self.max_depth,
                'interval_scale':SCALE,'arithmetic':'integer_outward_dyadic_80_bits',
                'shape_catalog':[{'index':i,'name':s.name,'owner':s.owner,'kind':s.kind,'params':s.params}
                                 for i,s in enumerate(self.model.shapes)],
                'pair_catalog':self.model.pairs,
                'temporal_confirmation':'frozen explicit observation sequence only; interval points never observations',
                'runtime_s':time.perf_counter()-start}
