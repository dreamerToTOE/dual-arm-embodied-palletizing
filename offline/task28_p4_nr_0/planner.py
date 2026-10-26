"""Bounded object-sampling NR-RRT with projected-midpoint local connections.

P4 relation: full-coordinate SVD Newton projection, nearest/range tree extension
and recursive projected midpoint connection. Object-conditioned residuals, FR3
redundancy, metric and lift/place primitives are explicit Task28 adaptations.
No precomputed trajectory or old IK path is loaded. All accepted parent edges
have been checked before insertion; the result is a tree backtrace, not a path
created before the search and subsequently relabelled.
"""
from __future__ import annotations
from dataclasses import dataclass,field
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation
from . import bootstrap  # noqa: F401
from .projection import NRProjector
from .validation import CandidateValidator
from task28_validator.se3 import checked_transform
from task28_contact.modes import SupportSite
from task28_edge.types import Motion,EdgeStatus


@dataclass(frozen=True)
class PlannerConfig:
    max_expansions:int=64
    max_lift_trials:int=3
    max_projection_calls:int=1600
    max_nr_iterations_per_call:int=60
    max_step_rad:float=.2
    backtracks:int=8
    svd_cutoff:float=1e-12
    anchor_restarts:int=8
    max_connection_depth:int=8
    leaf_translation_m:float=.0025
    leaf_rotation_rad:float=.005
    leaf_joint_step_rad:float=.025
    object_extension_range_m:float=.04
    goal_bias:float=.35
    sample_translation_lower_m:tuple=(.58,-.15,.274)
    sample_translation_upper_m:tuple=(.89,.10,.44)
    sample_rotation_vector_abs_rad:tuple=(.02,.02,.03)
    lift_height_range_m:tuple=(.28,.42)
    max_accepted_nodes:int=80

    def __post_init__(self):
        integer_bounds={'max_expansions':(1,1000),'max_lift_trials':(1,10),'max_projection_calls':(1,10000),
            'max_nr_iterations_per_call':(1,2000),'backtracks':(0,20),'anchor_restarts':(1,32),
            'max_connection_depth':(0,12),'max_accepted_nodes':(2,1001)}
        for key,(lo,hi) in integer_bounds.items():
            if type(getattr(self,key)) is not int or not lo<=getattr(self,key)<=hi:raise ValueError('INVALID_PLANNER_BUDGET:'+key)
        for key in ('max_step_rad','svd_cutoff','leaf_translation_m','leaf_rotation_rad','leaf_joint_step_rad','object_extension_range_m'):
            value=getattr(self,key)
            if type(value) not in (int,float) or not math.isfinite(value) or value<=0:raise ValueError('INVALID_PLANNER_SCALE:'+key)
        if type(self.goal_bias) not in (int,float) or not 0<=self.goal_bias<=1:raise ValueError('INVALID_GOAL_BIAS')
        for name in ('sample_translation_lower_m','sample_translation_upper_m','sample_rotation_vector_abs_rad'):
            values=np.asarray(getattr(self,name))
            if values.shape!=(3,) or values.dtype.kind not in 'fi' or not np.isfinite(values).all():raise ValueError('INVALID_WORKSPACE:'+name)
        if np.any(np.asarray(self.sample_translation_lower_m)>=self.sample_translation_upper_m):raise ValueError('INVALID_WORKSPACE_ORDER')
        if np.any(np.asarray(self.sample_rotation_vector_abs_rad)<0):raise ValueError('INVALID_ROTATION_RANGE')
        if len(self.lift_height_range_m)!=2 or not all(type(x) in (int,float) and math.isfinite(x) for x in self.lift_height_range_m) or not 0.263<self.lift_height_range_m[0]<self.lift_height_range_m[1]:raise ValueError('INVALID_LIFT_RANGE')


@dataclass
class PlanResult:
    found:bool
    reason:str
    seed:int
    start:np.ndarray
    goal:np.ndarray
    lift:list=field(default_factory=list)
    transport:list=field(default_factory=list)
    place:list=field(default_factory=list)
    statistics:dict=field(default_factory=dict)
    validation:dict|None=None
    execution_authorized:bool=False
    label:str='NO_ACCEPTED_CANDIDATE'


class PlanningBudget(Exception):pass


def distances(a,b):
    return float(np.linalg.norm(a[:3,3]-b[:3,3])),float(np.linalg.norm(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).as_rotvec()))


def interpolate(a,b,u):
    out=np.eye(4);out[:3,3]=(1-u)*a[:3,3]+u*b[:3,3]
    out[:3,:3]=a[:3,:3]@Rotation.from_rotvec(u*Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).as_rotvec()).as_matrix()
    return out


class Planner:
    def __init__(self,seed,config=PlannerConfig(),validator=None,emit=None):
        if type(seed) is not int or seed<0:raise ValueError('NONNEGATIVE_INTEGER_SEED_REQUIRED')
        config.__post_init__();self.seed=seed;self.config=config
        self.rng=np.random.default_rng(seed);self.validator=validator or CandidateValidator();self.model=self.validator.model
        self.projector=NRProjector(self.model,config.max_nr_iterations_per_call,config.max_step_rad,config.backtracks,config.svd_cutoff)
        self.emit=emit or (lambda row:None)
        self.stats={key:0 for key in ('sampled_nodes','accepted_tree_nodes','ik_attempts','ik_failures','candidate_connection_attempts',
            'collision_rejections','closure_rejections','joint_limit_rejections','other_gate_rejections','leaf_edges_checked','full_candidate_attempts')}
        self.stats['rejection_reasons']={};self._connections=0

    def reject(self,reasons):
        reasons=tuple(reasons)
        for reason in reasons:self.stats['rejection_reasons'][reason]=self.stats['rejection_reasons'].get(reason,0)+1
        groups={'collision_rejections':('COLLISION','FCL','PENETRATION','CONTACT'),
                'closure_rejections':('GRASP','CLOSURE','LEFT_RIGHT_OBJECT'),
                'joint_limit_rejections':('JOINT_LIMIT','OUT_OF_LIMIT')}
        any_group=False
        for key,markers in groups.items():
            if any(any(m in r for m in markers) for r in reasons):self.stats[key]+=1;any_group=True
        if not any_group:self.stats['other_gate_rejections']+=1

    def project(self,X,q,purpose):
        if self.stats['ik_attempts']>=self.config.max_projection_calls:raise PlanningBudget('NR_CALL_BUDGET_EXHAUSTED')
        self.stats['ik_attempts']+=1
        result=self.projector.project(X,q)
        self.emit({'kind':'projection','purpose':purpose,'object_pose':X,'result':result})
        if not result.success:
            self.stats['ik_failures']+=1;self.reject((result.reason,));return None
        return (X.copy(),np.asarray(result.q14))

    def gate(self,node,site=None):
        result=self.validator.state(*node,site=site)
        self.emit({'kind':'state_gate','node':node,'result':result})
        if not result['valid']:self.reject(result['reasons'])
        return result['valid']

    def anchor(self,X,site,label):
        midpoint=(self.model.joint_lower+self.model.joint_upper)/2
        for attempt in range(self.config.anchor_restarts):
            if attempt==0:q=np.tile(self.model.home,2)
            elif attempt==1:q=np.tile(midpoint,2)
            else:q=self.rng.uniform(np.tile(self.model.joint_lower,2),np.tile(self.model.joint_upper,2))
            node=self.project(X,q,label+f':restart{attempt}')
            if node is not None and self.gate(node,site):return node
        return None

    def connect(self,a,b,motion,depth=0):
        self.stats['candidate_connection_attempts']+=1;self._connections+=1
        d,angle=distances(a[0],b[0]);dq=float(np.max(np.abs(a[1]-b[1])))
        if (d<=self.config.leaf_translation_m and angle<=self.config.leaf_rotation_rad and dq<=self.config.leaf_joint_step_rad):
            self.stats['leaf_edges_checked']+=1
            result=self.validator.edge([a,b],motion=motion,identity=f'nr-search-{self.seed}-{self._connections}')
            self.emit({'kind':'leaf_edge','motion':motion,'nodes':[a,b],'result':result})
            if result.status is EdgeStatus.SAMPLED_EDGE_VALID:return [a,b]
            self.reject(result.reasons);return None
        if depth>=self.config.max_connection_depth:
            self.reject(('PROJECTED_RBS_DEPTH_BUDGET',));return None
        # The midpoint is an NR projection target and its joint seed is the
        # endpoint arithmetic average: P4 RBS relationship, adapted to object X.
        mid=self.project(interpolate(a[0],b[0],.5),.5*(a[1]+b[1]),'projected_RBS_midpoint')
        if mid is None or not self.gate(mid):return None
        left=self.connect(a,mid,motion,depth+1)
        if left is None:return None
        right=self.connect(mid,b,motion,depth+1)
        return None if right is None else left+right[1:]

    def sample(self,goal):
        self.stats['sampled_nodes']+=1
        bias=bool(self.rng.random()<self.config.goal_bias)
        if bias:X=goal.copy()
        else:
            X=np.eye(4);X[:3,3]=self.rng.uniform(self.config.sample_translation_lower_m,self.config.sample_translation_upper_m)
            spread=np.asarray(self.config.sample_rotation_vector_abs_rad)
            X[:3,:3]=Rotation.from_rotvec(self.rng.uniform(-spread,spread)).as_matrix()
        self.emit({'kind':'object_sample','goal_bias':bias,'object_pose':X})
        return X

    def plan(self,start,goal,*,continuous=True):
        start,goal=checked_transform(start),checked_transform(goal);clock=time.perf_counter();result=PlanResult(False,'NOT_STARTED',self.seed,start,goal)
        def finish(reason):
            result.reason=reason;self.stats['planning_time_s']=time.perf_counter()-clock
            result.statistics=dict(self.stats);return result
        self.emit({'kind':'query','start':start,'goal':goal,'seed':self.seed})
        try:
            # The necessary reach sphere is independent of a local IK failure.
            for X,label in [(start,'START'),(goal,'GOAL')]:
                for side,grasp in [('left',self.projector._grasps.left),('right',self.projector._grasps.right)]:
                    target=X@grasp;base=np.asarray(self.model.manifest['bases_world_m'][side])
                    if np.linalg.norm(target[:3,3]-base)>self.model.reach_bounds[side]+1e-12:
                        return finish(label+'_OUTSIDE_NECESSARY_REACH_BOUND')
            start_node=self.anchor(start,SupportSite.SUPPLY,'start_anchor')
            if start_node is None:return finish('START_NR_OR_STATE_GATE_FAILED')
            goal_probe=self.anchor(goal,SupportSite.TARGET,'goal_anchor_precondition')
            if goal_probe is None:return finish('GOAL_NR_OR_STATE_GATE_FAILED')
            for lift_trial in range(self.config.max_lift_trials):
                if self.stats['accepted_tree_nodes']>=self.config.max_accepted_nodes:return finish('TREE_NODE_BUDGET_EXHAUSTED')
                height=float(self.rng.uniform(*self.config.lift_height_range_m));X=start.copy();X[2,3]=height
                self.stats['sampled_nodes']+=1;self.emit({'kind':'lift_sample','trial':lift_trial,'height_m':height})
                root=self.project(X,start_node[1],'lift_endpoint')
                if root is None or not self.gate(root):continue
                lift=self.connect(start_node,root,Motion.LIFT)
                if lift is None:continue
                target=goal.copy();target[2,3]=height
                tree=[{'node':root,'parent':None,'edge':None}];self.stats['accepted_tree_nodes']+=1
                for iteration in range(self.config.max_expansions):
                    if self.stats['accepted_tree_nodes']>=self.config.max_accepted_nodes:break
                    sampled=self.sample(target)
                    metric=lambda node:sum(v*w for v,w in zip(distances(node[0],sampled),(1.,.1)))
                    parent=min(range(len(tree)),key=lambda j:metric(tree[j]['node']))
                    base=tree[parent]['node'];d,angle=distances(base[0],sampled)
                    distance=d+.1*angle
                    if distance<1e-12:continue
                    reached=distance<=self.config.object_extension_range_m
                    X=sampled if reached else interpolate(base[0],sampled,self.config.object_extension_range_m/distance)
                    node=self.project(X,base[1],'tree_extend')
                    if node is None or not self.gate(node):continue
                    edge=self.connect(base,node,Motion.TRANSPORT)
                    if edge is None:continue
                    tree.append({'node':node,'parent':parent,'edge':edge});self.stats['accepted_tree_nodes']+=1
                    index=len(tree)-1;self.emit({'kind':'tree_insert','iteration':iteration,'node_index':index,'parent':parent,'node':node,'edge':edge})
                    if max(distances(node[0],target))>1e-10:continue
                    end=self.project(goal,node[1],'placement_endpoint')
                    if end is None or not self.gate(end,SupportSite.TARGET):continue
                    place=self.connect(node,end,Motion.PLACE)
                    if place is None:continue
                    edges=[];current=index
                    while tree[current]['parent'] is not None:
                        edges.append(tree[current]['edge']);current=tree[current]['parent']
                    transport=[root]
                    for segment in reversed(edges):transport.extend(segment[1:])
                    self.stats['full_candidate_attempts']+=1
                    validation=self.validator.confirm_plan(lift,transport,place,identity=f'nr-seed{self.seed}-candidate{self.stats["full_candidate_attempts"]}',continuous=continuous)
                    self.emit({'kind':'full_candidate','lift':lift,'transport':transport,'place':place,'validation':validation})
                    if not validation['valid']:
                        self.reject(validation['reasons']);continue
                    certified_rejections=[name for name,proof in validation['continuous'].items() if proof.get('combined_status')=='CONTINUOUSLY_REJECTED']
                    if certified_rejections:
                        self.reject(tuple('CONTINUOUS_CERTIFIED_REJECTION:'+name for name in certified_rejections));continue
                    result.found=True;result.lift=lift;result.transport=transport;result.place=place;result.validation=validation
                    result.label='OFFLINE_CANDIDATE_UNCERTIFIED'
                    points=lift+transport[1:]+place[1:]
                    self.stats['path_translation_length_m']=sum(distances(a[0],b[0])[0] for a,b in zip(points,points[1:]))
                    self.stats['path_rotation_length_rad']=sum(distances(a[0],b[0])[1] for a,b in zip(points,points[1:]))
                    self.stats['path_joint_length_rad']=sum(float(np.linalg.norm(a[1]-b[1])) for a,b in zip(points,points[1:]))
                    self.stats['path_waypoints']=len(points)
                    sampled_results=list(validation['sampled'].values())
                    self.stats['maximum_sampled_closure_translation_m']=max(s.maxima['max_closure_translation_m'] or 0 for s in sampled_results)
                    self.stats['maximum_sampled_closure_rotation_rad']=max(s.maxima['max_closure_rotation_rad'] or 0 for s in sampled_results)
                    self.stats['minimum_joint_limit_margin_rad']=min(s.maxima['minimum_joint_limit_margin_rad'] for s in sampled_results)
                    return finish('ALL_OFFLINE_SAMPLED_GATES_PASSED_CONTINUOUS_STATUS_SEPARATE')
            return finish('BOUNDED_SEARCH_EXHAUSTED_WITHOUT_ACCEPTED_CANDIDATE')
        except PlanningBudget as error:return finish(str(error))
