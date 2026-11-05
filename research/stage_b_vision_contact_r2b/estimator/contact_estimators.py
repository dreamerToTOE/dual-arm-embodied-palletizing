"""Small multi-label E0/E1/E2 estimators for the actual Stage B observation policy."""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Mapping

class Edge(str, Enum):
    LEFT_TOOL_CUBE='LEFT_TOOL_CUBE'; RIGHT_TOOL_CUBE='RIGHT_TOOL_CUBE'; CUBE_TABLE='CUBE_TABLE'; CUBE_DEEP_WALL='CUBE_DEEP_WALL'; CUBE_SIDE_WALL='CUBE_SIDE_WALL'
class Mode(str, Enum):
    APPROACH='APPROACH'; DUAL_ATTACHED='DUAL_ATTACHED'; CARRY='CARRY'; RELEASE='RELEASE'; SINGLE_PUSH='SINGLE_PUSH'; DEEP_SEAT='DEEP_SEAT'; AUX_SIDE_PRESS='AUX_SIDE_PRESS'; WITHDRAW='WITHDRAW'; SUSPECTED_STALL='SUSPECTED_STALL'; UNKNOWN='UNKNOWN'
@dataclass(frozen=True)
class ContactBelief:
    step:int; time_s:float; edges:Mapping[Edge,float]; mode:Mode; uncertainty:float; causes:tuple[str,...]=()

def _clamp(v): return max(0., min(1., float(v)))
def _logistic(x, scale): return 1/(1+math.exp(-max(-50.,min(50.,x/scale))))

class E0MultiLabel:
    """Geometry/motion rules; visual pose and proprioception only."""
    def __init__(self, tool_threshold=0.085, table_gap=0.004, wall_gap=0.012, speed=0.01):
        self.tool_threshold=tool_threshold; self.table_gap=table_gap; self.wall_gap=wall_gap; self.speed=speed
    def estimate(self, f):
        x=f.get('cube_position_m'); vis=f.get('visual_confidence',0.0); unknown=x is None or f.get('visual_health') not in ('PRESENT','ORACLE')
        if unknown:
            edges={e:0.5 for e in Edge}; return ContactBelief(f['physics_step'],f['time_s'],edges,Mode.UNKNOWN,1.0,('visual_unknown',))
        left=f.get('left_tool_distance_m'); right=f.get('right_tool_distance_m'); gap=f.get('table_gap_m'); wall=f.get('deep_wall_gap_m'); vx=f.get('cube_velocity_x_m_s')
        edges={Edge.LEFT_TOOL_CUBE: _clamp(_logistic((self.tool_threshold-(left or 9.0)),.008)), Edge.RIGHT_TOOL_CUBE:_clamp(_logistic((self.tool_threshold-(right or 9.0)),.008)), Edge.CUBE_TABLE:_clamp(_logistic((self.table_gap-abs(gap if gap is not None else 9.0)),.0015)), Edge.CUBE_DEEP_WALL:_clamp(_logistic((self.wall_gap-(wall if wall is not None else 9.0)),.002)), Edge.CUBE_SIDE_WALL:0.5}
        tool=max(edges[Edge.LEFT_TOOL_CUBE],edges[Edge.RIGHT_TOOL_CUBE]);
        if wall is not None and wall <= self.wall_gap: mode=Mode.DEEP_SEAT
        elif f.get('suction_closed_count',0)>=2 and tool>=.5: mode=Mode.DUAL_ATTACHED
        elif f.get('commanded_advancement_m') is not None and vx is not None and f['commanded_advancement_m']>0 and vx<self.speed and tool>=.5: mode=Mode.SUSPECTED_STALL
        elif f.get('commanded_advancement_m') is not None and f['commanded_advancement_m']>0 and tool>=.5: mode=Mode.SINGLE_PUSH
        elif tool>=.5: mode=Mode.APPROACH
        else: mode=Mode.APPROACH
        return ContactBelief(f['physics_step'],f['time_s'],edges,mode,max(0.02,1.-vis),())

class E1Hysteresis:
    def __init__(self, base=None, dwell=3): self.base=base or E0MultiLabel(); self.dwell=max(1,int(dwell)); self.state=Mode.UNKNOWN; self.pending=None; self.count=0
    def estimate(self,f):
        raw=self.base.estimate(f)
        if raw.mode==Mode.UNKNOWN: self.state=Mode.UNKNOWN; self.pending=None; self.count=0; return raw
        if raw.mode==self.state: self.pending=None; self.count=0; return raw
        if raw.mode!=self.pending: self.pending=raw.mode; self.count=1
        else: self.count+=1
        if self.count>=self.dwell: self.state=raw.mode; self.pending=None; self.count=0; cause=('hysteresis_transition',)
        else: cause=('candidate_pending',)
        return ContactBelief(raw.step,raw.time_s,raw.edges,self.state,raw.uncertainty,cause)

class E2PhysicsConstrained:
    """Small causal Bayesian-like filter; no oracle measurement update."""
    def __init__(self, base=None, transition=0.12, visual_sigma_px=2.0):
        self.base=base or E0MultiLabel(); self.transition=transition; self.visual_sigma_px=visual_sigma_px; self.prev={e:0.5 for e in Edge}; self.prev_mode=Mode.UNKNOWN
    def estimate(self,f):
        raw=self.base.estimate(f); available=f.get('visual_health') in ('PRESENT','ORACLE') and f.get('visual_confidence',0)>0
        if not available:
            self.prev={e:_clamp(p*(1-self.transition)+.5*self.transition) for e,p in self.prev.items()}; unc=min(1.,raw.uncertainty+.08)
            return ContactBelief(f['physics_step'],f['time_s'],self.prev,self.prev_mode,unc,('prediction_only',))
        posterior={e:_clamp((1-self.transition)*self.prev[e]+self.transition*raw.edges[e]) for e in Edge}
        # A commanded +x motion with little observed x velocity is only a stall
        # hypothesis when the visual observation is valid and a tool edge exists.
        mode=raw.mode
        if mode==Mode.SUSPECTED_STALL and f.get('commanded_advancement_m') is None: mode=Mode.UNKNOWN
        self.prev,self.prev_mode=posterior,mode
        return ContactBelief(f['physics_step'],f['time_s'],posterior,mode,max(0.01,raw.uncertainty*.75),('bayesian_update',))
