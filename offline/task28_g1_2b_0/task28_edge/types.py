"""Typed trajectory contract in world/metres/radians/synthetic seconds."""
from dataclasses import dataclass
from enum import Enum
from typing import Protocol
from task28_contact.modes import OperationMode,SupportSite,AttachmentIntent,ObservedAttachment
from task28_contact.clearance import ConfirmationEvent,ClearanceFrame,ClearancePhase


class Source(str,Enum):
    EXPLICIT_SYNTHETIC = 'explicit_synthetic_waypoint'
    MODEL_INTERPOLATION = 'model_interpolation_not_an_observation'


class JointInterpolation(str,Enum):
    LINEAR='linear'
    CUBIC_HERMITE='cubic_hermite'


class TranslationInterpolation(str,Enum):
    LINEAR='linear'
    CUBIC_HERMITE='cubic_hermite'


class Motion(str,Enum):
    HOLD='hold'
    ATTACH='attach_boundary'
    LIFT='vertical_lift'
    TRANSPORT='bilateral_transport'
    PLACE='vertical_placement'
    RELEASE='release_boundary'
    GENERIC='generic_bilateral_candidate'


class EdgeStatus(str,Enum):
    ENDPOINT_VALID='endpoint_valid'
    SAMPLED_EDGE_VALID='sampled_edge_valid'
    SAMPLED_EDGE_REJECTED='sampled_edge_rejected'
    EDGE_INCONCLUSIVE='edge_inconclusive'


@dataclass(frozen=True)
class Provenance:
    source:Source
    cube_evidence_id:str
    attachment_evidence_id:str
    description:str


@dataclass(frozen=True)
class Waypoint:
    time_s:float
    left_q:tuple
    right_q:tuple
    object_position_m:tuple
    object_quaternion_xyzw:tuple
    mode:OperationMode
    support_site:SupportSite|None
    left_attached:bool
    right_attached:bool
    intent:AttachmentIntent
    observed:ObservedAttachment
    provenance:Provenance
    event:ConfirmationEvent=ConfirmationEvent.OBSERVE
    left_velocity_rad_s:tuple|None=None
    right_velocity_rad_s:tuple|None=None
    object_velocity_m_s:tuple|None=None
    observation_stamp_s:float|None=None


@dataclass(frozen=True)
class Interpolation:
    joints:JointInterpolation=JointInterpolation.LINEAR
    translation:TranslationInterpolation=TranslationInterpolation.LINEAR
    rotation:str='shortest_so3_geodesic_xyzw'


@dataclass(frozen=True)
class SamplingPolicy:
    max_dt_s:float=.05
    max_joint_step_rad:float=.02
    max_object_step_m:float=.001
    max_rotation_step_rad:float=.01
    max_samples:int=10001


@dataclass(frozen=True)
class TemporalObservation:
    frame:ClearanceFrame
    now_s:float
    event:ConfirmationEvent


@dataclass(frozen=True)
class Trajectory:
    identity:str
    waypoints:tuple[Waypoint,...]
    motion:Motion
    interpolation:Interpolation=Interpolation()
    scope:str='prescribed_offline_candidate_no_execution_claim'
    # Only EXPLICIT context and waypoint observations go to R1. Interpolated
    # sample density cannot manufacture temporal confirmation.
    temporal_prelude:tuple[TemporalObservation,...]=()
    enforce_temporal_confirmation:bool=False
    required_terminal_phase:ClearancePhase|None=None


class StrongerVerifier(Protocol):
    """Future G1-2B-1 extension seam; deliberately no implementation here."""
    def verify(self,trajectory:Trajectory,sampled_evidence:dict)->dict: ...
