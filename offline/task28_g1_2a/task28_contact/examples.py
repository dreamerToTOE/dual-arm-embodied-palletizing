"""Deterministic synthetic snapshots; never attachment feedback from a command.

Intent and feedback flags are separately specified in the versioned JSON input.
TCP observations here are synthetic model poses, not measured ROS/sensor values.
"""
from dataclasses import dataclass
import json
from pathlib import Path
from scipy.spatial.transform import Rotation
from task28_validator.state import AttachmentMode,InputState,JointObservation,PoseObservation,expected_joint_names
from .modes import Snapshot,OperationMode,SupportSite,AttachmentIntent,ObservedAttachment,EvidenceSource,TransitionEvent


@dataclass(frozen=True)
class NamedSnapshot:
    label: str
    snapshot: Snapshot
    now_s: float


def input_state(model,configuration,stamp_s,left_attached,right_attached,attachment_mode):
    fields={}
    for side in ('left','right'):
        q=tuple(configuration[side+'_q'])
        fields[side+'_joints']=JointObservation(side,'world',stamp_s,expected_joint_names(side),q)
        tcp=model.fk(side,q)
        fields[side+'_tcp']=PoseObservation('world',stamp_s,tuple(tcp[:3,3]),tuple(Rotation.from_matrix(tcp[:3,:3]).as_quat()))
    fields['cube']=PoseObservation('world',stamp_s,tuple(configuration['object_position_m']),
                                 tuple(Rotation.from_euler('z',configuration['object_yaw_rad']).as_quat()),'Cube_01')
    return InputState(**fields,attachment_mode=attachment_mode,left_attached=left_attached,right_attached=right_attached)


def load_sequence(model):
    path=Path(__file__).resolve().parents[1]/'examples/a_stage_sequence.json'
    data=json.loads(path.read_text());sequence=[]
    for row in data['snapshots']:
        # These flags are deliberately read from the feedback record, not intent.
        feedback=row['observed']
        mode=OperationMode(row['mode'])
        old_mode=AttachmentMode.BILATERAL if mode in (OperationMode.BILATERAL_SUPPORTED,OperationMode.BILATERAL_AIRBORNE) else AttachmentMode.DETACHED
        stamp=row['now_s']-.005
        state=input_state(model,data['configurations'][row['configuration']],stamp,feedback['left'],feedback['right'],old_mode)
        snapshot=Snapshot(state,mode,SupportSite(row['site']) if row['site'] else None,
                          AttachmentIntent(**row['intent']),
                          ObservedAttachment(stamp_s=stamp,source=EvidenceSource.INDEPENDENT_SYNTHETIC,**feedback))
        sequence.append(NamedSnapshot(row['label'],snapshot,row['now_s']))
    return tuple(sequence),tuple(TransitionEvent(value) for value in data['events'])
