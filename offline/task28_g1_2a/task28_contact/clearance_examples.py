"""Versioned, deterministic SYNTHETIC inputs; never measured sensor evidence."""
from dataclasses import replace
import json
from pathlib import Path
from .clearance import ClearanceFrame, CubePoseSource, ConfirmationEvent
from .examples import input_state
from .modes import (Snapshot, OperationMode, SupportSite, AttachmentIntent,
                    ObservedAttachment, EvidenceSource)
from task28_validator.state import AttachmentMode

CONFIG_PATH = Path(__file__).resolve().parents[1]/'examples/r1_configurations.json'


class SyntheticFrames:
    def __init__(self, model):
        self.model = model
        self.configurations = json.loads(CONFIG_PATH.read_text())['configurations']
        self.index = 0
        self.now_s = 20.
        self.intent = AttachmentIntent('initial-detached', 19.9, False, False)

    def make(self, site='supply', gap_m=0., mode=None, request=None, key=None):
        self.index += 1
        self.now_s = 20. + .01*self.index
        stamp = self.now_s - .005
        if request is not None:
            name, attached = request
            self.intent = AttachmentIntent(name, self.now_s-.0075, attached, attached)
        attached = self.intent.left
        if mode is None:
            mode = (OperationMode.BILATERAL_SUPPORTED if gap_m == 0 and attached else
                    OperationMode.DETACHED_SUPPORTED if not attached else
                    OperationMode.BILATERAL_TRANSITIONING)
        config_key = key or f'{site}_gap_{int(round(gap_m*1e6)):04d}um'
        config = self.configurations[config_key]
        state = input_state(self.model, config, stamp, attached, attached,
                            AttachmentMode.BILATERAL if attached else AttachmentMode.DETACHED)
        state = replace(state, cube=replace(state.cube, quaternion_xyzw=tuple(config['object_quaternion_xyzw'])))
        support_mode = mode in (OperationMode.DETACHED_SUPPORTED, OperationMode.BILATERAL_SUPPORTED, OperationMode.RELEASED_SUPPORTED)
        snapshot = Snapshot(state, mode, SupportSite(site) if support_mode else None, self.intent,
                            ObservedAttachment(f'synthetic-feedback-{self.index}', stamp,
                                               self.intent.request_id, attached, attached,
                                               EvidenceSource.INDEPENDENT_SYNTHETIC))
        return ClearanceFrame(snapshot, f'synthetic-cube-{self.index}', CubePoseSource.SYNTHETIC_TEST_INPUT), self.now_s


def representative_sequence(model):
    frames = SyntheticFrames(model)
    output = []
    def add(label, site='supply', gap=0., event=ConfirmationEvent.OBSERVE, **kwargs):
        frame, now = frames.make(site, gap, **kwargs)
        output.append((label, frame, now, event))
    for i in range(3): add(f'initial_support_{i+1}')
    for i in range(3): add(f'attach_{i+1}', event=ConfirmationEvent.ATTACH if i==0 else ConfirmationEvent.OBSERVE,
                           request=('attach-1',True) if i==0 else None)
    add('lift_initiation', event=ConfirmationEvent.BEGIN_LIFT)
    for gap in (.0005,.001,.002): add(f'lift_{gap*1000:g}mm', gap=gap)
    for i in range(3): add(f'high_3mm_{i+1}',gap=.003)
    add('target_transport', 'target', .004)
    add('place_initiation','target',.004,event=ConfirmationEvent.BEGIN_PLACE)
    for gap in (.003,.002,.001,.0005,.0005): add(f'descend_{len(output)}','target',gap)
    for i in range(3): add(f'target_strict_support_{i+1}','target')
    for i in range(3): add(f'release_{i+1}','target',event=ConfirmationEvent.RELEASE if i==0 else ConfirmationEvent.OBSERVE,
                           request=('release-1',False) if i==0 else None,mode=OperationMode.RELEASED_SUPPORTED)
    return tuple(output)
