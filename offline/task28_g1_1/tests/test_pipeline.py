import json
import sys
from dataclasses import replace
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from task28_validator.model import RobotModel
from task28_validator.collision import CollisionValidator
from task28_validator.state import AttachmentMode,InputState,JointObservation,PoseObservation,expected_joint_names
from task28_validator.validator import ClosedChainValidator
from reference_fk import usd_fk


def state_from_control(model,row):
    observations={}
    for side in ('left','right'):
        q=tuple(row[side+'_q'])
        observations[side+'_joints']=JointObservation(side,'world',9.99,expected_joint_names(side),q)
        # Synthetic TCP observations use the independent USD-frame oracle.
        tcp=usd_fk(model.fixture_dir,side,q)
        observations[side+'_tcp']=PoseObservation('world',9.99,tuple(tcp[:3,3]),tuple(Rotation.from_matrix(tcp[:3,:3]).as_quat()))
    quat=tuple(Rotation.from_euler('z',row['object_yaw_rad']).as_quat())
    observations['cube']=PoseObservation('world',9.99,tuple(row['object_position_m']),quat,'Cube_01')
    return InputState(**observations,attachment_mode=AttachmentMode.BILATERAL,left_attached=True,right_attached=True)


class PipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model=RobotModel();cls.collision=CollisionValidator(cls.model.fixture_dir)
        cls.validator=ClosedChainValidator(cls.model,cls.collision)
        cls.controls=json.loads(Path(__file__).with_name('control_states.json').read_text())['states']

    def test_all_three_physical_model_states_pass_all_offline_gates(self):
        for row in self.controls:
            with self.subTest(label=row['label']):
                result=self.validator.validate(state_from_control(self.model,row),10.)
                self.assertTrue(result.valid,result.reasons)
                self.assertTrue(result.closure.valid)
                self.assertTrue(result.collision.valid)
                self.assertEqual(result.collision.checked_shape_pairs,414)
                for _,pos,rot in result.fk_observation_errors:
                    self.assertLess(pos,1e-6);self.assertLess(rot,1e-6)

    def test_limit_violation_is_rejected_before_collision(self):
        state=state_from_control(self.model,self.controls[0])
        q=list(state.left_joints.positions_rad);q[0]=float(self.model.joint_upper[0]+.01)
        state=replace(state,left_joints=replace(state.left_joints,positions_rad=tuple(q)))
        result=self.validator.validate(state,10.)
        self.assertFalse(result.valid);self.assertIn('LEFT_JOINT_LIMIT_VIOLATION',result.reasons)
        self.assertIsNone(result.collision)

    def test_fk_observation_orientation_mismatch(self):
        state=state_from_control(self.model,self.controls[0])
        state=replace(state,left_tcp=replace(state.left_tcp,quaternion_xyzw=(0.,0.,0.,1.)))
        result=self.validator.validate(state,10.)
        self.assertFalse(result.valid);self.assertIn('LEFT_FK_OBSERVATION_MISMATCH',result.reasons)

    def test_payload_collision_still_checked_when_closure_already_fails(self):
        state=state_from_control(self.model,self.controls[0])
        state=replace(state,cube=replace(state.cube,position_m=(.65,0.,.24)))
        result=self.validator.validate(state,10.)
        self.assertFalse(result.valid)
        self.assertFalse(result.closure.valid)
        self.assertFalse(result.collision.valid)
        self.assertIn('COLLISION:Cube_01/box|task28_table/box',result.reasons)

    def test_stale_async_and_invalid_attachment_are_rejected(self):
        state=state_from_control(self.model,self.controls[0])
        candidates=[replace(state,cube=replace(state.cube,stamp_s=9.5)),
                    replace(state,right_joints=replace(state.right_joints,stamp_s=9.95)),
                    replace(state,right_attached=False),
                    replace(state,attachment_mode=AttachmentMode.DETACHED,left_attached=False,right_attached=False)]
        for candidate in candidates:
            result=self.validator.validate(candidate,10.)
            self.assertFalse(result.valid)
            self.assertIsNone(result.collision)

    def test_frozen_supported_endpoints_are_not_silently_accepted_as_carried(self):
        boundaries=json.loads(Path(__file__).with_name('control_states.json').read_text())['supported_boundary_states']
        for row in boundaries:
            with self.subTest(label=row['label']):
                result=self.validator.validate(state_from_control(self.model,row),10.)
                self.assertFalse(result.valid)
                self.assertTrue(result.closure.valid,result.closure.reasons)
                self.assertEqual(result.collision.reasons,('COLLISION:Cube_01/box|task28_table/box',))
                self.assertEqual(max(c['depth_m'] for pair in result.collision.collisions for c in pair['contacts']),0.)
