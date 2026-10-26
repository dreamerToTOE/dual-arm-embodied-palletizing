import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import unittest
import numpy as np
from task28_validator.closed_chain import GraspPair,desired_tcps,evaluate_closure,nominal_grasps
from task28_validator.se3 import inverse,pose_matrix,residual,transform


class ClosedChainTests(unittest.TestCase):
    def test_independently_defined_symmetric_grasp(self):
        obj=np.eye(4);obj[:3,3]=[.65,0,.4]
        left=np.array([[0,1,0,.65],[1,0,0,-.061],[0,0,-1,.4],[0,0,0,1.]])
        right=np.array([[0,-1,0,.65],[-1,0,0,.061],[0,0,-1,.4],[0,0,0,1.]])
        result=evaluate_closure(obj,left,right)
        self.assertTrue(result.valid,result.reasons)
        self.assertLess(result.translation_error_m,1e-14)
        self.assertLess(result.rotation_error_rad,1e-14)

    def test_independent_nonzero_translation(self):
        # Explicit expected coordinates, not desired_tcps calling the same code.
        obj=transform(translation=[.72,-.025,.48])
        left=transform(np.array([[0,1,0],[1,0,0],[0,0,-1]]),[.72,-.086,.48])
        right=transform(np.array([[0,-1,0],[-1,0,0],[0,0,-1]]),[.72,.036,.48])
        self.assertTrue(evaluate_closure(obj,left,right).valid)

    def test_independent_quarter_turn(self):
        # Rz(pi/2) rotates the Y contact offsets onto X, and rotations change too.
        obj=np.array([[0,-1,0,.7],[1,0,0,.02],[0,0,1,.5],[0,0,0,1.]])
        left=np.array([[-1,0,0,.761],[0,1,0,.02],[0,0,-1,.5],[0,0,0,1.]])
        right=np.array([[1,0,0,.639],[0,-1,0,.02],[0,0,-1,.5],[0,0,0,1.]])
        result=evaluate_closure(obj,left,right)
        self.assertTrue(result.valid,result.reasons)
        self.assertLess(result.rotation_error_rad,1e-14)

    def test_grasp_transform_mismatch(self):
        obj=transform(translation=[.65,0,.4]);left,right=desired_tcps(obj)
        grasp=nominal_grasps();wrong=grasp.right.copy();wrong[0,3]+=.01
        result=evaluate_closure(obj,left,right,GraspPair(grasp.left,wrong))
        self.assertFalse(result.valid)
        self.assertAlmostEqual(result.translation_error_m,.01,places=12)
        self.assertIn('LEFT_RIGHT_OBJECT_TRANSLATION_MISMATCH',result.reasons)

    def test_orientation_rejected_despite_identical_tcp_positions(self):
        obj=transform(translation=[.65,0,.4]);left,right=desired_tcps(obj)
        wrong=right.copy();wrong[:3,:3]=right[:3,:3]@np.diag([-1.,-1.,1.])
        result=evaluate_closure(obj,left,wrong)
        self.assertFalse(result.valid)
        self.assertAlmostEqual(result.rotation_error_rad,np.pi,places=12)
        self.assertIn('RIGHT_GRASP_ROTATION_MISMATCH',result.reasons)

    def test_common_pose_bias_does_not_fool_mutual_closure(self):
        obj=transform(translation=[.65,0,.4]);left,right=desired_tcps(obj)
        left[:3,3]+=[.01,0,0];right[:3,3]+=[.01,0,0]
        result=evaluate_closure(obj,left,right)
        self.assertLess(result.translation_error_m,1e-14)
        self.assertFalse(result.valid)
        self.assertIn('LEFT_GRASP_TRANSLATION_MISMATCH',result.reasons)

    def test_invalid_full_transform(self):
        for value in (np.zeros((4,4)),np.diag([-1.,1.,1.,1.]),np.full((4,4),np.nan)):
            with self.assertRaises(ValueError):inverse(value)
        with self.assertRaises(ValueError):pose_matrix([0,0,0],[0,0,0,0])
