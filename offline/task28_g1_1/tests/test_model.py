import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from task28_validator.model import RobotModel
from task28_validator.se3 import residual,transform
from reference_fk import usd_fk


class ModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.model=RobotModel()

    def test_joint_limits_and_finite_values(self):
        m=self.model
        self.assertTrue(m.within_limits(m.home))
        for q in (m.joint_lower-.001,m.joint_upper+.001,[np.nan]*7,[np.inf]*7,[0.]*6,[True]*7):
            self.assertFalse(m.within_limits(q))
        self.assertTrue(m.within_limits(m.joint_lower))
        self.assertTrue(m.within_limits(m.joint_upper))

    def test_analytic_geometric_jacobian_against_central_differences(self):
        m=self.model; q=np.array([.3,-.7,.5,-2.2,.6,1.4,.4]);h=1e-6
        for side in ('left','right'):
            numerical=np.zeros((6,7))
            for i in range(7):
                plus=q.copy();minus=q.copy();plus[i]+=h;minus[i]-=h
                first,second=m.fk(side,plus),m.fk(side,minus)
                numerical[:3,i]=(first[:3,3]-second[:3,3])/(2*h)
                numerical[3:,i]=Rotation.from_matrix(first[:3,:3]@second[:3,:3].T).as_rotvec()/(2*h)
            self.assertLess(np.max(np.abs(m.jacobian(side,q)-numerical)),2e-8)

    def test_bounded_ik_target_defined_by_independent_usd(self):
        m=self.model;q=np.array([.3,-.7,.5,-2.2,.6,1.4,.4])
        for side in ('left','right'):
            target=usd_fk(m.fixture_dir,side,q)
            solved=m.ik(side,target,max_attempts=4,max_nfev=250)
            self.assertTrue(solved.success,solved)
            self.assertTrue(m.within_limits(solved.q))
            independent=usd_fk(m.fixture_dir,side,solved.q)
            pos,rot=residual(independent,target)
            self.assertLess(pos,1.1e-6);self.assertLess(rot,1.1e-5)

    def test_unreachable_target_has_necessary_geometric_certificate(self):
        result=self.model.ik('left',transform(translation=[100,0,0]))
        self.assertFalse(result.success)
        self.assertEqual(result.reason,'IK_OUTSIDE_REACH_BOUND')
        self.assertEqual(result.function_evaluations,0)

    def test_ik_invalid_seed_and_target_rejected(self):
        with self.assertRaises(ValueError):self.model.ik('left',np.zeros((4,4)))
        with self.assertRaises(ValueError):self.model.ik('left',self.model.fk('left',self.model.home),seeds=[self.model.joint_upper+.01])

    def test_full_model_forward_matches_tcp_chain(self):
        frames=self.model.forward(self.model.home,self.model.home)
        self.assertEqual(set(frames),set(self.model.links))
        for side in ('left','right'):
            np.testing.assert_allclose(frames[f'{side}_fr3_side_suction_tcp'],self.model.fk(side,self.model.home),atol=1e-14,rtol=0)
