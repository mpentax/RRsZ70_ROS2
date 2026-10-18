import unittest
import numpy as np
from scipy.spatial.transform import Rotation as R
from analyze import rel,desc,prediction,ang

class NumericFixtures(unittest.TestCase):
 def test_mount_cancels(self):
  A=R.from_euler('xyz',[27,-14,81],degrees=True).as_matrix();M=R.from_euler('xyz',[17,48,-22],degrees=True).as_matrix();D=R.from_rotvec(np.array([-.2,0,0])).as_matrix()
  self.assertLess(ang(rel(A@M,D@A@M)@D.T),1e-10)
 def test_inverse_sign(self):
  A=R.from_euler('xyz',[28,2,4],degrees=True).as_matrix();B=R.from_rotvec([-.2,0,0]).as_matrix()@A
  np.testing.assert_allclose(rel(A,B).T,rel(B,A),atol=1e-12)
 def test_prediction_separates_conditional(self):
  D=R.from_euler('x',-12,degrees=True).as_matrix();p=prediction([-1,0,0],10,D,1)
  self.assertAlmostEqual(p['command_only_error_deg'],2,places=10);self.assertLess(p['conditional_axis_error_deg'],1e-10)
 def test_full_SO3_not_Euler_difference(self):
  A=R.from_euler('xyz',[15,40,170],degrees=True).as_matrix();D=R.from_rotvec([.08,.12,-.05]).as_matrix();d=desc(rel(A,D@A))
  self.assertAlmostEqual(d['angle_deg'],np.degrees(np.linalg.norm([.08,.12,-.05])),places=10)
if __name__=='__main__':unittest.main()
