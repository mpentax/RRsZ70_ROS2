import unittest,cv2,numpy as np
from scipy.spatial.transform import Rotation as Rot
from observe import X,branches,lens,delta
class RotationTests(unittest.TestCase):
 def test_left_relative_frame(self):
  a=Rot.from_euler('xyz',[.4,.5,-.1]).as_matrix();d=Rot.from_rotvec([.3,0,0]).as_matrix()
  np.testing.assert_allclose(delta(a,d@a),[.3,0,0],atol=1e-12)
 def test_ippe_square_corner_order_both_branches(self):
  K,d=lens('front');r=np.array([.2,-.1,.3]);t=np.array([.01,-.01,.25]);p=cv2.projectPoints(X,r,t,K,d)[0].reshape(4,2);bs=branches(p,K,d)
  self.assertEqual(len(bs),2);self.assertTrue(all(b['positive_depth'] for b in bs));self.assertLess(min(b['rms_px'] for b in bs),1e-8)
  self.assertLess(min(np.linalg.norm(delta(cv2.Rodrigues(r)[0],np.array(b['R']))) for b in bs),1e-8)
 def test_wrap_not_euler_subtraction(self):
  a=Rot.from_euler('z',179,degrees=True).as_matrix();b=Rot.from_euler('z',-179,degrees=True).as_matrix()
  self.assertAlmostEqual(np.degrees(np.linalg.norm(delta(a,b))),2)
 def test_rotation_chain_holdout(self):
  R0=Rot.from_euler('xyz',[.4,.2,.1]).as_matrix();D4=Rot.from_rotvec([.2,0,0]).as_matrix();D5=Rot.from_rotvec([0,0,.7]).as_matrix();A=D5@R0;B0=D4@R0;B=D4@D5@R0
  np.testing.assert_allclose((B0@R0.T)@A,B,atol=1e-12)
if __name__=='__main__':unittest.main()
