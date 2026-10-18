import unittest
import numpy as np
from scipy.spatial.transform import Rotation
import geometry

class AxisTests(unittest.TestCase):
    def test_rigid_sweep_finds_axis_line_not_mount_station(self):
        n=np.array([.99,.1,-.02]);n/=np.linalg.norm(n)
        c=np.array([-.02,-.06,-.13]);c-=n*(c@n)
        R0=Rotation.from_euler('xyz',[.7,-.2,.4]).as_matrix()
        p0=np.array([.03,-.15,.23])
        poses=[]
        for a in [0,.17,.35,.53,.72]:
            Q=Rotation.from_rotvec(n*a).as_matrix()
            T=np.eye(4);T[:3,:3]=Q@R0;T[:3,3]=c+Q@(p0-c);poses.append(T)
        result=geometry.fit_axis(poses)
        self.assertLess(np.linalg.norm(np.cross(result['direction'],n)),1e-8)
        np.testing.assert_allclose(result['closest_point'],c,atol=1e-8)
        self.assertLess(result['rms_m'],1e-9)
        self.assertTrue(result['axial_station_unobservable'])

    def test_pixel_axis_fit_uses_real_projective_geometry_and_latent_angles(self):
        import cv2
        n=np.array([1.,0,0]);c=np.array([0,.06,-.13]);C0=np.array([.015,-.03,.4])
        R0=Rotation.from_euler('xyz',[.35,.1,.05]).as_matrix()
        X=np.array([[x,y,0] for x in np.linspace(-.08,.08,6) for y in np.linspace(-.10,.10,7)])
        K=np.array([[840.,0,800],[0,840,450],[0,0,1.]])
        obs=[]
        for angle in [0,.14,.33,.51,.69]:
            Q=Rotation.from_rotvec(n*angle).as_matrix();R=Q@R0;C=c+Q@(C0-c)
            T=np.eye(4);T[:3,:3]=R;T[:3,3]=C
            px=cv2.projectPoints((X-C)@R,np.zeros(3),np.zeros(3),K,np.zeros(5))[0].reshape(-1,2)
            obs.append({'obj':X,'px':px,'T':T})
        fit=geometry.fit_pixel_axis(obs,K,np.zeros(5))
        self.assertLess(fit['point_rms_px'],1e-5)
        np.testing.assert_allclose(fit['closest_point'],c,atol=1e-5)
        P=geometry.predict_pose(fit,.69)
        np.testing.assert_allclose(P,obs[-1]['T'],atol=1e-5)
        self.assertEqual(len(fit['latent_angles_rad']),5)

    def test_translation_circle_reconstructs_same_line_without_pose_orientation(self):
        n=np.array([1.,0,0]);c=np.array([.09,.06,-.13]);p0=np.array([.09,-.08,.20])
        pts=[c+Rotation.from_rotvec(n*a).as_matrix()@(p0-c) for a in [0,.15,.32,.51,.72]]
        fit=geometry.fit_circle_axis(pts)
        np.testing.assert_allclose(fit['closest_point'],[0,.06,-.13],atol=1e-8)
        self.assertLess(fit['circle_rms_m'],1e-8)

if __name__=='__main__': unittest.main()
