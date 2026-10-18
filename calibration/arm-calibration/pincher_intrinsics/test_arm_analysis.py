"""Numerical tests use explicitly mathematical fixtures, not hardware data."""
import unittest
import numpy as np
import arm_analysis as a

class GeometryTest(unittest.TestCase):
    def test_station_separation_not_remote_skew_shortest_distance(self):
        shoulder={'direction':[1.,0,0],'closest_point':[0,0,0]}
        elbow={'direction':[1.,.01,0],'closest_point':[0,.18,0]}
        result=a.station_separation(shoulder,elbow,[0,0,0])
        self.assertAlmostEqual(result['separation_m'],.18,places=4)
        self.assertIn('station',result['convention'])

    def test_registers_unknown_marker_mount_without_fk(self):
        import cv2
        from scipy.spatial.transform import Rotation
        K=np.array([[900.,0,800],[0,900,600],[0,0,1.]])
        M=np.eye(4);M[:3,:3]=Rotation.from_euler('xyz',[.2,.1,-.3]).as_matrix();M[:3,3]=[.02,.01,.08]
        poses=[];pixels=[]
        for i in range(7):
            T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',[i*.06,i*.04,-i*.03]).as_matrix();T[:3,3]=[-.1+i*.02,-.02,.4+i*.015]
            P=T@M;poses.append(T);pixels.append(cv2.projectPoints(a.MARKER,Rotation.from_matrix(P[:3,:3]).as_rotvec(),P[:3,3],K,None)[0].reshape(-1,2))
        result=a.fit_mount(poses,pixels,K,np.zeros(5),initial=np.eye(4))
        self.assertTrue(result['success'])
        self.assertLess(np.max(np.abs(np.array(result['T_G_M'])-M)),1e-5)
        self.assertLess(result['rms_px'],.001)

    def test_conditional_axis_distance_profile_minimum(self):
        from scipy.spatial.transform import Rotation
        def poses(center,point):
            out=[]
            for rad in [0,.15,.3,.45,.6]:
                Q=Rotation.from_rotvec([rad,0,0]).as_matrix();T=np.eye(4);T[:3,:3]=Q;T[:3,3]=center+Q@(point-center);out.append(T)
            return out
        S=poses(np.array([0.,0,0]),np.array([.02,.2,.1]));E=poses(np.array([0.,.18,0]),np.array([.02,.3,.1]))
        p=a.axis_distance_profile(S,E,[1.,0,0],[1.,0,0],[.16,.18,.2])
        self.assertEqual(min(p,key=lambda x:x['translation_rms_m'])['fixed_L1_m'],.18)
        self.assertLess(p[1]['translation_rms_m'],1e-6)

if __name__=='__main__':unittest.main()
