"""Real retained images plus explicitly synthetic numerical solver tests."""
import unittest
from pathlib import Path
import numpy as np
import cv2
import calibrate

class DetectionTest(unittest.TestCase):
    def test_archived_board_without_lens_assumption(self):
        f=Path(__file__).resolve().parent.parent/'20261015T061415Z-physical-base/raw/064-D-M4-190-forward-g190-85-0.jpg'
        o=calibrate.detect_board(f)
        self.assertGreater(len(o['px']),20)
        self.assertEqual(o['size'],[1600,896])
        self.assertEqual(len(o['obj']),len(o['px']))
        self.assertTrue(o['homography_rms_px']<5)

class SolverTest(unittest.TestCase):
    def test_recovers_known_intrinsics_from_independent_plane_views(self):
        K=np.array([[880.,0,790],[0,865,420],[0,0,1.]])
        d=np.array([-.08,.025,0,0,0.])
        rng=np.random.default_rng(42);obs=[]
        for i in range(24):
            r=rng.uniform([-.6,-.6,-.7],[.6,.6,.7]);t=rng.uniform([-.12,-.1,.32],[.03,.02,.65])
            uv=cv2.projectPoints(calibrate.OBJ,r,t,K,d)[0].reshape(-1,2)
            obs.append({'obj':calibrate.OBJ.tolist(),'px':uv.tolist(),'size':[1600,896]})
        fit=calibrate.fit_model(obs,'k1k2')
        self.assertLess(np.max(np.abs(np.array(fit['K'])-K)),.2)
        self.assertLess(fit['rms_px'],.01)
        self.assertTrue(fit['success'])
        self.assertLess(calibrate.evaluate(obs,fit)['rms_px'],.01)
        diag=calibrate.conditioning(obs,fit)
        self.assertEqual(diag['rank'],6)
        self.assertTrue(np.isfinite(diag['condition_number']))
        metric=calibrate.metric_check(obs[0],fit)
        self.assertLess(metric['relative_pairwise_distance_rms'],.001)

if __name__=='__main__':unittest.main()
