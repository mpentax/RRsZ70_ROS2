import json
import math
from pathlib import Path
import tempfile
import unittest
import analyze

class LiveAnalysisTests(unittest.TestCase):
    def test_live_primary_survives_all_rejected_pairs_and_unwinds_old_offset(self):
        rows=[]; x=.23; gain=.7
        motions=[(0,sign*.5,ms) for ms in [400,600,800]*2 for sign in [1,-1]]+[(sign*.1,0,ms) for ms in [400,600,800] for sign in [1,-1]]
        for i,(velocity,omega,ms) in enumerate(motions):
            p=dict(id=str(i),velocity=velocity,omega=omega,duration_ms=ms)
            theta=p['omega']*p['duration_ms']/1000*gain
            # Base odom published with guessed offset .16, not the true .23.
            d=p['velocity']*p['duration_ms']/1000*.55
            rows.append(dict(p,before=None,after=None,odom_before=dict(x_m=0,y_m=0,yaw_rad=0,camera_x_m=.16),odom_after=dict(x_m=d+(x-.16)*(math.cos(theta)-1),y_m=(x-.16)*math.sin(theta),yaw_rad=theta)))
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'pairs.json'; path.write_text(json.dumps(rows))
            result=analyze.analyze(path,analyze.REPO/'calibration/front-camera/front_camera.json')
            self.assertEqual(result['primary_source'],'live_ros_odom')
            self.assertEqual(len(result['pulses']),18)
            self.assertEqual(result['accepted_pairs'],0)
            self.assertAlmostEqual(result['fit']['camera_x_m'],x)
            self.assertAlmostEqual(result['fit']['yaw_positive']['gain'],gain)
            self.assertAlmostEqual(result['fit']['forward']['gain'],.55)
            self.assertEqual(result['verdict']['decision'],'ADOPT')
            analyze.write_report(result,Path(folder))
            self.assertTrue((Path(folder)/'RESULTS.md').exists())
    def test_empty_trial_has_a_dont_adopt_report(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'pairs.json'; path.write_text('[]')
            result=analyze.analyze(path,analyze.REPO/'calibration/front-camera/front_camera.json')
            self.assertEqual(result['verdict']['decision'],"DON'T ADOPT")

if __name__=='__main__': unittest.main()
