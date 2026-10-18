"""Offline trial fit from known camera offset and rendered real board images."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import cv2
import numpy as np
REPO=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(REPO/'ros2/src/z70_odometry'))
from z70_odometry.floor_vo import FloorCamera,render_floor_view

class TrialTest(unittest.TestCase):
    def test_fit_camera_offset_and_pulse_gains_from_images(self):
        path=REPO/'calibration/chassis/analyze.py'
        self.assertTrue(path.exists(),'chassis analysis script is missing')
        spec=importlib.util.spec_from_file_location('chassis_analyze',path)
        module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        cam=FloorCamera.load(REPO/'calibration/front-camera/front_camera.json')
        img=cv2.imread(str(REPO/'calibration/front-camera/raw/frame-3-telegram.jpg'))
        with tempfile.TemporaryDirectory(dir=REPO/'calibration/chassis') as directory:
            directory=Path(directory); cv2.imwrite(str(directory/'before.png'),img)
            rows=[]; c=.23; yaw_gain=.6; forward_gain=.55
            for i,(velocity,omega,ms) in enumerate([(0,.5,400),(0,-.5,400),(0,.5,600),(0,-.5,600),
                                                    (0,.5,800),(0,-.5,800),(.1,0,400),(.1,0,600),(.1,0,800)]):
                th=omega*ms/1000*yaw_gain; dx=velocity*ms/1000*forward_gain
                tc=(dx+c*(math.cos(th)-1),c*math.sin(th),th)
                cv2.imwrite(str(directory/f'after-{i}.png'),render_floor_view(img,cam,tc))
                rows.append({'id':str(i),'before':'before.png','after':f'after-{i}.png',
                             'velocity':velocity,'omega':omega,'duration_ms':ms})
            manifest=directory/'pairs.json'; manifest.write_text(json.dumps(rows))
            result=module.analyze(manifest,REPO/'calibration/front-camera/front_camera.json')
            cv2.imwrite(str(directory/'blank.png'),np.zeros_like(img))
            rejection_manifest=directory/'with-blank.json'
            rejection_manifest.write_text(json.dumps(rows+[{'id':'blank','before':'blank.png','after':'blank.png',
                                                            'velocity':0,'omega':.5,'duration_ms':400}]))
            rejected_result=module.analyze(rejection_manifest,REPO/'calibration/front-camera/front_camera.json')
            self.assertEqual(rejected_result['accepted_pairs'],9)
            self.assertEqual(rejected_result['rejected_pairs'],1)
            self.assertEqual(rejected_result['rejected'][0]['id'],'blank')
            print(json.dumps(result['fit'],indent=2))
            self.assertEqual(result['accepted_pairs'],9)
            self.assertLess(abs(result['fit']['camera_x_m']-c),.002)
            self.assertLess(abs(result['fit']['yaw']['gain']-yaw_gain),.01)
            self.assertLess(abs(result['fit']['forward']['gain']-forward_gain),.01)
            self.assertLess(result['fit']['camera_rotation_residual_rms_m'],.001)
            # The CLI must emit the same measured fit, not just expose a library.
            import subprocess
            target=directory/'fit.json'
            subprocess.run([sys.executable,str(path),str(manifest),'--output',str(target)],check=True,capture_output=True,text=True)
            self.assertAlmostEqual(json.loads(target.read_text())['fit']['camera_x_m'],result['fit']['camera_x_m'])
            (REPO/'calibration/chassis/synthetic-results.json').write_text(json.dumps(
                {'truth':{'camera_x_m':c,'yaw_gain':yaw_gain,'forward_gain':forward_gain},
                 'accepted_pairs':result['accepted_pairs'],'fit':result['fit']},indent=2)+'\n')

if __name__=='__main__': unittest.main()
