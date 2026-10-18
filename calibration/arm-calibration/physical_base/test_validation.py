import unittest,subprocess,sys,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
D=ROOT/'calibration/arm-calibration/20261015T061415Z-physical-base'
class ValidationTests(unittest.TestCase):
    def test_cross_view_holdout_is_genuinely_different_pixels_not_refitted_marker(self):
        p=subprocess.run([sys.executable,str(Path(__file__).with_name('validate.py')),str(D),'--quick'],text=True,capture_output=True)
        self.assertEqual(p.returncode,0,p.stderr)
        o=json.loads((D/'validation.json').read_text())
        v=o['cross_view_rigid_mount_holdout']
        self.assertNotEqual(v['training_front_file'],v['heldout_front_file'])
        self.assertTrue(v['heldout_predicted_from_pincher_board'])
        self.assertFalse(v['validates_physical_shoulder_or_L2'])
        self.assertGreater(v['point_errors_px']['rms'],0)
        self.assertIsNone(o['absolute_5mm_1deg_verdict']['certified_95percent_bound_m'])
        self.assertFalse(o['stationarity_brackets']['D']['joint_pair_accepted'])
        self.assertFalse(v['eligible_for_paired_calibration'])
if __name__=='__main__':unittest.main()
