import unittest,json,subprocess,sys,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
SESSION=ROOT/'calibration/arm-calibration/20261015T061415Z-physical-base'
class AnalysisTests(unittest.TestCase):
    def test_actual_capture_inventory_and_physical_unknowns_are_not_fabricated(self):
        proc=subprocess.run([sys.executable,str(Path(__file__).with_name('analyze.py')),str(SESSION),'--quick'],capture_output=True,text=True)
        self.assertEqual(proc.returncode,0,proc.stderr)
        fit=json.loads((SESSION/'fit.json').read_text())
        self.assertEqual(fit['counts']['retained_raw_frames'],len(list((SESSION/'raw').glob('*.jpg'))))
        self.assertGreaterEqual(fit['counts']['distinct_nominal_M3M4'],20)
        self.assertIsNone(fit['unidentifiable']['physical_L2_m'])
        self.assertIn('D_M3_85',fit['sweeps'])
        self.assertGreater(fit['sweeps']['D_M3_85']['pixel_fit']['point_rms_px'],0)
        self.assertEqual(fit['adoption'],'none')
if __name__=='__main__':unittest.main()
