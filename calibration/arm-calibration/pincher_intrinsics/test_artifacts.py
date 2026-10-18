"""Read-only integration checks of the actual retained acquisition artifacts."""
import unittest,json,hashlib
from pathlib import Path
from lens_report import group
D=Path(__file__).resolve().parent.parent/'20261015T080103Z-pincher-intrinsics-arm'

class RealArtifacts(unittest.TestCase):
    def test_all_originals_and_baselines_unchanged(self):
        manifest=json.loads((D/'manifest.json').read_text());self.assertTrue(manifest)
        for row in manifest:self.assertEqual(hashlib.sha256((D/row['file']).read_bytes()).hexdigest(),row['sha256'])
        report=json.loads((D/'validation/verification.json').read_text());self.assertEqual(report['originals_verified'],len(manifest));self.assertEqual(report['changed_baseline_files'],[])
    def test_wholepose_holdouts_and_converged_bootstrap(self):
        obs=json.loads((D/'intrinsics-observations.json').read_text());by={o['file']:o for o in obs};split=json.loads((D/'lens/splits.json').read_text());train={group(by[f]) for f in split['train']};test={group(by[f]) for f in split['heldout']}
        self.assertTrue(train and test);self.assertFalse(train&test)
        hold=json.loads((D/'lens/holdout.json').read_text());self.assertLess(hold['new']['rms_px'],3);self.assertLess(hold['new']['rms_px'],hold['old_on_identical_corners']['rms_px'])
        unc=json.loads((D/'lens/uncertainty.json').read_text());self.assertEqual(unc['attempted'],unc['successful']+len(unc['failed']));self.assertGreaterEqual(unc['successful'],200)
    def test_rigid_mount_holdouts_and_anatomical_nulls(self):
        fit=json.loads((D/'arm/marker-registration.json').read_text())['new'];self.assertTrue(fit['fit']['success']);self.assertTrue(fit['heldout']);self.assertTrue(all(v['independent_marker_pixels_heldout'] for v in fit['heldout']))
        g=json.loads((D/'validation/effective-second-assembly-and-gauges.json').read_text())
        for k in ['physical_L2_m','physical_M4_zero_deg','physical_M5_axial_station_m','exact_contact_pad_vector_m']:self.assertIsNone(g[k])
        self.assertEqual(g['gauge_jacobian']['rank'],1)
    def test_camera_identity_and_cleanup_not_filename_or_ack(self):
        logs=[json.loads(l) for l in (D/'log.jsonl').read_text().splitlines()];wrong_label=next(r for r in logs if r.get('file')=='raw/002-deployed-gripper.jpg');self.assertEqual(wrong_label['camera'],'front')
        end=json.loads((D/'final-state.json').read_text());self.assertEqual(end['status']['state'],3);self.assertEqual(end['grab_mode_decoded_common_status_bits10_11'],0);self.assertEqual(end['grab_status_decoded_common_status_bits6_9'],0);self.assertEqual(end['camera_consumers'],{'z70':[],'z70_jpeg':[]})

if __name__=='__main__':unittest.main()
