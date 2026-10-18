"""Mathematical projection fixtures below are NOT robot observations."""
import unittest
import importlib
import numpy as np
import cv2
from scipy.spatial.transform import Rotation


class SurveyRefitTests(unittest.TestCase):
    def test_height_constraint_refits_pixels_not_posthoc_translation(self):
        try:
            fit = importlib.import_module('survey_refit').fit_front
        except ModuleNotFoundError:
            self.fail('constrained pixel registration is not implemented')
        K = np.array([[850., 0, 800], [0, 850, 600], [0, 0, 1]])
        d = np.zeros(5)
        X = np.array([[x, y, 0] for x in np.linspace(0, .18, 8)
                      for y in np.linspace(0, .26, 10)])
        R = Rotation.from_euler('xyz', [155, 4, 2], degrees=True).as_matrix()
        C = np.array([.1, -.2, .06])
        t = -R @ C
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
        px = cv2.projectPoints(X, Rotation.from_matrix(R).as_rotvec(), t, K, d)[0].reshape(-1, 2)
        o = {'file': 'mathematical', 'board': {'obj': X.tolist(), 'px': px.tolist(),
             'T_cam_board': T.tolist(), 'quality_accepted': True}}
        exact = fit([o], K, d, height=.06)
        wrong = fit([o], K, d, height=.065)
        self.assertAlmostEqual(exact['height_m'], .06, places=10)
        self.assertLess(exact['point_errors_px']['rms'], 1e-6)
        self.assertGreater(wrong['point_errors_px']['rms'], exact['point_errors_px']['rms'] + .01)
        # A true constrained refit improves over simply changing height in the old pose.
        Cwrong = C.copy(); Cwrong[2] = .065
        naive = cv2.projectPoints(X, Rotation.from_matrix(R).as_rotvec(), -R @ Cwrong, K, d)[0].reshape(-1, 2)
        self.assertLess(wrong['point_errors_px']['rms'], np.sqrt(np.mean(np.sum((naive-px)**2, axis=1))))
        self.assertGreater(np.linalg.norm(np.array(wrong['T_F_W'])[:3, :3]-R), 1e-5)

    def test_provenance_key_changes_for_pixels_intrinsics_log_and_height(self):
        import survey_refit as s
        self.assertTrue(hasattr(s, 'provenance_key'), 'content-keyed provenance is not implemented')
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as td:
            p = Path(td)/'image.jpg'; p.write_bytes(b'original mathematical bytes')
            log = Path(td)/'log.jsonl'; log.write_text('original log')
            inputs = [p, log]
            K = np.eye(3); d = np.zeros(5)
            a = s.provenance_key(inputs, K, d, .06)
            self.assertNotEqual(a, s.provenance_key(inputs, K*2, d, .06))
            self.assertNotEqual(a, s.provenance_key(inputs, K, d, .061))
            p.write_bytes(b'different pixels same number of files')
            self.assertNotEqual(a, s.provenance_key(inputs, K, d, .06))
            p.write_bytes(b'original mathematical bytes'); log.write_text('new log')
            self.assertNotEqual(a, s.provenance_key(inputs, K, d, .06))

    def test_real_scenarios_preserve_archive_and_identical_holdouts(self):
        import survey_refit as s
        self.assertTrue(hasattr(s, 'run'), 'isolated scenario runner is not implemented')
        import tempfile
        from pathlib import Path
        session = Path(__file__).resolve().parents[1]/'20261015T061415Z-physical-base'
        before = s.file_hashes(p for p in session.rglob('*') if p.is_file())
        with tempfile.TemporaryDirectory() as td:
            result = s.run(session, Path(td), heights=(None, .06), bootstrap=0, lens=False)
            self.assertEqual(before, s.file_hashes(p for p in session.rglob('*') if p.is_file()))
            self.assertEqual(result['adoption'], 'none')
            # Match the archived front-optical closest-point station convention;
            # projecting board-origin closest points is not translation-invariant
            # when estimated lines are not exactly parallel.
            archived = __import__('json').loads((session/'validation.json').read_text())
            for key, item in result['position_circle_checks'].items():
                self.assertIn('by_scenario', item, 'comparison must retain front-optical station for every scenario')
                self.assertAlmostEqual(item['by_scenario']['free']['perpendicular_line_discrepancy_m'],
                    archived['position_only_circle_crosschecks'][key]['perpendicular_distance_from_pixel_screw_line_m'], places=7)
            for key, item in result['effective_L1_checks'].items():
                self.assertAlmostEqual(item['by_scenario']['free']['effective_parallel_axis_distance_m'],
                    archived['effective_upper_link_checks'][key]['effective_pitch_axis_perpendicular_distance_m'], places=7)
            self.assertFalse(result['cross_view_pair_eligible'])
            for block in ['A', 'B', 'D']:
                a = result['scenarios']['free']['front'][block]
                b = result['scenarios']['h60']['front'][block]
                self.assertEqual(a['holdout_file'], b['holdout_file'])
                self.assertNotIn(a['holdout_file'], a['training_files'])
                self.assertAlmostEqual(b['full_fit']['height_m'], .06, places=10)
                self.assertGreaterEqual(b['full_fit']['robust_cost']+1e-8, a['full_fit']['robust_cost'])
                self.assertGreater(b['full_fit']['point_errors_px']['n'], 20)
            self.assertEqual(result['scenarios']['free']['pincher_holdout_id'], result['scenarios']['h60']['pincher_holdout_id'])
        with self.assertRaises(ValueError):
            s.run(session, session/'must-not-write')

    def test_alternate_planar_seed_is_checked_without_using_holdout(self):
        import survey_refit as s
        import inspect, json
        from pathlib import Path
        self.assertIn('initial_T', inspect.signature(s.fit_front).parameters,
                      'planar alternative initialization is not supported')
        session = Path(__file__).resolve().parents[1]/'20261015T061415Z-physical-base'
        old = json.loads((session/'fit.json').read_text())
        by = {o['file']: o for o in json.loads((session/'observations.json').read_text())['frames']}
        train = [by[f] for f in old['front_registration']['D']['frames'][:-1]]
        alt = train[0]['board']['planar_pnp_candidates'][1]['T_cam_board']
        K, d = s.intrinsics('front')
        for h in [None, .06]:
            a = s.fit_front(train, K, d, h)
            b = s.fit_front(train, K, d, h, initial_T=np.array(alt))
            self.assertAlmostEqual(a['robust_cost'], b['robust_cost'], places=6)
            self.assertTrue(b['success'])


if __name__ == '__main__':
    unittest.main()
