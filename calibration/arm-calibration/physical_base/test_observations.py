import unittest
from pathlib import Path
import numpy as np
import cv2
import observations

ROOT=Path(__file__).resolve().parents[3]
class ObservationTests(unittest.TestCase):
    def test_existing_real_front_board_has_consistent_positive_depth_pose(self):
        p=ROOT/'calibration/arm-calibration/20261015T061415Z-physical-base/raw/000-front-setup.jpg'
        result=observations.detect(p,'front',floor_min_y=710)
        self.assertGreater(result['board']['accepted_corners'],10)
        self.assertGreater(result['board']['T_cam_board'][2][3],0)
        self.assertLess(result['board']['rms_px'],4)
        self.assertIsNone(result['marker'])
        self.assertEqual(len(result['board']['planar_pnp_candidates']),2)

    def test_noisy_real_board_retains_pose_and_quality_instead_of_disappearing(self):
        p=ROOT/'calibration/arm-calibration/20261015T061415Z-physical-base/raw/011-A-M4-forward-g200-65-0.jpg'
        result=observations.detect(p,'gripper')
        self.assertIsNotNone(result['board'])
        self.assertIn('quality_accepted',result['board'])
        self.assertGreater(result['board']['full_rms_px'],result['board']['rms_px'])

    def test_clipped_front_board_and_elevated_repeated_id_are_separated(self):
        p=ROOT/'calibration/arm-calibration/20261015T061415Z-physical-base/raw/B-front-reference-before.jpg'
        result=observations.detect(p,'front')
        self.assertIsNotNone(result['board'])
        self.assertIsNotNone(result['marker'])
        self.assertGreater(result['marker']['absolute_height_over_board_m'],.03)

if __name__=='__main__':unittest.main()
