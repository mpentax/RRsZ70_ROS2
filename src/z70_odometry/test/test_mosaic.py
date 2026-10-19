import math
import sys
import unittest
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from z70_description.resources import config_path
from z70_odometry.floor_vo import FloorCamera, render_floor_view
try:
    from z70_odometry.mosaic import FloorMosaic
except ImportError:
    FloorMosaic = None


class MosaicTest(unittest.TestCase):
    def test_rotated_translated_views_align_in_odom(self):
        self.assertIsNotNone(FloorMosaic, 'metric floor mosaic is missing')
        cam = FloorCamera.load(config_path('front_camera.json'))
        rng = np.random.default_rng(7)
        coarse = rng.integers(30, 225, (cam.H // 16, cam.W // 16, 3), dtype=np.uint8)
        img = cv2.resize(coarse, (cam.W, cam.H), interpolation=cv2.INTER_CUBIC)
        mosaic = FloorMosaic(cam, resolution=.005, size_m=3, max_range=1.2, camera_x_m=.16)
        mosaic.update(img, (0, 0, 0))
        x, y, th = .03, .01, math.radians(8)
        tc = (x + .16 * (math.cos(th) - 1), y + .16 * math.sin(th), th)
        moved = render_floor_view(img, cam, tc)
        stats = mosaic.update(moved, (x, y, th))
        self.assertGreater(stats['overlap_cells'], 500)
        self.assertGreater(stats['overlap_ncc'], .95)
        self.assertGreater(np.count_nonzero(mosaic.count), 1000)
        self.assertEqual(mosaic.image().dtype, np.uint8)
        wrong = FloorMosaic(cam, resolution=.005, size_m=3, max_range=1.2, camera_x_m=.16)
        wrong.update(img, (0, 0, 0))
        misaligned = wrong.update(moved, (0, 0, 0))
        self.assertLess(misaligned['overlap_ncc'], .9)

    def test_empty_view_and_outside_map_do_not_paint(self):
        self.assertIsNotNone(FloorMosaic)
        cam = FloorCamera.load(config_path('front_camera.json'))
        mosaic = FloorMosaic(cam)
        blank = np.zeros((cam.H, cam.W, 3), np.uint8)
        self.assertEqual(mosaic.update(blank, (0, 0, 0))['total_cells'], 0)
        self.assertEqual(mosaic.update(blank + 128, (20, 20, 0))['total_cells'], 0)
        with self.assertRaises(ValueError):
            mosaic.update(blank[:10], (0, 0, 0))


if __name__ == '__main__':
    unittest.main()
