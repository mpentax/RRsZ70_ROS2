import math
import cv2
import numpy as np
import pytest
from z70_odometry.floor_vo import FloorCamera, render_floor_view
from z70_odometry.mosaic_node import FloorMosaic

CAM = FloorCamera.load()


def test_rotated_translated_views_align_in_odom():
    rng = np.random.default_rng(7)
    coarse = rng.integers(30, 225, (CAM.H // 16, CAM.W // 16, 3), dtype=np.uint8)
    img = cv2.resize(coarse, (CAM.W, CAM.H), interpolation=cv2.INTER_CUBIC)
    mosaic = FloorMosaic(CAM, resolution=.005, size_m=3, max_range=1.2, camera_x_m=.16)
    mosaic.update(img, (0, 0, 0))
    x, y, th = .03, .01, math.radians(8)
    tc = (x + .16 * (math.cos(th) - 1), y + .16 * math.sin(th), th)
    moved = render_floor_view(img, CAM, tc)
    stats = mosaic.update(moved, (x, y, th))
    assert stats['overlap_cells'] > 500
    assert stats['overlap_ncc'] > .95
    assert np.count_nonzero(mosaic.count) > 1000
    assert mosaic.image().dtype == np.uint8
    wrong = FloorMosaic(CAM, resolution=.005, size_m=3, max_range=1.2, camera_x_m=.16)
    wrong.update(img, (0, 0, 0))
    misaligned = wrong.update(moved, (0, 0, 0))
    assert misaligned['overlap_ncc'] < .9


def test_empty_view_and_outside_map_do_not_paint():
    mosaic = FloorMosaic(CAM)
    blank = np.zeros((CAM.H, CAM.W, 3), np.uint8)
    assert mosaic.update(blank, (0, 0, 0))['total_cells'] == 0
    assert mosaic.update(blank + 128, (20, 20, 0))['total_cells'] == 0
    with pytest.raises(ValueError):
        mosaic.update(blank[:10], (0, 0, 0))
