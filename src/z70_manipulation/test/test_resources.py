"""Installed default models and adopted K/D consistency."""
import json
import numpy as np
import pytest
from z70_description.resources import config_path
from z70_manipulation import perception, planner


def test_default_front_works_without_checkout_or_cwd(monkeypatch,tmp_path):
    monkeypatch.chdir(tmp_path); monkeypatch.setenv('Z70_REPO','/missing/archive')
    perception.front_camera.cache_clear()
    cam=perception.front_camera()
    assert (cam.width,cam.height)==(1600,1200)
    pixel=cam.floor_to_pixel(400.,50.)
    point=cam.pixel_to_floor(*pixel)
    assert [point['forward_mm'],point['left_mm']]==pytest.approx([400.,50.],abs=1e-7)
    assert perception.calibration()['front_camera_x_m']>0


def test_fallback_lenses_match_active_resources():
    lens=perception.gripper_camera_info()
    assert np.array_equal(planner.GRIPPER_MATRIX,np.array(lens['camera_matrix']['data']).reshape(3,3))
    assert np.array_equal(planner.GRIPPER_D,np.array(lens['distortion_coefficients']['data']))
    front=json.loads(config_path('front_camera.json').read_text())
    import yaml
    info=yaml.safe_load(config_path('z70_front_camera_info.yaml').read_text())
    # Original front camera_info exports K to three decimal places and D to
    # six; preserve that exact export, not a relaxed geometry tolerance.
    expected=[front['f_px'],0.,front['cx'],0.,front['f_px'],front['cy'],0.,0.,1.]
    assert info['camera_matrix']['data']==[round(x,3) for x in expected]
    assert info['distortion_coefficients']['data']==[round(front['k1'],6),round(front['k2'],6),0.,0.,0.]

