"""Unit tests for supported-placement geometry and descent step selection."""
import cv2
import numpy as np
import pytest
from z70_manipulation import perception, planner


def scene(monkeypatch, gap=.020, elevated=False):
    from z70_manipulation import placement
    image = np.zeros((400, 600, 3), np.uint8)
    # Synthetic pinhole camera fixture: x=0.5, y=(300-u)*0.001, z=(350-v)*0.001.
    class Camera:
        width, height, h = 600, 400, 350
        def pixel_to_ray(self, u, v):
            return np.array([1., (300-u)*.002, -v*.002])
    monkeypatch.setattr(perception, 'front_camera', lambda: Camera())
    monkeypatch.setattr(perception, 'calibration', lambda: {'front_camera_x_m': 0.})
    support_top = .132 if elevated else .066
    ytop = round(350-support_top*1000)
    cv2.rectangle(image, (275, 284), (325, 350), (0, 160, 240), -1)
    if elevated:
        cv2.rectangle(image, (275, ytop), (325, 283), (200, 0, 200), -1)
    bottom = round(350-(support_top+gap)*1000)
    cv2.rectangle(image, (275, bottom-60), (325, bottom), (0, 200, 0), -1)
    monkeypatch.setattr(perception, 'front_objects', lambda _: [dict(color='yellow', position=[.5,0.,0.])])
    T = np.eye(4); T[:3, 3] = [.5, 0., support_top+gap+.03]
    jaw = dict(position=T[:3,3].tolist(), hand_pose_base=T.tolist(), uncertainty_bound_m=.015)
    monkeypatch.setattr(perception, 'front_jaw', lambda _: jaw)
    return placement, image


def test_measures_held_bottom_to_elevated_support_without_floor_projection(monkeypatch):
    placement, image = scene(monkeypatch, elevated=True)
    sample = placement.measure(image, 'green', 'purple', 'yellow', [.5,0.,0.], [0.,0.,0.])
    assert sample['gap_m'] == pytest.approx(.020, abs=.001)
    assert sample['held_bottom_m'][2] == pytest.approx(.152, abs=.001)
    assert sample['support_top_m'][2] == pytest.approx(.132, abs=.001)
    assert sample['lateral_error_m'] == pytest.approx([0.,0.], abs=.001)
    assert sample['uncertainty_m'] >= .015


def test_placement_keeps_pale_held_facets_in_contact_geometry(monkeypatch):
    placement, image = scene(monkeypatch)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv[204:265, 275:326] = (60, 95, 170)
    hsv[204:235, 275:326] = (60, 220, 170)
    image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    sample = placement.measure(image, 'green', 'yellow', 'yellow', [.5, 0., 0.], [0., 0., 0.])
    assert sample['gap_m'] == pytest.approx(.020, abs=.001)


def test_held_depth_uses_retained_object_transform_not_jaw_origin(monkeypatch):
    placement, image = scene(monkeypatch)
    baseline = placement.measure(image,'green','yellow','yellow',[.5,0.,0.],[0.,0.,0.])
    shifted = placement.measure(image,'green','yellow','yellow',[.5,0.,0.],[.010,0.,0.])
    assert shifted['held_center_m'][0]-baseline['held_center_m'][0] == pytest.approx(.010)


def test_rejects_same_hue_background_instead_of_assuming_object_depth(monkeypatch):
    placement,image=scene(monkeypatch)
    cv2.rectangle(image,(220,180),(380,283),(0,200,0),-1)
    with pytest.raises(LookupError,match='contour'):
        placement.measure(image,'green','yellow','yellow',[.5,0.,0.],[0.,0.,0.])


def test_observation_recovers_local_pose_not_stale_pregrip_angles():
    from z70_manipulation import placement
    current = [210., 75., 10.]
    observed = planner.hand.hand_pose(current)
    recovered = placement.observed_joints(observed)
    assert recovered['joints'] == pytest.approx(current)
    bad = observed.copy(); bad[1,3] += .05
    assert placement.observed_joints(bad) is None


def test_descent_never_substitutes_40mm_for_20mm_gap():
    from z70_manipulation import placement
    sample = dict(gap_m=.020, lateral_error_m=[0.,0.])
    assert placement.descent_step(sample, None) is None
    sample['gap_m'] = .060
    assert placement.descent_step(sample, None) == dict(method='app_arm_direction_move', params={'direction':2}, descent_m=.04)
    sample['gap_m'] = .030
    step = placement.descent_step(sample, [210.,75.,0.])
    assert step is not None and step['method'] == 'app_arm_move'
    assert .002 < step['descent_m'] <= .030
    assert set(step['params']['m']) <= {0, 1, 2} and step['params']['m'][2] == 0 and any(step['params']['m'][:2])
    sample['gap_m'] = .025
    coord_step = placement.descent_step(sample, [210.,75.,0.])
    assert coord_step is not None and coord_step['params']['m'] == [1, 1, 0]
    sample['gap_m'] = .022
    assert placement.descent_step(sample, [210.,75.,0.]) is None
    sample['gap_m'] = .050
    sample['uncertainty_m'] = .015
    assert placement.descent_step(sample, None) is None
