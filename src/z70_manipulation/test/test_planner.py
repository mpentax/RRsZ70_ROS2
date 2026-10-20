import math
import cv2
import numpy as np
import pytest
from z70_description import pincher_geometry as hand
from z70_manipulation import perception, placement, planner as p


def test_firmware_endpoint_is_separate_from_cad_hand_and_camera():
    a1, a2 = map(math.radians, (120, -15))
    endpoint = (.0475 + .1775*math.cos(a1) + .205*math.cos(a2),
                .14 + .1775*math.sin(a1) + .205*math.sin(a2))
    assert p.vendor_endpoint((150, 45)) == pytest.approx(endpoint)
    T = hand.hand_pose((150, 45))
    assert not np.allclose(T[[0, 2], 3], endpoint)
    C, R = p.camera_pose((150, 45))
    optical = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    assert C == pytest.approx(T[:3, 3] + T[:3, :3] @ p.CAD_MOUNT[0])
    assert R == pytest.approx(T[:3, :3] @ hand.rpy(p.CAD_MOUNT[1]) @ optical)
    assert p.vendor_check(240, 130, 0) == 'FAULT_PROJECTION'
    assert not p.safe((240, 130, 0, 90))


def test_default_grid_grasp_targets_front_measured_region():
    assert p.GRASP==(250,95)
    assert p.tip_height(p.GRASP) == pytest.approx(.04158389094665366,abs=1e-10)
    assert p.grasp_point()[:2] == pytest.approx(p.operational_point((240,95))[:2])
    assert p.vendor_check(*p.GRASP, 0) == 'SUCCESS'
    assert p.body_floor_bound(p.GRASP) < p.MIN_TIP
    assert not p.safe(p.GRASP)


def test_pincher_camera_follows_measured_M5_rotation():
    C0,R0=p.camera_pose((240,105,0))
    C40,R40=p.camera_pose((240,105,40))
    angle=math.degrees(math.acos(np.clip((np.trace(R40@R0.T)-1)/2,-1,1)))
    assert angle == pytest.approx(40.,abs=1e-6)
    assert np.linalg.norm(C40-C0) == pytest.approx(.009050553550760482,abs=1e-6)
    assert p.camera_pose((150,45,0))[0][2] == pytest.approx(.2337875037399874,abs=1e-9)
    axis=np.array([math.cos(math.radians(-45)),0,math.sin(math.radians(-45))])
    cross=np.cross(R0[:,0],R40[:,0])
    assert np.dot(cross,axis)>0


def test_metric_closed_jaw_height_uses_owner_registered_closing_point():
    cfg=p._CALIBRATION
    expected=np.array(cfg['marker_effective_from_elbow_m'])+np.array(cfg['marker_rotation_link2'])@cfg['jaw_region_in_marker_m']
    assert cfg['effective_jaw_from_elbow_m']==pytest.approx(expected,abs=1e-12)
    assert p._CALIBRATION['jaw_z_correction_m']==0.
    assert p.tip_height((250,115)) == pytest.approx(.07953791717046343,abs=1e-10)
    assert p.tip_height((240,105)) == pytest.approx(.10802715944384225,abs=1e-10)
    assert p.tip_height((230,95)) == pytest.approx(.13401118896525022,abs=1e-10)


def test_approach_places_target_on_arm_plane():
    grasp=p.grasp_point(); plan=p.approach([.9,.20,0],grasp)
    th=plan['yaw']; d=plan['forward']
    x=math.cos(th)*.9+math.sin(th)*.20-d
    y=-math.sin(th)*.9+math.cos(th)*.20
    assert [x,y]==pytest.approx(grasp[:2],abs=1e-8)
    assert .25<=p.vendor_endpoint(p.GRASP)[0]<=p.REACH_LIMIT-.003
    aligned=p.approach(grasp,grasp)
    assert abs(aligned['yaw'])<1e-9 and abs(aligned['forward'])<1e-9


def test_bfs_safe_grid_and_negative_code_two():
    goal=tuple(p.grasp_goal(p.grasp_point(),(190,65))['state'])
    plan=p.joint_plan((150,45),goal)
    state=(150,45)
    assert plan
    for step in plan:
        assert set(step['m'])<={0,1,2}
        assert 1 <= sum(code!=0 for code in step['m']) <= 2
        for a,b in zip(state,step['state']): assert abs(a-b) in (0,10)
        assert p.safe_edge(state,tuple(step['state']))
        state=tuple(step['state'])
    assert any(sum(code!=0 for code in step['m']) == 2 for step in plan)
    assert state==goal
    reverse=p.joint_plan(goal,(150,45))
    assert any(2 in step['m'] for step in reverse)
    assert not p.safe((270,145))
    assert not p.safe((240,35))
    with pytest.raises(ValueError): p.joint_plan((150,45),(270,145))


def test_joint_path_floor_envelope_rejects_low_shortcut():
    assert p.tip_floor_bound((260,95))<p.MIN_TIP
    assert not p.safe_edge((250,95),(260,95))
    assert p.tip_floor_bound(p.GRASP)>=p.MIN_TIP
    assert p.body_floor_bound(p.GRASP)<p.MIN_TIP
    goal=tuple(p.grasp_goal(p.grasp_point(),(190,65))['state'])
    path=p.joint_plan((230,95),goal)
    assert path and all(p.safe_edge(a,b) for a,b in zip([(230,95)]+[tuple(s['state']) for s in path], [tuple(s['state']) for s in path]))


def test_stack_tier_plans_cover_three_dodecahedron_tiers():
    tiers = [p.stack_tier_plan(t) for t in (1, 2, 3)]
    assert [t['tier'] for t in tiers] == [1, 2, 3]
    assert [t['target_equator_z_m'] for t in tiers] == [0.033, 0.099, 0.165]
    assert [t['tower_top_below_z_m'] for t in tiers] == [0.0, 0.066, 0.132]
    assert [t['lift_steps'] for t in tiers] == [2, 2, 3]
    for t in tiers:
        assert p.safe(tuple(t['placement_state']))
        assert abs(t['operational_point_m'][2] - t['target_equator_z_m']) <= 0.020
    with pytest.raises(ValueError):
        p.stack_tier_plan(4)


def test_floor_safety_checks_the_whole_open_hand_and_roll():
    candidates = [(a, b, roll, 90) for a in range(180, 271, 5) for b in range(35, 171, 5) for roll in (0, 40, 90)]
    examples = [s for s in candidates if p.tip_floor_bound(s) >= p.MIN_TIP and
                hand.lowest(hand.world_bodies(s)) - hand.CONFIG['uncertainty_m'] < p.MIN_TIP]
    assert examples
    assert not any(p.safe(s) for s in examples)
    with pytest.raises(ValueError):
        hand.world_bodies([250, 95, float('nan'), 90])
    with pytest.raises(ValueError):
        hand.world_bodies([250, 95, 0, 91])


def test_new_anatomy_routes_to_operational_equator_not_unsafe_historical_preset():
    assert p.GRASP == (250, 95)
    plan = p.grasp_goal([.43, 0, 0], (190, 65))
    state = tuple(plan['state'])
    assert p.safe(state)
    assert abs(p.operational_point(state)[2] - .033) <= .012
    assert plan['steps']


def test_operational_query_exposes_frames_regions_and_opening_not_legacy_tcp():
    gaps = []
    for m6 in (0, 45, 90):
        detail = hand.articulated_geometry((240, 95, 40, m6))
        gaps.append(detail['terminal_gap_m'])
        assert set(detail['finger_frames']) == {'finger_left', 'finger_right'}
        assert {b.name for b in detail['pad_regions']} == {'finger_left', 'finger_right'}
        assert detail['finger_frames']['finger_left'] == pytest.approx(hand.hand_pose((240, 95, 40, m6)) @ hand.finger_frame(1, m6))
    assert gaps[0] > .01 and gaps[0] < gaps[1] < gaps[2]
    body = hand.world_bodies((190, 65))[0]
    assert not p.safe((190, 65), [(body.center - .001, body.center + .001)])
    assert not p.safe_edge((190, 65), (190, 75), [(body.center - .001, body.center + .001)])
    assert p.safe((190, 65), [([8, 8, 8], [9, 9, 9])])
    for invalid in (float('nan'), -1, 91):
        with pytest.raises(ValueError):
            hand.local_bodies(invalid)
    with pytest.raises(ValueError, match='M3/M4'):
        p.joint_plan((190, 65, 0, 90), (190, 65, 0, 45))


def test_four_angle_edges_include_m6_and_reject_mismatched_states():
    assert p.safe_edge((190, 65, 0, 90), (190, 65, 40, 45))
    with pytest.raises(ValueError):
        p.safe_edge((190, 65), (190, 65, 0, 90))


def _placement_scene(monkeypatch, gap=.020, elevated=False):
    image = np.zeros((400, 600, 3), np.uint8)
    class Camera:
        width, height, h = 600, 400, 350
        def pixel_to_ray(self, u, v):
            return np.array([1., (300 - u) * .002, -v * .002])
    monkeypatch.setattr(perception, 'front_camera', lambda: Camera())
    monkeypatch.setattr(perception, 'calibration', lambda: {'front_camera_x_m': 0.})
    support_top = .132 if elevated else .066
    ytop = round(350 - support_top * 1000)
    cv2.rectangle(image, (275, 284), (325, 350), (0, 160, 240), -1)
    if elevated:
        cv2.rectangle(image, (275, ytop), (325, 283), (200, 0, 200), -1)
    bottom = round(350 - (support_top + gap) * 1000)
    cv2.rectangle(image, (275, bottom - 60), (325, bottom), (0, 200, 0), -1)
    monkeypatch.setattr(perception, 'front_objects', lambda _: [dict(color='yellow', position=[.5, 0., 0.])])
    T = np.eye(4); T[:3, 3] = [.5, 0., support_top + gap + .03]
    jaw = dict(position=T[:3, 3].tolist(), hand_pose_base=T.tolist(), uncertainty_bound_m=.015)
    monkeypatch.setattr(perception, 'front_jaw', lambda _: jaw)
    return image


def test_measures_held_bottom_to_elevated_support_without_floor_projection(monkeypatch):
    image = _placement_scene(monkeypatch, elevated=True)
    sample = placement.measure(image, 'green', 'purple', 'yellow', [.5, 0., 0.], [0., 0., 0.])
    assert sample['gap_m'] == pytest.approx(.020, abs=.001)
    assert sample['held_bottom_m'][2] == pytest.approx(.152, abs=.001)
    assert sample['support_top_m'][2] == pytest.approx(.132, abs=.001)
    assert sample['lateral_error_m'] == pytest.approx([0., 0.], abs=.001)
    assert sample['uncertainty_m'] >= .015


def test_placement_keeps_pale_held_facets_in_contact_geometry(monkeypatch):
    image = _placement_scene(monkeypatch)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv[204:265, 275:326] = (60, 95, 170)
    hsv[204:235, 275:326] = (60, 220, 170)
    image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    sample = placement.measure(image, 'green', 'yellow', 'yellow', [.5, 0., 0.], [0., 0., 0.])
    assert sample['gap_m'] == pytest.approx(.020, abs=.001)


def test_held_depth_uses_retained_object_transform_not_jaw_origin(monkeypatch):
    image = _placement_scene(monkeypatch)
    baseline = placement.measure(image, 'green', 'yellow', 'yellow', [.5, 0., 0.], [0., 0., 0.])
    shifted = placement.measure(image, 'green', 'yellow', 'yellow', [.5, 0., 0.], [.010, 0., 0.])
    assert shifted['held_center_m'][0] - baseline['held_center_m'][0] == pytest.approx(.010)


def test_rejects_same_hue_background_instead_of_assuming_object_depth(monkeypatch):
    image = _placement_scene(monkeypatch)
    cv2.rectangle(image, (220, 180), (380, 283), (0, 200, 0), -1)
    with pytest.raises(LookupError, match='contour'):
        placement.measure(image, 'green', 'yellow', 'yellow', [.5, 0., 0.], [0., 0., 0.])


def test_observation_recovers_local_pose_not_stale_pregrip_angles():
    current = [210., 75., 10.]
    observed = hand.hand_pose(current)
    recovered = placement.observed_joints(observed)
    assert recovered['joints'] == pytest.approx(current)
    bad = observed.copy(); bad[1, 3] += .05
    assert placement.observed_joints(bad) is None


def test_descent_never_substitutes_40mm_for_20mm_gap():
    sample = dict(gap_m=.020, lateral_error_m=[0., 0.])
    assert placement.descent_step(sample, None) is None
    sample['gap_m'] = .060
    assert placement.descent_step(sample, None) == dict(method='app_arm_direction_move', params={'direction': 2}, descent_m=.04)
    sample['gap_m'] = .030
    step = placement.descent_step(sample, [210., 75., 0.])
    assert step is not None and step['method'] == 'app_arm_move'
    assert .002 < step['descent_m'] <= .030
    assert set(step['params']['m']) <= {0, 1, 2} and step['params']['m'][2] == 0 and any(step['params']['m'][:2])
    sample['gap_m'] = .025
    coord_step = placement.descent_step(sample, [210., 75., 0.])
    assert coord_step is not None and coord_step['params']['m'] == [1, 1, 0]
    sample['gap_m'] = .022
    assert placement.descent_step(sample, [210., 75., 0.]) is None
    sample['gap_m'] = .050
    sample['uncertainty_m'] = .015
    assert placement.descent_step(sample, None) is None
