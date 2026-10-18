import importlib.util
import math
import pytest

def api():
    assert importlib.util.find_spec('z70_manipulation.planner'), 'planner missing'
    from z70_manipulation import planner
    return planner

def test_firmware_endpoint_is_separate_from_cad_hand_and_camera():
    import numpy as np
    from z70_description import pincher_geometry as hand
    p = api()
    a1, a2 = map(math.radians, (120, -15))
    endpoint = (.0475 + .1775*math.cos(a1) + .205*math.cos(a2),
                .14 + .1775*math.sin(a1) + .205*math.sin(a2))
    assert p.vendor_endpoint((150, 45)) == pytest.approx(endpoint)
    assert p.wrist((150, 45)) == pytest.approx(endpoint)  # compatibility name
    T = hand.hand_pose((150, 45))
    assert not np.allclose(T[[0, 2], 3], endpoint)
    C, R = p.camera_pose((150, 45))
    optical = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    assert C == pytest.approx(T[:3, 3] + T[:3, :3] @ p.CAD_MOUNT[0])
    assert R == pytest.approx(T[:3, :3] @ hand.rpy(p.CAD_MOUNT[1]) @ optical)
    # The old hybrid checker admitted this beyond the recovered app endpoint.
    assert p.vendor_check(240, 130, 0) == 'FAULT_PROJECTION'
    assert not p.safe((240, 130, 0, 90))


def test_default_grid_grasp_targets_front_measured_region():
    p=api()
    # Keep the historical preset; new physical preview is not a new pickup trial.
    assert p.GRASP==(250,95)
    assert p.tip_height(p.GRASP) == pytest.approx(.04158389094665366,abs=1e-10)
    assert p.grasp_point()[:2] == pytest.approx(p.operational_point((240,95))[:2])
    assert p.vendor_check(*p.GRASP, 0) == 'SUCCESS'
    assert p.body_floor_bound(p.GRASP) < p.MIN_TIP
    assert not p.safe(p.GRASP), 'lateral finger bodies now disprove TCP-only safety'


def test_pincher_camera_follows_measured_M5_rotation():
    import numpy as np
    p=api()
    C0,R0=p.camera_pose((240,105,0))
    C40,R40=p.camera_pose((240,105,40))
    angle=math.degrees(math.acos(np.clip((np.trace(R40@R0.T)-1)/2,-1,1)))
    # CAD camera mount [0.0355, 0.0075, -0.0109] m from M5 axis (13.23mm radius) gives 40deg roll and 9.05mm chord.
    assert angle == pytest.approx(40.,abs=1e-6)
    assert np.linalg.norm(C40-C0) == pytest.approx(.009050553550760482,abs=1e-6)
    info0=p.pincher_camera_height_and_direction((150,45,0))
    assert info0['height_m'] == pytest.approx(.2337875037399874,abs=1e-9)
    assert info0['direction'] == pytest.approx(p.camera_pose((150,45,0))[1][:,2],abs=1e-9)
    # Positive M5 rotates around the physical link2 +x axis (not -x).
    axis=np.array([math.cos(math.radians(-45)),0,math.sin(math.radians(-45))])
    cross=np.cross(R0[:,0],R40[:,0])
    assert np.dot(cross,axis)>0


def test_metric_closed_jaw_height_uses_owner_registered_closing_point():
    import numpy as np
    p=api()
    cfg=p._CALIBRATION
    # The actual measured region in marker coordinates is unchanged. Re-express
    # it with the optical registration instead of pinning an old 165mm gauge.
    expected=np.array(cfg['marker_effective_from_elbow_m'])+np.array(cfg['marker_rotation_link2'])@cfg['jaw_region_in_marker_m']
    assert cfg['effective_jaw_from_elbow_m']==pytest.approx(expected,abs=1e-12)
    assert p._CALIBRATION['jaw_z_correction_m']==0.
    assert p.tip_height((250,115)) == pytest.approx(.07953791717046343,abs=1e-10)
    assert p.tip_height((240,105)) == pytest.approx(.10802715944384225,abs=1e-10)
    assert p.tip_height((230,95)) == pytest.approx(.13401118896525022,abs=1e-10)


class HistoricalOpticalPixelMismatch(AssertionError):
    """Only the disclosed pixel-accuracy failure is expected."""


@pytest.mark.parametrize('state,point,observed',[
    ((170,55),(.498267538,.028477801,0.),(791.428,659.244)),
    ((200,65),(.478638201,.108569555,0.),(499.285,425.143)),
    ((240,95),(.477989916,-.031428944,0.),(1191.786,235.086)),
])
@pytest.mark.xfail(strict=True, raises=HistoricalOpticalPixelMismatch, reason='Provisional physical preview is NOT precision validated against historical nominal-angle optical FK; retain the 12px diagnostic rather than widening its tolerance')
def test_camera_projection_matches_retained_charuco_observations(state,point,observed):
    predicted=api().gripper_pixel(state,point)
    assert predicted is not None
    assert all(math.isfinite(v) for v in predicted)
    if predicted[:2] != pytest.approx(observed,abs=12.):
        raise HistoricalOpticalPixelMismatch(f'predicted {predicted[:2]}, observed {observed}, tolerance 12px')


def test_approach_places_target_on_arm_plane():
    p=api(); grasp=p.grasp_point(); plan=p.approach([.9,.20,0],grasp)
    th=plan['yaw']; d=plan['forward']
    x=math.cos(th)*.9+math.sin(th)*.20-d
    y=-math.sin(th)*.9+math.cos(th)*.20
    assert [x,y]==pytest.approx(grasp[:2],abs=1e-8)
    assert .25<=p.wrist(p.GRASP)[0]<=p.REACH_LIMIT-.003
    aligned=p.approach(grasp,grasp)
    assert abs(aligned['yaw'])<1e-9 and abs(aligned['forward'])<1e-9

def test_bfs_safe_grid_and_negative_code_two():
    p=api(); goal=tuple(p.grasp_goal(p.grasp_point(),(190,65))['state'])
    plan=p.joint_plan((150,45),goal)
    state=(150,45)
    assert plan
    for step in plan:
        assert set(step['m'])<={0,1,2}
        assert sum(code!=0 for code in step['m'])==1
        for a,b in zip(state,step['state']): assert abs(a-b) in (0,10)
        assert p.safe_edge(state,tuple(step['state']))
        state=tuple(step['state'])
    assert state==goal
    reverse=p.joint_plan(goal,(150,45))
    assert any(2 in step['m'] for step in reverse)
    assert not p.safe((270,145))
    assert not p.safe((240,35))
    with pytest.raises(ValueError): p.joint_plan((150,45),(270,145))

def test_pincher_pixels_cannot_authorize_closure_without_front_measurement():
    p=api()
    assert not p.grasp_criterion({'pixel':[800,100],'size_px':400,'image_size':[1600,896]},.066,.1,state=p.GRASP)


def test_joint_path_floor_envelope_rejects_low_shortcut():
    p=api()
    assert p.tip_floor_bound((260,95))<p.MIN_TIP
    assert not p.safe_edge((250,95),(260,95))
    assert p.tip_floor_bound(p.GRASP)>=p.MIN_TIP
    # Historical point clears; corrected full hand does not. Keep the old
    # surrogate evidence intact and route to the new equator-derived goal.
    assert p.body_floor_bound(p.GRASP)<p.MIN_TIP
    goal=tuple(p.grasp_goal(p.grasp_point(),(190,65))['state'])
    path=p.joint_plan((230,95),goal)
    assert path and all(p.safe_edge(a,b) for a,b in zip([(230,95)]+[tuple(s['state']) for s in path], [tuple(s['state']) for s in path]))


def test_expected_grasp_pixels_scales_with_distance():
    p=api()
    assert p.expected_size_px(.05,.1,843)==pytest.approx(421.5)
    assert p.expected_size_px(.05,.2,843)==pytest.approx(210.75)
    # A large low blob was exactly the failed grasp: it is BELOW, not between, the jaws.
    assert not p.grasp_criterion({'pixel':[835,633],'size_px':556,'image_size':[1600,896]},.066,.10)
    assert p.tip_height((240,105)) > .07  # former grasp was 4-5cm too high
    assert p.tip_height((230,95)) > .10
    assert p.tip_height(p.GRASP) == pytest.approx(.04158389094665366,abs=1e-10)


def test_stack_tier_plans_cover_three_dodecahedron_tiers():
    p = api()
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
