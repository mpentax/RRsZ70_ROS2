"""Hand collision body floor clearance and operational grasp planning."""
import pytest
from z70_manipulation import planner as p


def test_floor_safety_checks_the_whole_open_hand_and_roll():
    from z70_description import pincher_geometry as g
    candidates = [(a, b, roll, 90) for a in range(180, 271, 5) for b in range(35, 171, 5) for roll in (0, 40, 90)]
    examples = [s for s in candidates if p.tip_floor_bound(s) >= p.MIN_TIP and
                g.lowest(g.world_bodies(s)) - g.CONFIG['uncertainty_m'] < p.MIN_TIP]
    assert examples
    assert not any(p.safe(s) for s in examples)
    with pytest.raises(ValueError):
        g.world_bodies([250, 95, float('nan'), 90])
    with pytest.raises(ValueError):
        g.world_bodies([250, 95, 0, 91])


def test_new_anatomy_routes_to_operational_equator_not_unsafe_historical_preset():
    assert p.GRASP == (250, 95)
    plan = p.grasp_goal([.43, 0, 0], (190, 65))
    state = tuple(plan['state'])
    assert p.safe(state)
    assert abs(p.operational_point(state)[2] - .033) <= .012
    assert plan['steps']


def test_operational_query_exposes_frames_regions_and_opening_not_legacy_tcp():
    from z70_description import pincher_geometry as g
    gaps = []
    for m6 in (0, 45, 90):
        detail = g.articulated_geometry((240, 95, 40, m6))
        gaps.append(detail['terminal_gap_m'])
        assert set(detail['finger_frames']) == {'finger_left', 'finger_right'}
        assert {b.name for b in detail['pad_regions']} == {'finger_left', 'finger_right'}
        assert detail['finger_frames']['finger_left'] == pytest.approx(g.hand_pose((240, 95, 40, m6)) @ g.finger_frame(1, m6))
    assert gaps[0] > .01 and gaps[0] < gaps[1] < gaps[2]
    body = g.world_bodies((190, 65))[0]
    assert not p.safe((190, 65), [(body.center - .001, body.center + .001)])
    assert not p.safe_edge((190, 65), (190, 75), [(body.center - .001, body.center + .001)])
    assert p.safe((190, 65), [([8, 8, 8], [9, 9, 9])])
    for invalid in (float('nan'), -1, 91):
        with pytest.raises(ValueError):
            g.local_bodies(invalid)
    with pytest.raises(ValueError, match='M3/M4'):
        p.joint_plan((190, 65, 0, 90), (190, 65, 0, 45))


def test_four_angle_edges_include_m6_and_reject_mismatched_states():
    assert p.safe_edge((190, 65, 0, 90), (190, 65, 40, 45))
    with pytest.raises(ValueError):
        p.safe_edge((190, 65), (190, 65, 0, 90))
