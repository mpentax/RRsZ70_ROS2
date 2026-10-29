import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from rclpy.serialization import deserialize_message, serialize_message

from z70_interfaces.action import RunMission
from z70_interfaces.msg import CommandOutcome
from z70_manipulation.executor import SafetyStop, UncertainMotion
from z70_manipulation.goals import GoalContext, GoalSpec, simulate_goal
from z70_manipulation.mission import (
    ALLOWED_MISSION_OPS,
    MAX_MISSION_STEPS,
    MAX_OBSERVED_OBJECTS,
    MAX_STEPS_JSON_BYTES,
    MISSION_DESCRIPTION,
    MISSION_FIELDS,
    _encode_step_results,
    parse_mission_steps,
    run_mission,
    validate_mission_goal,
    validate_mission_payload,
)


def _mission_spec() -> GoalSpec:
    return GoalSpec(
        'mission',
        RunMission,
        '/z70/mission/run',
        lambda r: float(r.max_duration_s),
        validate_mission_goal,
        run_mission,
        needs_executor=lambda r: not bool(r.dry_run),
        navigation=True,
        fields=MISSION_FIELDS,
        description=MISSION_DESCRIPTION,
        validate_payload=validate_mission_payload,
    )


def test_run_mission_action_ros_interface_and_bounded_serialization():
    goal_fields = RunMission.Goal.get_fields_and_field_types()
    assert goal_fields == {
        'steps_json': 'string<4096>',
        'max_duration_s': 'double',
        'dry_run': 'boolean',
    }

    result_fields = RunMission.Result.get_fields_and_field_types()
    assert result_fields == {
        'outcome': 'z70_interfaces/CommandOutcome',
        'status_code': 'string<64>',
        'completed': 'boolean',
        'partial_completion': 'boolean',
        'dry_run_valid': 'boolean',
        'steps_completed': 'uint32',
        'steps_total': 'uint32',
        'failed_step': 'string<64>',
        'detail': 'string<128>',
        'step_results_json': 'string<4096>',
        'evidence_uri': 'string<256>',
    }

    feedback_fields = RunMission.Feedback.get_fields_and_field_types()
    assert feedback_fields == {
        'phase': 'uint8',
        'current_step': 'uint32',
        'current_op': 'string<64>',
        'reason': 'string<64>',
        'current_position_m': 'double[2]',
        'remaining_distance_m': 'double',
        'latest_outcome': 'z70_interfaces/CommandOutcome',
        'latest_observation': 'z70_interfaces/EvidenceStamp',
    }

    goal_msg = RunMission.Goal(
        steps_json=json.dumps([{'op': 'observe', 'camera': 'front', 'color': 'yellow'}]),
        max_duration_s=120.0,
        dry_run=False,
    )
    goal_rt = deserialize_message(serialize_message(goal_msg), RunMission.Goal)
    assert goal_rt.steps_json == goal_msg.steps_json
    assert goal_rt.max_duration_s == 120.0
    assert goal_rt.dry_run is False

    res_msg = RunMission.Result(
        outcome=CommandOutcome(error_code='NONE'),
        status_code='COMPLETED',
        completed=True,
        partial_completion=False,
        dry_run_valid=False,
        steps_completed=1,
        steps_total=1,
        failed_step='',
        detail='',
        step_results_json='[{"index":0,"op":"observe","ok":true,"status_code":"OBSERVED"}]',
        evidence_uri='mission-evidence',
    )
    res_rt = deserialize_message(serialize_message(res_msg), RunMission.Result)
    assert res_rt.status_code == 'COMPLETED'
    assert res_rt.completed is True
    assert res_rt.steps_completed == 1
    assert json.loads(res_rt.step_results_json)[0]['op'] == 'observe'

    fb_msg = RunMission.Feedback(
        phase=3,
        current_step=2,
        current_op='navigate',
        reason='MONITORING',
        current_position_m=[1.25, -0.5],
        remaining_distance_m=0.75,
    )
    fb_rt = deserialize_message(serialize_message(fb_msg), RunMission.Feedback)
    assert fb_rt.phase == 3
    assert fb_rt.current_step == 2
    assert fb_rt.current_op == 'navigate'
    assert list(fb_rt.current_position_m) == [1.25, -0.5]
    assert fb_rt.remaining_distance_m == pytest.approx(0.75)

    # Oversized step objects payload compacts cleanly below 4096 bytes and remains valid JSON.
    large_steps = [
        {
            'index': i,
            'op': 'observe',
            'camera': 'front',
            'color': 'yellow',
            'ok': True,
            'status_code': 'OBSERVED',
            'count': MAX_OBSERVED_OBJECTS,
            'objects': [
                {
                    'color': 'yellow_object_with_long_label',
                    'position': [0.3512, -0.1234, 0.0331],
                    'size_m': 0.066,
                    'area_px': 1234.5,
                    'camera': 'front',
                }
                for _ in range(MAX_OBSERVED_OBJECTS)
            ],
        }
        for i in range(MAX_MISSION_STEPS)
    ]
    encoded = _encode_step_results(large_steps)
    assert len(encoded.encode('utf-8')) <= MAX_STEPS_JSON_BYTES
    decoded = json.loads(encoded)
    assert len(decoded) == MAX_MISSION_STEPS
    assert all('objects' not in item for item in decoded)


def test_parse_and_validate_mission_plan_rejects_malformed_inputs_before_any_call():
    assert ALLOWED_MISSION_OPS == ('navigate_room', 'navigate', 'observe', 'pick')

    # Malformed top-level JSON / step counts / unknown ops / invalid step fields
    invalid_plans = [
        '',
        'not-json',
        '{}',
        '[]',
        json.dumps([{'op': 'observe'}] * (MAX_MISSION_STEPS + 1)),
        '[' + (' ' * MAX_STEPS_JSON_BYTES) + ']',
        json.dumps(['not-an-object']),
        json.dumps([{'op': 'stack', 'base_color': 'yellow', 'middle_color': 'blue', 'top_color': 'green'}]),
        json.dumps([{'op': 'rpc', 'method': 'app_arm_move'}]),
        json.dumps([{'op': 'navigate_room'}]),
        json.dumps([{'op': 'navigate_room', 'room': ''}]),
        json.dumps([{'op': 'navigate_room', 'room': '   '}]),
        json.dumps([{'op': 'navigate_room', 'room': True}]),
        json.dumps([{'op': 'navigate_room', 'room': 0}]),
        json.dumps([{'op': 'navigate_room', 'room': 1001}]),
        json.dumps([{'op': 'navigate_room', 'room': 'Kitchen', 'extra': 1}]),
        json.dumps([{'op': 'navigate_room', 'room': 'Kitchen', 'max_duration_s': 0.5}]),
        json.dumps([{'op': 'navigate_room', 'room': 'Kitchen', 'max_duration_s': 601.0}]),
        json.dumps([{'op': 'navigate_room', 'room': 'Kitchen', 'max_duration_s': True}]),
        json.dumps([{'op': 'navigate', 'target_x_m': 1.0}]),
        json.dumps([{'op': 'navigate', 'target_x_m': 101.0, 'target_y_m': 0.0}]),
        json.dumps([{'op': 'navigate', 'target_x_m': True, 'target_y_m': 0.0}]),
        json.dumps([{'op': 'navigate', 'target_x_m': 1.0, 'target_y_m': 0.0, 'extra': 2}]),
        json.dumps([{'op': 'navigate', 'target_x_m': 1.0, 'target_y_m': 0.0, 'max_duration_s': 0.2}]),
        json.dumps([{'op': 'observe', 'camera': 'rear'}]),
        json.dumps([{'op': 'observe', 'color': 'bad color!'}]),
        json.dumps([{'op': 'observe', 'require_visible': 1}]),
        json.dumps([{'op': 'observe', 'unknown': True}]),
        json.dumps([{'op': 'pick'}]),
        json.dumps([{'op': 'pick', 'color': 'yellow', 'dry_run': True}]),
        json.dumps([{'op': 'pick', 'color': 'bad/color'}]),
    ]
    for raw in invalid_plans:
        with pytest.raises(ValueError):
            parse_mission_steps(raw)

    # Color allowlist enforcement
    with pytest.raises(ValueError):
        parse_mission_steps(
            json.dumps([{'op': 'pick', 'color': 'magenta'}]),
            allowed_colors={'yellow': {}, 'purple': {}},
        )

    valid_json = json.dumps([
        {'op': 'navigate_room', 'room': ' Kitchen ', 'max_duration_s': 45},
        {'op': 'observe', 'camera': 'front', 'color': 'yellow'},
        {'op': 'pick', 'color': 'yellow'},
    ])
    assert validate_mission_payload({
        'steps_json': valid_json,
        'max_duration_s': 120.0,
        'dry_run': False,
    }, allowed_colors={'yellow': {}}) is True

    assert validate_mission_payload({
        'steps_json': valid_json,
        'max_duration_s': 0.5,
        'dry_run': False,
    }) is False
    assert validate_mission_payload({
        'steps_json': valid_json,
        'max_duration_s': 120.0,
        'dry_run': 'false',
    }) is False
    assert validate_mission_payload({
        'steps_json': valid_json,
        'max_duration_s': 120.0,
        'dry_run': False,
        'extra': 1,
    }) is False

    # Whole-plan validation happens before any GoalContext primitive is invoked.
    spec = _mission_spec()
    trace = []
    bad_later_step = RunMission.Goal(
        steps_json=json.dumps([
            {'op': 'navigate_room', 'room': 'Kitchen'},
            {'op': 'observe', 'camera': 'front', 'color': 'yellow'},
            {'op': 'pick', 'color': 'yellow'},
            {'op': 'navigate', 'target_x_m': 250.0, 'target_y_m': 0.0},
        ]),
        max_duration_s=120.0,
        dry_run=False,
    )
    ok, result = simulate_goal(spec, bad_later_step, _trace=trace)
    assert ok is False
    assert result.status_code == 'INVALID_INPUT'
    assert result.outcome.error_code == 'INVALID_INPUT'
    assert result.completed is False
    assert result.partial_completion is False
    assert result.steps_completed == 0
    assert result.steps_total == 0
    assert result.step_results_json == '[]'
    assert trace == []


def test_run_mission_multi_step_success_via_stateful_simulate_goal():
    spec = _mission_spec()
    trace = []
    steps = [
        {'op': 'navigate_room', 'room': 'Kitchen', 'max_duration_s': 45.0},
        {'op': 'observe', 'camera': 'front', 'color': 'yellow', 'require_visible': True},
        {'op': 'pick', 'color': 'yellow'},
        {'op': 'navigate', 'target_x_m': 0.25, 'target_y_m': -0.5, 'max_duration_s': 30.0},
    ]
    req = RunMission.Goal(
        steps_json=json.dumps(steps),
        max_duration_s=180.0,
        dry_run=False,
    )
    ok, result = simulate_goal(
        spec,
        req,
        _trace=trace,
        spatial_context={
            'current_room': 'Hall',
            'rooms': [
                {'segment_id': 2, 'name': 'Kitchen', 'reachable': True, 'safe_waypoint': [1.5, 0.75]},
            ],
        },
        objects=[
            {
                'color': 'yellow',
                'position': [0.35, 0.0, 0.03],
                'size_m': 0.066,
                'area_px': 410.0,
                'camera': 'front',
            },
        ],
    )
    assert ok is True
    assert result.completed is True
    assert result.partial_completion is False
    assert result.dry_run_valid is False
    assert result.status_code == 'COMPLETED'
    assert result.outcome.error_code == 'NONE'
    assert result.steps_completed == 4
    assert result.steps_total == 4
    assert result.failed_step == ''
    assert result.detail == ''

    step_results = json.loads(result.step_results_json)
    assert [s['op'] for s in step_results] == ['navigate_room', 'observe', 'pick', 'navigate']
    assert all(s['ok'] is True for s in step_results)
    assert step_results[0]['status_code'] == 'ARRIVED'
    assert step_results[0]['segment_id'] == 2
    assert step_results[0]['final_position_m'] == [1.5, 0.75]
    assert step_results[1]['status_code'] == 'OBSERVED'
    assert step_results[1]['count'] == 1
    assert step_results[1]['objects'][0]['color'] == 'yellow'
    assert step_results[2]['status_code'] == 'PICKED'
    assert step_results[2]['verified'] is True
    assert step_results[3]['status_code'] == 'ARRIVED'
    assert step_results[3]['final_position_m'] == [0.25, -0.5]

    primitives = [item['primitive'] for item in trace if item['primitive'] != 'feedback']
    assert primitives == ['navigate_room', 'navigate', 'objects', 'pick', 'navigate']
    feedbacks = [item for item in trace if item['primitive'] == 'feedback']
    assert [(f['current_step'], f['current_op']) for f in feedbacks] == [
        (0, 'navigate_room'),
        (1, 'observe'),
        (2, 'pick'),
        (3, 'navigate'),
    ]


def test_run_mission_partial_failure_stops_immediately_without_recovery_motion():
    spec = _mission_spec()
    trace = []
    steps = [
        {'op': 'navigate_room', 'room': 'Kitchen'},
        {'op': 'observe', 'camera': 'front', 'color': 'purple', 'require_visible': False},
        {'op': 'pick', 'color': 'purple'},
        {'op': 'navigate', 'target_x_m': 0.0, 'target_y_m': 0.0},
    ]
    req = RunMission.Goal(steps_json=json.dumps(steps), max_duration_s=120.0, dry_run=False)
    ok, result = simulate_goal(
        spec,
        req,
        _trace=trace,
        objects=[{'color': 'yellow', 'position': [0.35, 0.0, 0.03]}],
    )
    assert ok is False
    assert result.completed is False
    assert result.partial_completion is True
    assert result.steps_completed == 2
    assert result.steps_total == 4
    assert result.failed_step == 'pick'
    assert result.status_code == 'PICK_UNVERIFIED'
    assert result.outcome.error_code == 'PICK_UNVERIFIED'

    step_results = json.loads(result.step_results_json)
    assert len(step_results) == 3
    assert [s['ok'] for s in step_results] == [True, True, False]
    assert step_results[1]['status_code'] == 'EMPTY_OBSERVATION'
    assert step_results[2]['status_code'] == 'PICK_UNVERIFIED'
    primitives = [item['primitive'] for item in trace if item['primitive'] != 'feedback']
    assert primitives == ['navigate_room', 'navigate', 'objects', 'pick']
    assert 'release_and_stow' not in primitives
    assert 'move_arm' not in primitives
    assert 'rpc' not in primitives

    # UncertainMotion preserves uncertainty status without automatic recovery motion.
    trace_uncertain = []

    def fail_uncertain(_color, dry_run=False):
        raise UncertainMotion('dispatch unknown: app_arm_move')

    ok_u, res_u = simulate_goal(
        spec,
        RunMission.Goal(
            steps_json=json.dumps([
                {'op': 'navigate', 'target_x_m': 1.0, 'target_y_m': 0.5},
                {'op': 'pick', 'color': 'yellow'},
                {'op': 'navigate', 'target_x_m': 0.0, 'target_y_m': 0.0},
            ]),
            max_duration_s=90.0,
            dry_run=False,
        ),
        _trace=trace_uncertain,
        pick=fail_uncertain,
    )
    assert ok_u is False
    assert res_u.completed is False
    assert res_u.partial_completion is True
    assert res_u.steps_completed == 1
    assert res_u.steps_total == 3
    assert res_u.failed_step == 'pick'
    assert res_u.status_code == 'UNCERTAIN_MOTION'
    assert res_u.outcome.error_code == 'UNCERTAIN_MOTION'
    u_primitives = [item['primitive'] for item in trace_uncertain if item['primitive'] != 'feedback']
    assert u_primitives == ['navigate']


def test_run_mission_obeys_total_deadline_cancellation_and_hazard_across_steps():
    spec = _mission_spec()

    # 1. Total deadline caps navigation step duration and refuses <1s navigation remainder without dispatch.
    now = [100.0]
    nav_calls = []

    def fake_navigate(target_x_m, target_y_m, max_duration_s=60.0, dry_run=False):
        nav_calls.append((target_x_m, target_y_m, max_duration_s, dry_run))
        now[0] += 9.4
        return {
            'arrived': True,
            'dry_run_valid': False,
            'status_code': 'ARRIVED',
            'final_position_m': [float(target_x_m), float(target_y_m)],
            'position_error_m': 0.0,
        }

    def fake_objects(camera='front', color=None):
        now[0] += 0.2
        return [{'color': 'yellow', 'position': [0.35, 0.0, 0.03]}]

    ok_dl, res_dl = simulate_goal(
        spec,
        RunMission.Goal(
            steps_json=json.dumps([
                {'op': 'navigate', 'target_x_m': 1.0, 'target_y_m': 0.0, 'max_duration_s': 60.0},
                {'op': 'observe', 'camera': 'front', 'color': 'yellow'},
                {'op': 'navigate', 'target_x_m': 0.0, 'target_y_m': 0.0, 'max_duration_s': 60.0},
            ]),
            max_duration_s=10.0,
            dry_run=False,
        ),
        clock=lambda: now[0],
        navigate=fake_navigate,
        objects=fake_objects,
    )
    assert ok_dl is False
    assert res_dl.status_code == 'TIMED_OUT'
    assert res_dl.outcome.error_code == 'TIMED_OUT'
    assert res_dl.partial_completion is True
    assert res_dl.steps_completed == 2
    assert res_dl.steps_total == 3
    assert res_dl.failed_step == 'navigate'
    # Only step 0 dispatched navigate, capped to 10.0s; step 2 had 0.4s (<1.0s) left and did not dispatch.
    assert nav_calls == [(1.0, 0.0, 10.0, False)]

    # 2. Cancellation between steps stops immediately and prevents subsequent primitives.
    cancelled = [False]
    trace_cancel = []

    def nav_then_cancel(target_x_m, target_y_m, max_duration_s=60.0, dry_run=False):
        trace_cancel.append(('navigate', target_x_m, target_y_m))
        cancelled[0] = True
        return {'arrived': True, 'status_code': 'ARRIVED', 'final_position_m': [target_x_m, target_y_m]}

    ok_c, res_c = simulate_goal(
        spec,
        RunMission.Goal(
            steps_json=json.dumps([
                {'op': 'navigate', 'target_x_m': 1.0, 'target_y_m': 1.0},
                {'op': 'pick', 'color': 'yellow'},
                {'op': 'navigate', 'target_x_m': 0.0, 'target_y_m': 0.0},
            ]),
            max_duration_s=60.0,
            dry_run=False,
        ),
        navigate=nav_then_cancel,
        is_cancelled=lambda: cancelled[0],
        pick=lambda *_, **__: pytest.fail('pick must not execute after cancellation'),
    )
    assert ok_c is False
    assert res_c.status_code == 'CANCELLED'
    assert res_c.outcome.error_code == 'CANCELLED'
    assert res_c.completed is False
    assert res_c.partial_completion is False
    assert res_c.steps_completed == 0
    assert res_c.failed_step == 'navigate'
    assert trace_cancel == [('navigate', 1.0, 1.0)]

    # 3. Safety hazard mid-mission halts with SAFETY_STOP and no recovery motion.
    hazard_state = [None]

    def observe_then_hazard(camera='front', color=None):
        return [{'color': 'yellow', 'position': [0.35, 0.0, 0.03]}]

    def pick_raises_safety(_color, dry_run=False):
        hazard_state[0] = 'person/pet/obstacle or operator stop signal'
        raise SafetyStop(hazard_state[0])

    ok_h, res_h = simulate_goal(
        spec,
        RunMission.Goal(
            steps_json=json.dumps([
                {'op': 'observe', 'camera': 'front', 'color': 'yellow'},
                {'op': 'pick', 'color': 'yellow'},
                {'op': 'navigate', 'target_x_m': 0.0, 'target_y_m': 0.0},
            ]),
            max_duration_s=60.0,
            dry_run=False,
        ),
        objects=observe_then_hazard,
        pick=pick_raises_safety,
        hazard=lambda: hazard_state[0],
        navigate=lambda *_, **__: pytest.fail('navigate must not execute after safety stop'),
    )
    assert ok_h is False
    assert res_h.status_code == 'SAFETY_STOP'
    assert res_h.outcome.error_code == 'SAFETY_STOP'
    assert res_h.partial_completion is True
    assert res_h.steps_completed == 1
    assert res_h.failed_step == 'pick'


def test_run_mission_dry_run_validates_without_action_camera_select_or_map_refresh():
    spec = _mission_spec()
    trace = []
    steps = [
        {'op': 'navigate_room', 'room': 'Kitchen'},
        {'op': 'navigate', 'target_x_m': 1.2, 'target_y_m': -0.4},
        {'op': 'observe', 'camera': 'gripper', 'color': 'yellow'},
        {'op': 'pick', 'color': 'yellow'},
    ]
    req = RunMission.Goal(steps_json=json.dumps(steps), max_duration_s=120.0, dry_run=True)

    ok, res = simulate_goal(
        spec,
        req,
        _trace=trace,
        navigate_room=lambda *_, **__: pytest.fail('dry_run must not call navigate_room'),
        navigate=lambda *_, **__: pytest.fail('dry_run must not call navigate'),
        objects=lambda *_, **__: pytest.fail('dry_run must not capture objects'),
        pick=lambda *_, **__: pytest.fail('dry_run must not call pick'),
        select_camera=lambda *_, **__: pytest.fail('dry_run must not select camera'),
        map_snapshot=lambda *_, **__: pytest.fail('dry_run must not refresh map'),
    )
    assert ok is True
    assert res.dry_run_valid is True
    assert res.completed is False
    assert res.partial_completion is False
    assert res.status_code == 'DRY_RUN_VALID'
    assert res.outcome.error_code == 'NONE'
    assert res.steps_completed == 0
    assert res.steps_total == 4
    assert trace == []
    dry_steps = json.loads(res.step_results_json)
    assert [s['status_code'] for s in dry_steps] == ['DRY_RUN_VALID'] * 4

    # Robot not ready (e.g. low battery) fails dry_run honestly without side effects.
    ok_bad, res_bad = simulate_goal(
        spec,
        req,
        status=lambda: {'state': 3, 'error_code': 0, 'battery': 12.0, 'common_status': 0},
    )
    assert ok_bad is False
    assert res_bad.dry_run_valid is False
    assert res_bad.status_code == 'ROBOT_NOT_READY'
    assert res_bad.outcome.error_code == 'ROBOT_NOT_READY'


def test_run_mission_preserves_single_borrowed_goal_context_and_feedback_compatibility(tmp_path: Path):
    published_feedback = []
    session_calls = []

    class _FakeCommands:
        def __init__(self):
            self.latest_outcome = CommandOutcome(error_code='NONE', dispatch=1, response=1)
            self.cancelled = SimpleNamespace(is_set=lambda: False)

        def begin(self):
            session_calls.append('begin')

        def end(self):
            session_calls.append('end')

    commands = _FakeCommands()
    obs = SimpleNamespace(
        status=lambda: {'state': 3, 'error_code': 0, 'battery': 90.0, 'common_status': 0},
        arm_estimate=lambda: {'state': 'IN_ROBOT', 'joints': {}},
    )
    server = SimpleNamespace(
        observations=obs,
        thresholds={'yellow': {}, 'purple': {}},
        gains={},
        deadline=500.0,
        safety_stop=False,
        hazard=lambda: None,
        odom=lambda: {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
    )
    handle = SimpleNamespace(
        request=RunMission.Goal(
            steps_json=json.dumps([
                {'op': 'navigate', 'target_x_m': 1.0, 'target_y_m': 0.5, 'max_duration_s': 20.0},
                {'op': 'observe', 'camera': 'front', 'color': 'yellow'},
            ]),
            max_duration_s=30.0,
            dry_run=False,
        ),
        is_cancel_requested=False,
        publish_feedback=published_feedback.append,
    )
    client = SimpleNamespace(commands=commands)
    ctx = GoalContext(server, handle, RunMission, client, tmp_path / 'mission-run', create_executor=True)
    ctx.clock = lambda: 100.0
    orig_ctx_feedback = ctx.feedback
    orig_ex_feedback = ctx.executor.feedback

    seen_server_deadlines = []

    def fake_ctx_navigate(target_x_m, target_y_m, max_duration_s=60.0, dry_run=False):
        seen_server_deadlines.append(server.deadline)
        ctx.feedback(
            phase=3,
            reason='MONITORING',
            current_position_m=[0.5, 0.25],
            remaining_distance_m=0.55,
            extra_ignored_field='ignored',
        )
        return {
            'arrived': True,
            'status_code': 'ARRIVED',
            'final_position_m': [target_x_m, target_y_m],
            'position_error_m': 0.0,
        }

    ctx.navigate = fake_ctx_navigate
    ctx.objects = lambda camera='front', color=None: [
        {'color': 'yellow', 'position': [0.35, 0.0, 0.03], 'size_m': 0.066, 'area_px': 300.0, 'camera': camera}
    ]

    ok, result = run_mission(ctx, handle.request)
    assert ok is True
    assert result.completed is True
    assert result.evidence_uri == 'mission-run'
    # Parent session lifecycle belongs to PickServer._run_goal, never re-entered by run_mission.
    assert session_calls == []
    # Server deadline was tightened from 500.0 to 130.0 during execution and restored afterward.
    assert seen_server_deadlines == [130.0]
    assert server.deadline == 500.0
    assert ctx.feedback == orig_ctx_feedback
    assert ctx.executor.feedback == orig_ex_feedback

    assert len(published_feedback) == 3
    assert all(isinstance(fb, RunMission.Feedback) for fb in published_feedback)
    assert [(fb.current_step, fb.current_op, fb.reason) for fb in published_feedback] == [
        (0, 'navigate', 'STEP_START'),
        (0, 'navigate', 'MONITORING'),
        (1, 'observe', 'STEP_START'),
    ]
    assert list(published_feedback[1].current_position_m) == [0.5, 0.25]
    assert published_feedback[1].remaining_distance_m == pytest.approx(0.55)
