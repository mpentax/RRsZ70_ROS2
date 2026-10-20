"""Dashboard diagnostic status, telemetry publishers, and Foxglove layout integrity."""
import json
import math
from pathlib import Path
import threading
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.serialization import serialize_message, deserialize_message
from z70_description.arm_model import vendor_forward, vendor_inverse
from z70_driver import driver as bridge
from z70_driver.commands import Dispatcher, ZERO
from z70_driver.dashboard import ArmEstimator, Dashboard, decode_arm, decode_calibration, decode_command, decode_status
from z70_driver.driver import estimate_to_app
from z70_interfaces.msg import CalibrationState, CommandEvent, OdometryStatus

LAYOUT = json.loads((Path(__file__).resolve().parents[2] / 'z70_bringup' / 'foxglove' / 'z70-dashboard.json').read_text())
CONFIGS = LAYOUT['configById']


@pytest.fixture
def dashboard():
    rclpy.init()
    node = Node('dashboard_unit_test')
    try:
        yield Dashboard(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_dashboard_publishes_disconnected_and_unknown_not_healthy_defaults(dashboard):
    diagnostic = dashboard.message(now=100)
    names = {s.name: s for s in diagnostic.status}
    assert names['Z70/Connection'].level == DiagnosticStatus.STALE
    assert names['Z70/Arm estimate'].level == DiagnosticStatus.STALE
    assert names['Z70/Robot'].level == DiagnosticStatus.STALE
    assert '/z70/battery' in dashboard.publishers
    assert '/z70/calibration/camera_x_m' in dashboard.publishers
    dashboard.publish(now=100)
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.pose.pose.position.x = 1.25
    dashboard.on_odometry(odom)
    assert len(dashboard.path.poses) == 1
    assert dashboard.path.poses[0].pose.position.x == 1.25
    dashboard.state = {'connection': {'state': 'connected'}, 'telemetry': {'status': {
        'value': {'battery': 80, 'state': 39, 'error_code': 5}, 'observedAt': 99000, 'error': None}}}
    dashboard.state_received = 99
    names = {s.name: s for s in dashboard.message(now=100).status}
    assert names['Z70/Connection'].level == DiagnosticStatus.OK
    assert names['Z70/Robot'].level == DiagnosticStatus.ERROR
    assert names['Z70/Robot'].message == 'REMOTE_PICKUP / MAIN_BRUSH_JAMMED'
    arm_keys = {kv.key for kv in names['Z70/Arm estimate'].values}
    cmd_keys = {kv.key for kv in names['Z70/Last RC command'].values}
    assert 'source' not in arm_keys and 'visual' not in arm_keys
    assert 'physical_completion' not in cmd_keys and 'scope' not in cmd_keys
    assert {s.name: s for s in dashboard.message(now=110).status}['Z70/Robot'].level == DiagnosticStatus.STALE


def test_diagnostic_levels_survive_ros_cdr_serialization(dashboard):
    restored = deserialize_message(serialize_message(dashboard.message(now=100)), DiagnosticArray)
    for status in restored.status:
        assert ord(status.level) == 3


def test_dashboard_consumes_typed_calibration_odom_and_dispatch_without_polling(dashboard):
    cal = CalibrationState(sample_count=7, camera_x_known=True, camera_x_m=.28,
        camera_x_ready=True, yaw_positive_known=True, yaw_positive_gain=.67)
    cal.evidence.valid = True
    dashboard.on_calibration(deserialize_message(serialize_message(cal), CalibrationState))
    values = dashboard.values(100)[3]
    assert values['camera_x_m'] == .28 and values['samples'] == 7
    assert values['yaw_positive_gain'] == .67 and math.isnan(values['forward_gain'])
    odom = OdometryStatus(pair_accepted=False, rejection='INSUFFICIENT_FEATURES',
        active_camera_x_known=True, active_camera_x_m=.16)
    odom.evidence.valid = True
    odom.validity.value = 4
    dashboard.on_odom(odom)
    assert dashboard.odom['rejected'] == 1 and dashboard.odom['state'] == 'rejected'
    assert dashboard.values(100)[3]['active_camera_x_m'] == .16
    event = CommandEvent(operation='app_arm_move')
    event.outcome.requested_at.sec = 10
    event.outcome.dispatch = 2
    event.outcome.response = 2
    dashboard.on_command(event)
    assert dashboard.values(11)[2] == {'age_s': 1., 'method': 'app_arm_move', 'outcome': 'enum:2/enum:2'}
    cal.evidence.stale = True
    dashboard.on_calibration(cal)
    assert math.isnan(dashboard.values(100)[3]['camera_x_m'])


def test_decodes_real_service_envelope_without_confusing_connection_state():
    state = {'connection': {'state': 'connected'}, 'telemetry': {'status': {
        'value': {'battery': 97, 'state': 8, 'error_code': 0, 'in_cleaning': 0, 'in_returning': 0},
        'observedAt': 100000, 'error': None}}}
    decoded = decode_status(state, now=101)
    assert decoded['battery'] == 97
    assert decoded['state_name'] == 'CHARGING'
    assert decoded['error_name'] == 'NONE'
    assert decoded['charging'] is True
    assert decoded['cleaning'] is False
    assert decoded['connected'] is True
    assert decoded['status_age_s'] == 1
    assert decoded['status_valid'] is True


def test_command_age_uses_latest_attempt_and_never_claims_physical_completion():
    decoded = decode_command({'commands': [
        {'requestedAt': 1000, 'sentAt': 1000, 'replyAt': 1200, 'method': 'app_rc_move', 'dispatch': 'sent', 'response': 'received'},
        {'requestedAt': 2000, 'sentAt': None, 'replyAt': 2100, 'method': 'app_rc_end', 'dispatch': 'not_sent', 'response': 'transport_error'}]}, now=4)
    assert decoded['age_s'] == 2
    assert decoded['outcome'] == 'not_sent/transport_error'
    assert decoded['method'] == 'app_rc_end'
    assert math.isnan(decode_command({'commands': []}, now=4)['age_s'])


def test_unknown_arm_is_not_a_measured_zero_joint():
    values = decode_arm({'state': 'UNKNOWN', 'unknownReason': 'mode exited', 'joints': {'m3': 0}, 'sigmaDeg': {'m3': 2.5}})
    assert values['state'] == 'UNKNOWN'
    assert values['unknown_reason'] == 'mode exited'
    assert math.isnan(values['m3_deg'])
    assert values['sigma_m3_deg'] == 2.5
    deployed = decode_arm({'state': 'WORKING_POS', 'joints': {'m3': 150}, 'sigmaDeg': {'m3': 1}})
    assert deployed['m3_deg'] == 150


def test_calibration_nulls_remain_unknown_and_vo_counts_are_typed():
    values = decode_calibration({'yaw_positive': {'gain': 1.2}, 'camera_x': {'value_m': None, 'ready': False}}, {'ok': 3, 'rejected': 2, 'camera_x_m': .16})
    assert values['yaw_positive_gain'] == 1.2
    assert math.isnan(values['camera_x_m'])
    assert values['camera_x_ready'] is False
    assert values['vo_accepted'] == 3
    assert values['vo_rejected'] == 2
    assert values['active_camera_x_m'] == .16


@pytest.mark.parametrize('code,name', [(2, 'CHARGER_DISCONNECTED'), (3, 'IDLE'), (39, 'REMOTE_PICKUP'), (42, 'PROGRAM_MODE'), (999, 'UNKNOWN(999)')])
def test_state_codes_are_not_inferred_from_modes(code, name):
    decoded = decode_status({'telemetry': {'status': {'value': {'state': code}}}}, now=10)
    assert decoded['state_name'] == name


def test_absent_status_does_not_claim_no_error_or_empty_battery():
    decoded = decode_status({}, now=10)
    assert decoded['status_valid'] is False
    assert decoded['error_code'] == -1
    assert decoded['error_name'] == 'UNKNOWN(-1)'
    assert decoded['charging'] is None
    assert math.isnan(decoded['battery'])


def test_cached_status_error_or_age_invalidates_live_battery_and_flags():
    for entry in [
        {'value': {'state': 8, 'battery': 80, 'in_cleaning': 0}, 'observedAt': 1000, 'error': None},
        {'value': {'state': 8, 'battery': 80, 'in_cleaning': 0}, 'observedAt': 10000, 'error': 'timeout'}]:
        decoded = decode_status({'telemetry': {'status': entry}}, now=11)
        assert decoded['status_valid'] is False
        assert math.isnan(decoded['battery'])
        assert decoded['charging'] is None
        assert decoded['cleaning'] is None


def test_tabs_reference_existing_panels_and_do_not_include_command_panels():
    visited = set()

    def walk(item):
        if isinstance(item, str):
            assert item in CONFIGS
            assert item not in visited
            visited.add(item)
            if item.startswith('Tab!'):
                for tab in CONFIGS[item]['tabs']:
                    walk(tab['layout'])
        else:
            assert item['direction'] in ('row', 'column')
            assert 0 < item['splitPercentage'] < 100
            walk(item['first'])
            walk(item['second'])

    walk(LAYOUT['layout'])
    assert visited == CONFIGS.keys()
    assert [t['title'] for t in CONFIGS['Tab!z70-dashboard']['tabs']] == [
        'Overview', 'Cameras', 'Arm', 'Odometry & calibration', 'Diagnostics']
    assert not any(k.split('!')[0] in ('Publish', 'Teleop', 'ServiceCall') for k in CONFIGS)


def test_3d_and_image_configs_use_real_fields_and_sources():
    scene = CONFIGS['3D!Robot-map-and-depth']
    assert scene['layers']['z70-urdf']['sourceType'] == 'topic'
    assert scene['layers']['z70-urdf']['topic'] == '/robot_description'
    assert scene['followTf'] == 'odom'
    assert scene['fixedFrame'] == 'map'
    assert scene['layers']['z70-grid']['frameId'] == 'odom'
    assert scene['topics']['/z70/odom_path']['type'] == 'line'
    assert scene['topics']['/z70/odom_path']['lineWidth'] == 0.02
    assert scene['topics']['/z70/floor_mosaic']['cameraInfoTopic'] == '/z70/camera_info'
    assert scene['topics']['/map']['visible'] is True
    assert scene['cameraState']['distance'] == 8.0
    dock = scene['topics']['/z70/map/dock_pose']
    assert dock['visible'] is True
    assert dock['type'] == 'arrow'
    assert dock['color'] == '#34d399'
    assert dock['arrowScale'] == [0.35, 0.08, 0.08]
    assert dock['showCovariance'] is False
    assert scene['topics']['/camera/front/depth/points']['colorField'] == 'z'
    for camera in ('front', 'gripper'):
        assert any(c.get('imageMode', {}).get('imageTopic') == f'/camera/{camera}/image_raw/compressed'
                   for c in CONFIGS.values())
    for removed in ('left', 'right', 'back'):
        assert not any(c.get('imageMode', {}).get('imageTopic') == f'/camera/{removed}/image_raw/compressed'
                       for c in CONFIGS.values())
    assert CONFIGS['RawMessages!Detected-objects']['topicPath'] == '/z70/objects/json'
    assert CONFIGS['RawMessages!Camera-state']['topicPath'] == '/z70/camera/state'
    assert CONFIGS['Indicator!Active-lens']['path'] == '/z70/camera/state.selected_camera'
    assert CONFIGS['3D!Arm-closeup']['followTf'] == 'base_footprint'


def test_spatial_context_and_floor_mosaic_are_visible_in_dashboard_tabs():
    assert CONFIGS['RawMessages!Robot-status']['topicPath'] == '/z70/state'
    assert CONFIGS['RawMessages!Spatial-context']['topicPath'] == '/z70/map/spatial_context'
    overview = CONFIGS['Tab!z70-dashboard']['tabs'][0]['layout']
    assert 'RawMessages!Spatial-context' in json.dumps(overview)
    mosaic = CONFIGS['Image!Floor-mosaic']['imageMode']
    assert mosaic['imageTopic'] == '/z70/floor_mosaic'
    assert mosaic['imageSchemaName'] == 'sensor_msgs/msg/Image'
    odometry = CONFIGS['Tab!z70-dashboard']['tabs'][3]['layout']
    assert 'Image!Floor-mosaic' in json.dumps(odometry)
    trail = CONFIGS['3D!Odometry-trail']
    assert trail['cameraState']['distance'] == 4.5
    assert trail['topics']['/z70/map/dock_pose']['visible'] is False


def deployed(now=lambda: 100.):
    e = ArmEstimator(now=now); e.set_anchor('stowed'); e.observe('app_arm_out_compartment', {}, 'received'); return e


def test_unknown_start_and_deploy_requires_unrestored_stow():
    e = ArmEstimator(); assert e.snapshot()['state'] == 'UNKNOWN'
    assert not e.observe('app_arm_out_compartment', {}, 'received')
    e.set_anchor('stowed'); e.observe('app_arm_out_compartment', {}, 'received')
    assert e.snapshot()['joints'] == dict(m3=150, m4=45, m5=0, m6=90)
    e.observe('app_stop_grasp', [], 'received'); assert e.snapshot()['wrist'] is None
    assert not e.observe('app_arm_out_compartment', {}, 'received')


def test_nominal_vendor_endpoint_is_not_the_cad_wrist():
    a1, a2 = map(math.radians, (120, -15))
    x = .0475 + .1775*math.cos(a1) + .205*math.cos(a2)
    y = .14 + .1775*math.sin(a1) + .205*math.sin(a2)
    e = deployed()
    assert e.snapshot()['wrist'] == pytest.approx(dict(x=x, y=y))
    assert e.observe('app_arm_direction_move', {'direction': 3}, 'received')
    d = e.snapshot()
    assert d['wrist'] == pytest.approx(dict(x=x+.04, y=y))
    j = d['joints']
    u, v = map(math.radians, (270-j['m3'], j['m4']-j['m3']+90))
    assert .0475+.1775*math.cos(u)+.205*math.cos(v) == pytest.approx(x+.04)
    assert .14+.1775*math.sin(u)+.205*math.sin(v) == pytest.approx(y)


def test_nominal_fk_ik_and_hand_not_physical_transform():
    for m3, m4 in ((150, 45), (200, 90), (240, 130)):
        assert vendor_inverse(*vendor_forward(m3, m4)[1], (m3, m4)) == pytest.approx((m3, m4))
    e = deployed(); before = e.snapshot()['wrist']
    for direction in (3, 3, 2): e.observe('app_arm_direction_move', dict(direction=direction), 'received')
    assert e.snapshot()['wrist'] == pytest.approx(dict(x=before['x']+.08, y=before['y']-.04))


def test_jog_direction_sigma_clamp_and_firmware_predictive_limit():
    e = deployed()
    for m in ([0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 2]): e.observe('app_arm_move', dict(m=m), 'received')
    d = e.snapshot(); assert d['joints']['m5'] == 20 and d['stepsSinceAnchor'] == 4
    assert d['sigmaDeg']['m5'] == pytest.approx(5.)
    assert e.observe('app_arm_move', dict(m=[0, 0, 2, 0, 0]), 'received')
    assert e.snapshot()['joints']['m5'] == 10
    assert not e.observe('app_pick_up_enter_exit', dict(flag=1), 'received')
    assert e.observe('app_put_down', dict(flag=0), 'received')
    assert e.snapshot()['state'] == 'IN_PROCESS_POS' and e.snapshot()['joints']['m6'] == 90
    for _ in range(20): e.observe('app_arm_move', dict(m=[0, 1, 0]), 'received')
    assert e.snapshot()['joints']['m4'] == 170
    e = deployed()
    for m in ([1, 1, 0], [1, 1, 0], [1, 0, 0], [1, 1, 0], [1, 1, 0], [1, 1, 0], [1, 1, 0], [1, 0, 0], [0, 1, 0]):
        e.observe('app_arm_move', dict(m=m), 'received')
        assert e.snapshot()['wrist']['x'] <= .367


@pytest.mark.parametrize('outcome', ['not_sent', 'vendor_error'])
def test_rejected_unsent_and_logical_duplicate_never_integrate(outcome):
    e = deployed(); before = e.snapshot()
    assert not e.observe('app_arm_move', dict(m=[0, 0, 1]), outcome)
    assert e.snapshot() == before
    e.observe('app_arm_move', dict(m=[0, 0, 1]), 'received', command_id='one')
    assert not e.observe('app_arm_move', dict(m=[0, 0, 1]), 'received', command_id='one')
    assert e.snapshot()['joints']['m5'] == 10


def test_unknown_jog_cannot_recover_without_stow_and_grip_never_lifts():
    e = deployed(); e.observe('app_arm_move', dict(m=[0, 0, 1]), 'unknown')
    assert e.snapshot()['state'] == 'UNKNOWN'
    assert not e.observe('app_arm_move', dict(m=[0, 0, 1]), 'received')
    e.observe('app_arm_in_compartment', {}, 'received'); assert e.snapshot()['state'] == 'IN_ROBOT'
    e = deployed(); wrist = e.snapshot()['wrist']
    e.observe('app_pick_up_enter_exit', dict(flag=1), 'received')
    assert not e.observe('app_grip', dict(flag=0), 'unknown')
    assert e.snapshot()['state'] == 'WORKING_POS' and e.snapshot()['joints']['m6'] == 90
    assert e.observe('app_grip', dict(flag=0), 'received')
    assert e.snapshot()['state'] == 'WORKING_POS' and e.snapshot()['joints']['m6'] == 0 and e.snapshot()['wrist'] == wrist
    for outcome in ('received', 'unknown'):
        e = deployed(); j = e.snapshot()['joints']
        e.observe('app_open_gripper', {}, 'unknown'); assert e.snapshot()['state'] == 'WORKING_POS'
        e.observe('app_grip', {}, outcome)
        assert e.snapshot()['state'] == 'UNKNOWN' and e.snapshot()['joints'] == j and e.snapshot()['wrist'] is None
        assert e.snapshot()['unknownReason'] == 'GRIP_POSE_UNVERIFIED'


def test_program_priority_steps_and_autonomous_unknown():
    e = deployed(); e.observe('app_start_program', dict(cmd_ids=[
        dict(cmd_id='arm_joints', value=[0, 1, 1, 2], step=2), dict(cmd_id='delay'),
        dict(cmd_id='arm_move', direction=1, step=1)]), 'received')
    d = e.snapshot(); assert d['stepsSinceAnchor'] == 3 and d['joints']['m5'] == 20 and d['joints']['m6'] == 0
    assert d['wrist']['y'] == pytest.approx(vendor_forward(150, 65)[1][1]+.04)
    e.observe('app_start_program', dict(cmd_ids=[dict(cmd_id='arm_grasp')]), 'received')
    assert e.snapshot()['state'] == 'UNKNOWN'
    e.observe('app_start_program', dict(cmd_ids=[dict(cmd_id='arm_in')]), 'received')
    assert e.snapshot()['state'] == 'IN_ROBOT'


def test_status_old_requests_mode_exit_and_official_app_visibility_limits():
    clock = [100.]; e = deployed(lambda: clock[0]); clock[0] += 1
    e.observe_status(dict(state=39, common_status=1 << 10), 101000)
    before = e.snapshot()
    e.observe_status(dict(state=39, common_status=1 << 10), 102000); assert e.snapshot() == before
    e.observe_status(dict(common_status=0), 101500); assert e.snapshot() == before
    e.observe_status(dict(common_status=0), 103000); assert e.snapshot()['state'] == 'UNKNOWN'
    e = deployed(lambda: 100.); e.observe_status(dict(state=3), 99999); assert e.snapshot()['state'] == 'WORKING_POS'
    e.observe_status(dict(state=42, common_status=0), 102000); assert e.snapshot()['state'] == 'WORKING_POS'
    e.observe_status(dict(state=41), 103000); assert e.snapshot()['state'] == 'UNKNOWN'


def test_restart_is_unreconciled_even_if_saved_stowed(tmp_path):
    path = tmp_path/'durable/arm.json'; e = ArmEstimator(path=path); e.set_anchor('stowed')
    e.observe('app_arm_out_compartment', {}, 'received'); e.observe('app_arm_move', dict(m=[0, 0, 1]), 'received')
    loaded = ArmEstimator(path=path); d = loaded.snapshot()
    assert d['restored'] and d['joints']['m5'] == 10 and d['wrist'] is None
    assert not loaded.observe('app_arm_move', dict(m=[0, 0, 1]), 'received')
    loaded.set_anchor('stowed'); assert not loaded.snapshot()['restored']
    assert ArmEstimator(path=path).snapshot()['restored']
    path.write_text('{broken'); assert ArmEstimator(path=path).snapshot()['state'] == 'UNKNOWN'


@pytest.mark.parametrize("state,joints", [
    ("WORKING_POS", {"m3": 150, "m4": 45, "m5": 0, "m6": 90}),
    ("IN_PROCESS_POS", {"m3": 210, "m4": 110, "m5": 35, "m6": 0}),
])
def test_estimate_maps_vendor_angles_to_joint_rotations(state, joints):
    q = bridge.app_to_urdf(**estimate_to_app({"state": state, "joints": joints}))
    assert q["m3_joint"] == pytest.approx(math.radians(joints["m3"] - 270))
    assert q["m4_joint"] == pytest.approx(math.radians(180 - joints["m4"]))
    assert q["m5_joint"] == pytest.approx(-math.radians(joints["m5"]))
    assert q["m6_left_joint"] == q["m6_right_joint"] == pytest.approx(math.radians(joints["m6"]) / 2)


@pytest.mark.parametrize("state", ["UNKNOWN", "IN_ROBOT"])
def test_untrusted_or_stowed_estimate_folds_visual_arm(state):
    folded = estimate_to_app({"state": state, "joints": {"m3": 0, "m4": 0, "m5": 0, "m6": 0}})
    assert folded == bridge.STOW_VISUAL
    assert bridge.app_to_urdf(**folded)["m3_joint"] == pytest.approx(-math.pi)
    assert bridge.app_to_urdf(**folded)["m4_joint"] == pytest.approx(math.radians(10))


def test_unknown_estimate_with_deployed_joints_preserves_visual_arm_pose():
    deployed_joints = {"m3": 150.0, "m4": 45.0, "m5": 0.0, "m6": 90.0}
    assert estimate_to_app({"state": "UNKNOWN", "joints": deployed_joints}) == deployed_joints
    assert estimate_to_app({"state": "IN_ROBOT", "joints": deployed_joints}) == bridge.STOW_VISUAL


def test_gripper_visual_stays_within_finger_limits():
    assert bridge.app_to_urdf(150, 45, 0, -10)["m6_left_joint"] == 0
    assert bridge.app_to_urdf(150, 45, 0, 180)["m6_left_joint"] == pytest.approx(math.pi / 4)


def test_no_queue_and_task_token_conflicts():
    cancelled = []; stops = []
    d = Dispatcher(lambda scope: stops.append(scope) or [], cancelled.append)
    ticket = d.reserve(); assert ticket and d.reserve() is None
    assert not d.begin()[0]
    d.release(ticket)
    accepted, token, reason = d.begin()
    assert accepted and token != ZERO and reason == 'NONE'
    assert d.reserve() is None and d.reserve(bytes([1]) * 16) is None
    assert d.begin(token)[0] and not d.begin()[0]
    owned = d.reserve(token); assert owned
    assert not d.end(bytes([1]) * 16)
    assert d.end(token) and cancelled == [token] and stops == [3]
    assert owned.cancelled.is_set() and owned.stop_done.is_set()
    assert d.reserve(token) is None
    assert d.reserve() is not None


def test_takeover_revokes_then_stop_then_new_write():
    entered = threading.Event(); release = threading.Event(); events = []
    def stop(scope):
        events.append('stop'); entered.set(); assert release.wait(2); events.append('stopped'); return [scope]
    d = Dispatcher(stop, lambda token: events.append('cancel'))
    _, token, _ = d.begin(); old = d.reserve(token)
    new = []
    thread = threading.Thread(target=lambda: new.append(d.takeover())); thread.start()
    assert entered.wait(1)
    assert events == ['cancel', 'stop'] and old.cancelled.is_set()
    assert d.reserve() is None and d.reserve(token) is None
    d.release(old)
    release.set(); thread.join(2)
    assert new[0] and events == ['cancel', 'stop', 'stopped']
    assert not d.valid(old) and d.valid(new[0])
    d.release(old); assert d.reserve() is None


def test_cancel_async_revokes_immediately_and_stops_once():
    callbacks = []; stops = []
    d = Dispatcher(lambda scope: stops.append(scope) or [scope])
    ticket = d.reserve()
    d.cancel(ticket, callbacks.append)
    assert ticket.cancelled.is_set() and not ticket.stop_done.is_set()
    assert d.reserve() is None and len(callbacks) == 1
    d.cancel(ticket, callbacks.append); assert len(callbacks) == 1
    callbacks[0]()
    assert stops == [3] and ticket.stop_done.is_set() and ticket.stop_outcomes == [3]
    assert d.reserve() is not None


def test_priority_stop_never_waits_for_reserved_command():
    stopped = []; d = Dispatcher(lambda scope: stopped.append(scope) or [scope])
    old = d.reserve()
    assert d.stop(2) == [2] and stopped == [2]
    assert old.cancelled.is_set() and old.stop_done.is_set()
    assert d.reserve() is not None


def test_suspended_session_cannot_takeover_or_reserve_but_scoped_stop_still_works():
    stops = []; d = Dispatcher(lambda scope: stops.append(scope) or [scope])
    d.session_enabled = False
    assert d.takeover() is None
    assert d.reserve() is None and not d.begin(takeover=True)[0]
    for scope in (1, 2, 3): assert d.stop(scope) == [scope]
    assert stops == [1, 2, 3] and not d.session_enabled


def test_failed_stop_latches_no_blind_takeover_and_successful_stop_recovers():
    received = [False]
    d = Dispatcher(lambda scope: [received[0]], stop_ok=lambda values: all(values))
    old = d.reserve()
    assert d.takeover() is None and old.cancelled.is_set()
    assert d.blocked and d.reserve() is None and not d.begin()[0]
    received[0] = True
    d.stop(3)
    assert not d.blocked and d.reserve() is not None
