"""Dashboard diagnostic status, telemetry publishers, Foxglove layout integrity, and shared ROS driver tests."""
from copy import deepcopy
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import importlib
import json
import math
import os
from pathlib import Path
import queue
import signal
import struct
import subprocess
import threading
import time
from types import SimpleNamespace
import uuid

import cv2
import numpy as np
import pytest
import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState
from z70_description.arm_model import vendor_forward, vendor_inverse
from z70_driver.commands import Dispatcher, ZERO
from z70_driver.driver import (
    ArmEstimator, Dashboard, Driver, LATCHED, MatchedFrames, STOW_VISUAL,
    app_to_urdf, decode_arm, decode_calibration, decode_command, decode_status,
    estimate_to_app, host_ms, host_time,
)
from z70_interfaces.msg import ArmEstimate, CalibrationState, CommandEvent, FrameMetadata, MapSnapshot, OdometryStatus, RobotState
from z70_interfaces.srv import RefreshMap, SelectCamera
from z70_manipulation.node import Observations
from z70_mapping.node import MappingNode

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
    q = app_to_urdf(**estimate_to_app({"state": state, "joints": joints}))
    assert q["m3_joint"] == pytest.approx(math.radians(joints["m3"] - 270))
    assert q["m4_joint"] == pytest.approx(math.radians(180 - joints["m4"]))
    assert q["m5_joint"] == pytest.approx(-math.radians(joints["m5"]))
    assert q["m6_left_joint"] == q["m6_right_joint"] == pytest.approx(math.radians(joints["m6"]) / 2)


@pytest.mark.parametrize("state", ["UNKNOWN", "IN_ROBOT"])
def test_untrusted_or_stowed_estimate_folds_visual_arm(state):
    folded = estimate_to_app({"state": state, "joints": {"m3": 0, "m4": 0, "m5": 0, "m6": 0}})
    assert folded == STOW_VISUAL
    assert app_to_urdf(**folded)["m3_joint"] == pytest.approx(-math.pi)
    assert app_to_urdf(**folded)["m4_joint"] == pytest.approx(math.radians(10))


def test_unknown_estimate_with_deployed_joints_preserves_visual_arm_pose():
    deployed_joints = {"m3": 150.0, "m4": 45.0, "m5": 0.0, "m6": 90.0}
    assert estimate_to_app({"state": "UNKNOWN", "joints": deployed_joints}) == deployed_joints
    assert estimate_to_app({"state": "IN_ROBOT", "joints": deployed_joints}) == STOW_VISUAL


def test_gripper_visual_stays_within_finger_limits():
    assert app_to_urdf(150, 45, 0, -10)["m6_left_joint"] == 0
    assert app_to_urdf(150, 45, 0, 180)["m6_left_joint"] == pytest.approx(math.pi / 4)


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


def wait_for(predicate, seconds=5.):
    deadline = time.monotonic() + seconds
    while not predicate():
        assert time.monotonic() < deadline, 'condition deadline'
        time.sleep(.01)


def synthetic_map():
    def block(kind,data,extra=b''):
        return struct.pack('<HHI',kind,8+len(extra),len(data))+extra+data
    image=block(2,bytes([1,23,0,39,255,7]),struct.pack('<iiII',100,200,2,3))
    body=image+block(8,struct.pack('<iii',10075,5025,90))+struct.pack('<HHI',1024,8,20)
    raw=b'rr'+struct.pack('<HI',20,len(body)+20)+bytes(12)+body
    return raw+hashlib.sha1(raw).digest()


@pytest.fixture
def backend():
    data:dict=dict(camera='front',calls=[],fail_state=False,fail_frame=False,wrong_tag=False,
              session=str(uuid.uuid4()),updated=int(time.time()*1000),frames=0,
              saved=int(time.time()*1000),stop=threading.Event(),events=queue.Queue(),arm_state='IN_PROCESS_POS',stale_state=False)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def reply(self,value,status=200,headers=None):
            content=value if isinstance(value,bytes) else json.dumps(value).encode()
            self.send_response(status)
            self.send_header('Content-Length',str(len(content)))
            for k,v in (headers or {}).items():self.send_header(k,str(v))
            self.end_headers();self.wfile.write(content)
        def do_GET(self):
            data['calls'].append(('GET',self.path,None))
            if self.path=='/api/commands/events':
                self.send_response(200);self.send_header('Content-Type','application/x-ndjson');self.end_headers()
                while not data['stop'].is_set():
                    try:row=data['events'].get(timeout=.2);line=json.dumps(row).encode()+b'\n'
                    except queue.Empty:line=b'\n'
                    try:self.wfile.write(line);self.wfile.flush()
                    except (BrokenPipeError,ConnectionResetError):break
                return
            if self.path=='/api/state':
                if data['fail_state']:return self.reply({},503)
                now=int(time.time()*1000)-(10000 if data['stale_state'] else 0)
                return self.reply(dict(observationSession=data['session'],connection=dict(state='connected'),telemetry=dict(
                    status=dict(value=dict(state=39,error_code=0,battery=80,common_status=256,in_cleaning=0,in_returning=0),observedAt=now),
                    camera=dict(value=[32768 if data['camera']=='gripper' else 0],observedAt=now))))

            if self.path=='/api/map/cache/raw':
                return self.reply(synthetic_map(),headers={'X-Roboclaw-Observation-Session':data['session'],'X-Map-Saved-At':data['saved'],'X-Map-Index':1,'X-Map-Sequence':4})
            if self.path=='/api/home/rooms':return self.reply(dict(rooms=[dict(id=10,name='Synthetic room')]))
            self.reply({},404)
        def do_POST(self):
            body=json.loads(self.rfile.read(int(self.headers.get('Content-Length',0))))
            data['calls'].append(('POST',self.path,body))
            if self.path=='/api/camera/frame':
                assert body=={}
                if data['fail_frame']:return self.reply({},503)
                data['frames']+=1
                height,width=data.get('image_shape',(12 if data['camera']=='front' else 10,16))
                image=np.full((height,width,3),data['frames']%255,np.uint8)
                jpeg=cv2.imencode('.jpg',image)[1].tobytes()
                return self.reply(jpeg,headers={'X-Roboclaw-Camera':'invalid' if data['wrong_tag'] else data['camera'],
                    'X-Roboclaw-Camera-Session':data['session'],'X-Roboclaw-Received-At':int(time.time()*1000)})
            if self.path=='/api/driver/camera/select':
                data['camera']=body['camera'];return self.reply(dict(camera=data['camera']))
            if self.path=='/api/map/cache/refresh':return self.reply(dict(available=True))
            if self.path=='/api/driver/rpc':
                assert body['method']=='get_room_mapping'  # No fake motion hidden inside acquisition.
                return self.reply(dict(result=[[2,10]], commandId=str(uuid.uuid4()), attemptId=str(uuid.uuid4()),
                    attemptIndex=0, startedAt=int(time.time()*1000), finishedAt=int(time.time()*1000),
                    dispatch='sent', response='received'))
            self.reply({},404)
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
    thread=threading.Thread(target=server.serve_forever);thread.start()
    data['origin']=f'http://127.0.0.1:{server.server_port}'
    try:yield data
    finally:data['stop'].set();server.shutdown();server.server_close();thread.join()


def camera_yaml(path,height):
    path.write_text(yaml.safe_dump(dict(image_width=16,image_height=height,distortion_model='plumb_bob',
        camera_matrix=dict(data=[20.,0.,8.,0.,21.,height/2,0.,0.,1.]),
        distortion_coefficients=dict(data=[.1,-.01,0.,0.,0.]),
        rectification_matrix=dict(data=[1.,0.,0.,0.,1.,0.,0.,0.,1.]),
        projection_matrix=dict(data=[20.,0.,8.,0.,0.,21.,height/2,0.,0.,0.,1.,0.]))))


def test_shared_acquisition_serialization_late_join_switch_stale_cleanup(backend,tmp_path):
    front=tmp_path/'front.yaml';gripper=tmp_path/'gripper.yaml'
    camera_yaml(front,12);camera_yaml(gripper,10)
    rclpy.init(domain_id=96)
    driver=Driver([Parameter('origin',value=backend['origin']),Parameter('camera_hz',value=8.),
                   Parameter('arm_estimate_path',value=''),
                   Parameter('state_hz',value=5.),Parameter('camera_info',value=str(front)),
                   Parameter('gripper_camera_info',value=str(gripper)),Parameter('map_refresh_s',value=60.),
                   Parameter('camera_stale_after_s',value=.4),Parameter('state_stale_after_s',value=2.)])
    with driver.estimator_lock:
        driver.estimator.set_anchor('deployed',reason='TEST_ANCHOR')
        expected_arm=driver.estimator.snapshot()
        driver.publish_estimate(expected_arm)
    mapping=MappingNode();client=rclpy.create_node('shared_test_consumers')
    executor=MultiThreadedExecutor(num_threads=5)
    for node in (driver,mapping,client):executor.add_node(node)
    thread=threading.Thread(target=executor.spin);thread.start()
    observers=[]
    try:
        wait_for(lambda:'map' in driver.samples and 'arm' in driver.samples and 'camera' in driver.samples)
        # Subscribe AFTER acquisition: latched source identity/time must be replayed, not recreated.
        original=deepcopy(driver.samples['map'][0])
        maps=[];states=[];arms=[];grids=[];raws=[];jpegs=[];infos=[];metas=[];diagnostics=[];events=[]
        client.create_subscription(MapSnapshot,'/z70/driver/map_snapshot',maps.append,LATCHED)
        client.create_subscription(RobotState,'/z70/state',states.append,LATCHED)
        client.create_subscription(ArmEstimate,'/z70/arm_estimate',arms.append,LATCHED)
        client.create_subscription(OccupancyGrid,'/map',grids.append,LATCHED)
        for camera in ('front','gripper'):
            client.create_subscription(Image,f'/camera/{camera}/image_raw',raws.append,2)
            client.create_subscription(CompressedImage,f'/camera/{camera}/image_raw/compressed',jpegs.append,2)
            client.create_subscription(CameraInfo,f'/camera/{camera}/camera_info',infos.append,2)
            client.create_subscription(FrameMetadata,f'/camera/{camera}/metadata',metas.append,2)
        client.create_subscription(DiagnosticArray,'/diagnostics',diagnostics.append,10)
        client.create_subscription(CommandEvent,'/z70/commands',events.append,256)
        observers=[Observations(client,timeout=2.),Observations(client,timeout=2.)]
        wait_for(lambda:maps and states and arms and grids and metas and infos and raws and jpegs)
        assert maps[0].evidence==original.evidence
        assert bytes(maps[0].raw)==synthetic_map() and maps[0].rooms[0].name=='Synthetic room'
        assert not maps[0].evidence.source_time_known and not maps[0].current_pose_verified
        assert list(grids[0].data)==[100,0,-1,0,0,0]
        assert states[-1].connected and states[-1].battery_fraction==pytest.approx(.8)
        assert arms[-1].validity.value==1 and arms[-1].source==1
        assert arms[-1].joint_sigma_rad==pytest.approx(np.radians([expected_arm['sigmaDeg'][j] for j in ('m3','m4','m5')]))
        assert not arms[-1].wrist_uncertainty_known
        arm_id=deepcopy(arms[-1].evidence)
        time.sleep(.5)
        assert driver.samples['arm'][0].evidence==arm_id
        before=backend['frames']
        for observer in observers:
            image,tag=observer.frame();assert tag=='front' and image.shape==(12,16,3)
            assert observer.get('/api/state')['telemetry']['status']['value']['state']==39
            assert observer.get('/api/state')['telemetry']['status']['value']['common_status']==256
            assert observer.get('/api/state')['telemetry']['status']['value']['in_cleaning'] is False
            assert observer.get('/api/arm/estimate')['joints']['m3']==pytest.approx(expected_arm['joints']['m3'])
        # Two independent ROS readers did not cause extra HTTP acquisition.
        assert backend['frames']-before<=2
        for msg in (maps[0],states[-1],arms[-1],raws[-1],jpegs[-1],infos[-1],metas[-1]):
            assert deserialize_message(serialize_message(msg),type(msg))==msg
        common=set((m.header.stamp.sec,m.header.stamp.nanosec) for m in raws)&set((m.header.stamp.sec,m.header.stamp.nanosec) for m in jpegs)&set((m.header.stamp.sec,m.header.stamp.nanosec) for m in infos)&set((m.header.stamp.sec,m.header.stamp.nanosec) for m in metas)
        assert common
        assert all(not m.exposure_time_known and not m.evidence.source_time_known for m in metas)
        assert all(m.session_scope==1 for m in metas)
        assert infos[-1].k[0]==20. and infos[-1].k[4]==21.
        observers[0].select('gripper')
        wait_for(lambda:any(m.camera==2 and m.height==10 for m in metas))
        image,tag=observers[0].frame();assert tag=='gripper' and image.shape==(10,16,3)
        assert len([c for c in backend['calls'] if c[1]=='/api/driver/camera/select'])==1
        assert all(c[2]=={} for c in backend['calls'] if c[1]=='/api/camera/frame')
        wait_for(lambda:client.count_subscribers('/z70/commands')>0)
        session=str(uuid.uuid4());command=str(uuid.uuid4());attempt=str(uuid.uuid4())
        for i,phase in enumerate(('accepted','dispatched','replied','finished'),1):
            backend['events'].put(dict(sessionId=session,sequence=i,commandId=command,attemptId=attempt,attemptIndex=0,
                phase=phase,operation='app_rc_move',requestedAt=backend['updated'],attemptedAt=backend['updated']+1,
                finishedAt=backend['updated']+20 if i>2 else None,dispatch='unknown' if i==2 else 'sent',
                response='none' if i<3 else 'received',errorCode='NONE',params=dict(velocity=0.,omega=.2,duration=500)))
        wait_for(lambda:len(events)==4)
        assert [e.phase for e in events]==[0,1,2,4]
        assert not any(e.outcome.write_time_known for e in events)
        assert events[1].outcome.dispatch==2
        # A duplicate is ignored; a skipped sequence is diagnosed, never replayed.
        row=dict(sessionId=session,sequence=6,commandId=command,attemptId=attempt,attemptIndex=0,
            phase='finished',operation='app_rc_stop',requestedAt=backend['updated'],finishedAt=backend['updated'],
            dispatch='unknown',response='transport_error',errorCode='TIMEOUT')
        backend['events'].put(row);backend['events'].put(row)
        wait_for(lambda:any(s.message=='EVENT_GAP' for d in diagnostics for s in d.status))
        wait_for(lambda:len(events)==5)
        assert len(events)==5
        # Backend restart changes acquisition sessions, even if cache/estimate values agree.
        oldsession=bytes(states[-1].evidence.session_id.uuid)
        backend['session']=str(uuid.uuid4())
        wait_for(lambda:bytes(states[-1].evidence.session_id.uuid)!=oldsession)
        wait_for(lambda:driver.samples['arm'][0].evidence.session_id==driver.acquisition_session(backend['session']))
        refresh=client.create_client(RefreshMap,'/z70/map/refresh')
        assert refresh.wait_for_service(timeout_sec=2.)
        future=refresh.call_async(RefreshMap.Request());wait_for(future.done)
        assert future.result().outcome.response==1
        wait_for(lambda:maps[-1].evidence.session_id!=original.evidence.session_id)
        assert maps[-1].evidence.received_at==original.evidence.received_at
        # Display-only stow is not planner state or observed physical stow.
        with driver.estimator_lock:
            driver.estimator.observe('app_arm_in_compartment',{},'received',command_id=str(uuid.uuid4()))
            driver.publish_estimate(driver.estimator.snapshot())
        wait_for(lambda:arms[-1].validity.value==3)
        wait_for(lambda:observers[0].arm.validity.value==3)
        assert observers[0].get('/api/arm/estimate')['state']=='IN_ROBOT'
        assert not arms[-1].joints.position
        # Correctly tagged but wrong-size calibration never reaches metric readers.
        oldcal=driver.calibrations['gripper'];wrong=deepcopy(oldcal[0]);wrong.height=100
        driver.calibrations['gripper']=(wrong,oldcal[1])
        wait_for(lambda:any(m.evidence.reason=='CALIBRATION_MISMATCH' for m in metas))
        wait_for(lambda:observers[0].image is not None and not observers[0].image[1].evidence.valid)
        observers[0].timeout=.1
        with pytest.raises(LookupError):observers[0].frame()
        driver.calibrations['gripper']=oldcal
        wait_for(lambda:observers[0].image is not None and observers[0].image[1].evidence.valid)
        # HTTP success with an old vendor observation does not refresh state.
        backend['stale_state']=True
        wait_for(lambda:states[-1].evidence.stale)
        backend['stale_state']=False
        wait_for(lambda:not states[-1].evidence.stale)
        oldstamp=deepcopy(driver.samples['state'][0].evidence.source_at)
        backend['fail_state']=True;backend['fail_frame']=True
        wait_for(lambda:states[-1].evidence.stale and any(m.evidence.stale for m in metas))
        assert not states[-1].connected and states[-1].evidence.source_at==oldstamp
        for d in diagnostics:
            roundtrip=deserialize_message(serialize_message(d),DiagnosticArray)
            assert [s.level for s in roundtrip.status] == [s.level for s in d.status]
            for status in roundtrip.status:
                if status.name.startswith('z70_driver/'):
                    assert status.level==DiagnosticStatus.WARN
        for observer in observers:observer.timeout=.1
        with pytest.raises(LookupError):observers[0].get('/api/state')
        with pytest.raises(LookupError):observers[0].frame()
    finally:
        for observer in observers:observer.close()
        driver.close();executor.shutdown();thread.join(timeout=5)
        for node in (driver,mapping,client):node.destroy_node()
        rclpy.try_shutdown()
    assert not thread.is_alive() and all(not w.is_alive() for w in driver.workers)
    assert not any(c[1]=='/api/commands/recent' for c in backend['calls'])
    assert not any(c[1]=='/api/arm/estimate' for c in backend['calls'])


def test_all_generated_observation_and_service_types_serialize():
    backend=str(uuid.uuid4())
    first=SimpleNamespace(driver_session=uuid.uuid4())
    second=SimpleNamespace(driver_session=uuid.uuid4())
    assert Driver.acquisition_session(first,backend)!=Driver.acquisition_session(second,backend)
    module=importlib.import_module('z70_interfaces.msg')
    names=['EvidenceStamp','Validity','RobotState','CameraState','FrameMetadata','ArmEstimate',
           'OdometryStatus','RoomLabel','MapSnapshot','GarmentObservation','GarmentObservations',
           'CommandOutcome','CommandEvent']
    for name in names:
        cls=getattr(module,name);msg=cls()
        assert deserialize_message(serialize_message(msg),cls)==msg
    msg=CommandEvent(operation='vendor_advanced_'+'research_'*10)
    assert deserialize_message(serialize_message(msg),CommandEvent)==msg
    for cls in (SelectCamera.Request,SelectCamera.Response,RefreshMap.Request,RefreshMap.Response):
        msg=cls();assert deserialize_message(serialize_message(msg),cls)==msg


@pytest.mark.parametrize('milliseconds',[1792074587789,1792074587123,1792074587999])
def test_backend_millisecond_timestamps_preserve_exact_source_identity(milliseconds):
    stamp=host_ms(milliseconds)
    assert stamp.sec*1000+stamp.nanosec//1_000_000==milliseconds
    assert stamp.nanosec%1_000_000==0


def test_frame_join_rejects_unknown_calibration_stale_and_mismatch():
    # Pure callback test with actual generated messages, no ROS graph needed.
    join=MatchedFrames.__new__(MatchedFrames)
    seen=[];sessions=[]
    join.callback=seen.append;join.on_session=lambda:sessions.append(True)
    join.frame=None;join.metadata=None;join.used=None;join.session=None
    image=Image();image.header.stamp=host_time(time.time());image.header.frame_id='camera_optical_frame'
    meta=FrameMetadata(header=deepcopy(image.header),calibration_id='synthetic')
    meta.evidence.valid=True;meta.evidence.received_at=host_time(time.time());meta.evidence.sequence=1
    join.image(image);join.info(meta);assert len(seen)==1
    join.info(meta);assert len(seen)==1
    meta.evidence.sequence=2;meta.evidence.valid=False;join.info(meta);assert len(seen)==1
    meta.evidence.sequence=3;meta.evidence.valid=True;meta.calibration_id='';join.info(meta);assert len(seen)==1
    meta.evidence.sequence=4;meta.calibration_id='synthetic';meta.evidence.stale=True;join.info(meta);assert len(seen)==1
    meta.evidence.sequence=5;meta.evidence.stale=False;meta.header.stamp.sec+=1;join.info(meta);assert len(seen)==1


def test_standard_launch_acquires_fake_images_maps_state_and_shuts_down(backend, tmp_path):
    backend['image_shape'] = (1200, 1600)
    token = tmp_path / 'synthetic-token'; token.write_text('offline-launch-fixture'); token.chmod(0o600)
    env = dict(os.environ, ROS_DOMAIN_ID='94', XDG_DATA_HOME=str(tmp_path / 'application-data'),
               ROS_LOG_DIR=str(tmp_path / 'ros-log'))
    env.pop('Z70_REPO', None)
    context = Context(); rclpy.init(context=context, domain_id=94)
    node = rclpy.create_node('offline_standard_launch_probe', context=context)
    executor = SingleThreadedExecutor(context=context); executor.add_node(node)
    states = []; images = []; maps = []; calibrations = []; odometry = []
    node.create_subscription(RobotState, '/z70/state', states.append, LATCHED)
    node.create_subscription(CompressedImage, '/camera/front/image_raw/compressed', images.append, 2)
    node.create_subscription(OccupancyGrid, '/map', maps.append, LATCHED)
    node.create_subscription(CalibrationState, '/z70/chassis_calibration', calibrations.append, LATCHED)
    node.create_subscription(OdometryStatus, '/z70/odom_status', odometry.append, 10)
    log = (tmp_path / 'standard-launch.log').open('w')
    process = subprocess.Popen(['ros2', 'launch', 'z70_bringup', 'bringup.launch.py',
        'origin:=' + backend['origin'], 'token_file:=' + str(token), 'foxglove_port:=0'],
        cwd=tmp_path, env=env, stdout=log, stderr=log, start_new_session=True)
    forced = False
    try:
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline and not (states and images and maps and calibrations and odometry):
            assert process.poll() is None, 'Standard launch exited during fake acquisition'
            executor.spin_once(timeout_sec=.1)
        assert states and images and maps and calibrations and odometry, 'Required launched ROS observation topics missing'
        assert odometry[-1].evidence.valid and odometry[-1].active_camera_x_known
        assert not odometry[-1].covariance_calibrated
        assert cv2.imdecode(np.frombuffer(bytes(images[-1].data), np.uint8), cv2.IMREAD_COLOR).shape[:2] == (1200, 1600)
        assert backend['frames'] > 0
        assert all(path != '/api/driver/rpc' or body['method'] == 'get_room_mapping'
                   for method, path, body in backend['calls'])
    finally:
        if process.poll() is None: process.send_signal(signal.SIGINT)
        try: process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            forced = True; os.killpg(process.pid, signal.SIGKILL); process.wait()
        log.close(); executor.shutdown(); node.destroy_node(); context.shutdown()
    assert not forced, 'Owned offline launch did not finish bounded standard shutdown'
    assert process.returncode == 0
