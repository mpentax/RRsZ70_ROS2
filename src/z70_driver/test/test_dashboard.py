"""Dashboard diagnostic status, telemetry publishers, and Foxglove layout integrity."""
import json
import math
from pathlib import Path
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.serialization import serialize_message, deserialize_message
from z70_driver.dashboard import Dashboard, decode_arm, decode_calibration, decode_command, decode_status
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
