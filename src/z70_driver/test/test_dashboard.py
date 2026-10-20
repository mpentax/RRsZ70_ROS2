"""Dashboard diagnostic status and telemetry publishers."""
import math
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.serialization import serialize_message, deserialize_message
from z70_driver.dashboard import Dashboard
from z70_interfaces.msg import CalibrationState, CommandEvent, OdometryStatus


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
