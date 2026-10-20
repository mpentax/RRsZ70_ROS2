import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace, SimpleNamespace as NS
import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from z70_interfaces.msg import CalibrationState, CommandEvent, OdometryStatus
from z70_odometry import chassis_calibrator as resources
from z70_odometry.chassis_calibrator import CalibrationPublication, CommandStream, OnlineCalibration, _arc, calibrated_camera_x, fill_state, gains_from_state


def motion(c, index, velocity=0, omega=.5, rejected=False, silent=False, dispatch='sent'):
    start = index * 6 + 1
    duration = .6
    th = omega * duration * (.65 if omega >= 0 else .67)
    x, old = .28, .16
    for j in range(17):
        t = start - 1 + j * .25
        if silent and 3 < j < 14:
            continue
        fraction = max(0, min(1, (t - start) / duration))
        angle = th * fraction
        distance = velocity * duration * .55 * fraction
        arc_x, arc_y = _arc(angle)
        dx = distance * arc_x + (x - old) * (math.cos(angle) - 1)
        dy = distance * arc_y + (x - old) * math.sin(angle)
        c.add_pose(dict(t=t, x=dx, y=dy, yaw=angle, camera_x_m=old, rejected=rejected and j == 6))
    command = dict(
        id=str(index),
        method='app_rc_move',
        params=[dict(velocity=velocity, omega=omega, duration=600, seqnum=index + 1)],
        dispatch=dispatch,
        response='received',
        sentAt=start * 1000,
        replyAt=(start + .02) * 1000,
    )
    c.add_command(command)
    c.process(start + 4)
    return command


@pytest.fixture
def cal(tmp_path):
    return OnlineCalibration(tmp_path / 'online.json')


def test_stream_convergence_dedup_and_persistence(cal, tmp_path):
    for i in range(24):
        command = motion(cal, i, velocity=.1 if i % 3 == 2 else 0, omega=0 if i % 3 == 2 else (.5 if i % 3 == 0 else -.5))
        cal.add_command(command)
        cal.process(i * 6 + 5)
    result = cal.summary()
    assert result['n'] == 24
    assert result['yaw_positive']['gain'] == pytest.approx(.65)
    assert result['yaw_negative']['gain'] == pytest.approx(.67)
    assert result['forward']['gain'] == pytest.approx(.55)
    assert result['camera_x']['value_m'] == pytest.approx(.28)
    assert result['camera_x']['ready']
    restored = OnlineCalibration(tmp_path / 'online.json')
    assert restored.summary() == result
    restored.add_command(command)
    restored.process(200)
    assert restored.summary()['n'] == 24


def test_rejected_silent_and_uncertain_samples_are_skipped(cal):
    motion(cal, 0, rejected=True)
    motion(cal, 1, silent=True)
    motion(cal, 2, dispatch='unknown')
    motion(cal, 3)
    assert cal.summary()['n'] == 1
    assert cal.summary()['skipped'] == 3


def test_mixed_turn_translation_converges_and_offset_is_consumed(cal):
    for i in range(12):
        motion(cal, i, velocity=.1, omega=.5 if i % 2 == 0 else -.5)
    summary = cal.summary()
    assert summary['camera_x']['value_m'] == pytest.approx(.28)
    assert summary['forward']['gain'] == pytest.approx(.55)

    def message():
        return SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace()),
            evidence=SimpleNamespace(stale=False),
            validity=SimpleNamespace(),
        )

    not_ready = dict(summary, camera_x=dict(summary['camera_x'], ready=False))
    assert calibrated_camera_x(fill_state(message(), not_ready, 'test', 1.)) is None
    assert calibrated_camera_x(fill_state(message(), summary, 'test', 1.)) == pytest.approx(.28)


def test_overlapping_commands_are_not_misassigned(cal):
    cal.add_command(dict(id='a', method='app_rc_move', params=[dict(velocity=0, omega=.5, duration=600)], dispatch='sent', response='received', sentAt=1000, replyAt=1020))
    cal.add_command(dict(id='b', method='app_rc_move', params=[dict(velocity=0, omega=-.5, duration=600)], dispatch='sent', response='received', sentAt=2000, replyAt=2020))
    cal.process(10)
    assert cal.summary()['n'] == 0
    assert cal.summary()['skipped'] == 2


def _cmd_event(phase, seq, attempt=2, dispatch=1, response=1):
    msg = CommandEvent(phase=phase, operation='app_rc_move')
    msg.evidence.session_id.uuid = [1] * 16; msg.evidence.sequence = seq
    msg.evidence.source_time_known = True; msg.evidence.source_clock = 1
    msg.evidence.source_at.sec = 100
    msg.outcome.command_id.uuid = [9] * 16
    msg.outcome.attempt_id.uuid = [attempt] * 16
    msg.outcome.dispatch = dispatch; msg.outcome.response = response
    msg.requested_twist.angular.z = .2; msg.requested_duration.nanosec = 500_000_000
    return msg


def test_unknown_retry_and_finished_event_never_double_count(tmp_path):
    estimator = OnlineCalibration(tmp_path / 'calibration.json'); stream = CommandStream(estimator)
    stream.add(_cmd_event(0, 1))
    stream.add(_cmd_event(1, 2)); stream.add(_cmd_event(2, 3, dispatch=2, response=3))
    assert not estimator.pending and estimator.skipped == 1
    stream.add(_cmd_event(1, 4, attempt=3)); stream.add(_cmd_event(2, 5, attempt=3))
    stream.add(_cmd_event(4, 6, attempt=3)); stream.add(_cmd_event(4, 6, attempt=3))
    assert len(estimator.pending) == 1 and len(estimator.seen) == 2
    assert estimator.pending[0]['start'] == 100
    assert estimator.pending[0]['id'].endswith(bytes([3] * 16).hex())


def test_gap_restart_or_unmapped_clock_does_not_invent_attempts(tmp_path):
    estimator = OnlineCalibration(tmp_path / 'calibration.json'); stream = CommandStream(estimator)
    stream.add(_cmd_event(1, 1)); stream.add(_cmd_event(2, 3))
    assert not estimator.pending and estimator.skip_reasons['command event gap'] == 1
    msg = _cmd_event(1, 4); msg.evidence.source_clock = 2
    stream.add(msg); stream.add(_cmd_event(2, 5))
    assert not estimator.pending
    msg = _cmd_event(1, 1); msg.evidence.session_id.uuid = [5] * 16; stream.add(msg)
    reply = _cmd_event(2, 2); reply.evidence.session_id.uuid = [5] * 16; stream.add(reply)
    assert len(estimator.pending) == 1
    assert estimator.skip_reasons['command event gap'] == 2


def _cal_message():
    return NS(header=NS(stamp=NS()), evidence=NS(stale=False, session_id=NS()), validity=NS())


def test_runtime_calibration_path(monkeypatch, tmp_path):
    monkeypatch.setenv('XDG_DATA_HOME', str(tmp_path / 'data'))
    first = resources.runtime_calibration_path()
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv('Z70_REPO', '/unusable/archive')
    assert first == resources.runtime_calibration_path() == tmp_path / 'data/RRsZ70_ROS2/calibration/chassis/online.json'
    assert not first.exists()
    monkeypatch.setenv('XDG_DATA_HOME', 'relative-invalid')
    assert resources.runtime_calibration_path() == Path.home() / '.local/share/RRsZ70_ROS2/calibration/chassis/online.json'
    monkeypatch.delenv('XDG_DATA_HOME')
    assert resources.runtime_calibration_path() == Path.home() / '.local/share/RRsZ70_ROS2/calibration/chassis/online.json'


def test_initial_camera_x_m():
    assert resources.initial_camera_x_m() == .16


def test_atomic_save_retains_identity(tmp_path):
    path = tmp_path / 'data/online.json'; estimator = OnlineCalibration(path)
    estimator.seed(resources.seed_directory())
    assert estimator.summary()['n'] > 0
    original = path.read_bytes(); ident = hashlib.sha256(original).hexdigest()
    assert estimator.calibration_id == ident
    restored = OnlineCalibration(path); restored.seed(resources.seed_directory()); restored.save()
    assert path.read_bytes() == original
    assert restored.updated_at == estimator.updated_at
    assert restored.calibration_id == ident
    assert restored.summary() == estimator.summary()
    restored.skip('test observation gap'); restored.save()
    assert restored.calibration_id != ident
    assert not list(path.parent.glob('*.tmp'))
    assert path.stat().st_mode & 0o077 == 0


def test_empty_state_snapshot(tmp_path):
    estimator = OnlineCalibration(tmp_path / 'online.json'); estimator.save()
    publication = CalibrationPublication()
    state = publication.snapshot(estimator, 'initial receipt', _cal_message)
    assert not state.evidence.valid and state.validity.value == 0
    assert not state.forward_known and not state.forward_sigma_known
    assert state.forward_gain == state.forward_sigma == 0
    assert not state.camera_x_known and not state.camera_x_ready
    assert calibrated_camera_x(state) is None and gains_from_state(state) == {}
    assert publication.snapshot(estimator, 'later timer', _cal_message) is state
    assert state.evidence.received_at == 'initial receipt'
    assert state.evidence.sequence == 1
    assert state.calibration_id == hashlib.sha256(estimator.path.read_bytes()).hexdigest()


def test_seeded_state_snapshot(tmp_path):
    estimator = OnlineCalibration(tmp_path / 'online.json'); estimator.seed(resources.seed_directory())
    publication = CalibrationPublication(); state = publication.snapshot(estimator, 'receipt', _cal_message)
    assert state.validity.value == 1 and state.evidence.valid
    assert state.sample_count == estimator.summary()['n']
    assert state.evidence.source_time_known and state.evidence.source_clock == 1
    assert state.evidence.timestamp_kind == 3
    assert state.camera_x_m == estimator.summary()['camera_x']['value_m']
    for name, value in gains_from_state(state).items():
        assert value['gain'] == estimator.summary()[name]['gain']
    state.evidence.stale = True
    assert gains_from_state(state) == {} and calibrated_camera_x(state) is None


def test_single_sample_sigma_and_ready_gate(tmp_path):
    estimator = OnlineCalibration(tmp_path / 'online.json')
    estimator.add_measurement(dict(id='synthetic', command_yaw=.3, command_forward=0., yaw=.2,
                                   camera_dx=.28 * (np.cos(.2) - 1), camera_dy=.28 * np.sin(.2)))
    estimator.save(); state = CalibrationPublication().snapshot(estimator, 'receipt', _cal_message)
    assert state.camera_x_known and not state.camera_x_sigma_known
    assert not state.camera_x_ready and calibrated_camera_x(state) is None
    assert state.yaw_positive_known and not state.yaw_positive_sigma_known


def test_ros_calibrator_consumes_stream_and_persists(tmp_path):
    context = Context()
    rclpy.init(args=[], context=context, domain_id=95)
    node = Node('synthetic_calibration_stream', context=context)
    executor = SingleThreadedExecutor(context=context)
    odom = node.create_publisher(Odometry, '/odom', 100)
    status = node.create_publisher(OdometryStatus, '/z70/odom_status', 100)
    commands = node.create_publisher(CommandEvent, '/z70/commands', 256)
    results = []
    node.create_subscription(CalibrationState, '/z70/chassis_calibration', results.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    path = tmp_path / 'online.json'
    log = (tmp_path / 'calibrator.log').open('w')
    process = subprocess.Popen(
        [sys.executable, '-m', 'z70_odometry.chassis_calibrator', '--ros-args', '-p', f'output_file:={path}', '-p', 'seed_directory:=""'],
        env=dict(os.environ, ROS_DOMAIN_ID='95'), stdout=log, stderr=log)
    try:
        deadline = time.monotonic() + 8
        while (odom.get_subscription_count() == 0 or status.get_subscription_count() == 0 or commands.get_subscription_count() == 0) and time.monotonic() < deadline:
            rclpy.spin_once(node, executor=executor, timeout_sec=.1)
        assert commands.get_subscription_count() > 0
        base = time.time_ns() - 6_000_000_000
        start = base + 1_000_000_000
        for i in range(17):
            stamp = base + i * 250_000_000
            fraction = max(0, min(1, (stamp - start) / 600_000_000))
            yaw = .5 * .6 * .65 * fraction
            msg = Odometry()
            msg.header.stamp.sec = stamp // 10**9
            msg.header.stamp.nanosec = stamp % 10**9
            msg.pose.pose.position.x = (.28 - .16) * (math.cos(yaw) - 1)
            msg.pose.pose.position.y = (.28 - .16) * math.sin(yaw)
            msg.pose.pose.orientation.z = math.sin(yaw / 2)
            msg.pose.pose.orientation.w = math.cos(yaw / 2)
            quality = OdometryStatus(pair_accepted=True, active_camera_x_known=True, active_camera_x_m=.16)
            quality.header = msg.header
            quality.evidence.valid = True
            quality.validity.value = 1
            odom.publish(msg)
            status.publish(quality)
        deadline = time.monotonic() + .3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, executor=executor, timeout_sec=.05)
        command = CommandEvent(operation='app_rc_move', phase=1)
        command.evidence.session_id.uuid = [1] * 16
        command.evidence.sequence = 1
        command.evidence.source_time_known = True
        command.evidence.source_clock = 1
        command.evidence.source_at.sec = start // 10**9
        command.evidence.source_at.nanosec = start % 10**9
        command.outcome.attempt_id.uuid = [2] * 16
        command.requested_twist.angular.z = .5
        command.requested_duration.nanosec = 600_000_000
        commands.publish(command)
        command.phase = 2
        command.evidence.sequence = 2
        command.outcome.dispatch = 1
        command.outcome.response = 1
        commands.publish(command)
        deadline = time.monotonic() + 5
        while not any(r.sample_count == 1 for r in results) and time.monotonic() < deadline:
            rclpy.spin_once(node, executor=executor, timeout_sec=.1)
        measured = next(r for r in results if r.sample_count == 1)
        assert measured.yaw_positive_known and abs(measured.yaw_positive_gain - .65) < 1e-6
        assert measured.camera_x_known and abs(measured.camera_x_m - .28) < 1e-6
        assert json.loads(path.read_text())['n'] == 1
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        log.close()
    assert process.poll() is not None
