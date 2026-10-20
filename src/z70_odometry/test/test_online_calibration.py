import math
from types import SimpleNamespace
import pytest
from z70_interfaces.msg import CommandEvent
from z70_odometry.calibration_state import calibrated_camera_x, fill_state
from z70_odometry.chassis_calibrator import CommandStream, OnlineCalibration, _arc


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
