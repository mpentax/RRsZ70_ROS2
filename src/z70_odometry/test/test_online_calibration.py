import math
from types import SimpleNamespace
import pytest
from z70_odometry.calibration_state import calibrated_camera_x, fill_state
from z70_odometry.chassis_calibrator import OnlineCalibration


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
        dx = distance * (math.sin(angle) / angle if angle else 1) + (x - old) * (math.cos(angle) - 1)
        dy = distance * ((1 - math.cos(angle)) / angle if angle else 0) + (x - old) * math.sin(angle)
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
