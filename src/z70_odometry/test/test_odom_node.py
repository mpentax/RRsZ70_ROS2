"""Floor odometry node tests."""
from copy import deepcopy
import math
from types import SimpleNamespace

import cv2
import numpy as np
from z70_interfaces.msg import CalibrationState, CommandEvent, EvidenceStamp
from sensor_msgs.msg import Image

import pytest
from builtin_interfaces.msg import Time

from z70_odometry import node as odom_node
from z70_odometry.floor_vo import FloorCamera, render_floor_view
from z70_odometry.node import FloorMosaic


def stamp(ns):
    return Time(sec=ns // 10**9, nanosec=ns % 10**9)


@pytest.fixture
def odom(monkeypatch):
    pubs, timers, subscriptions = {}, [], []
    clock = SimpleNamespace(ns=100 * 10**9)
    clock.now = lambda: SimpleNamespace(nanoseconds=clock.ns, to_msg=lambda: stamp(clock.ns))

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, msg):
            self.messages.append(deepcopy(msg))

    def publisher(self, kind, topic, qos):
        pubs[topic] = Publisher()
        return pubs[topic]

    monkeypatch.setattr(odom_node.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(odom_node.Node, 'declare_parameter',
                        lambda self, name, default: SimpleNamespace(value=default))
    monkeypatch.setattr(odom_node.Node, 'get_clock', lambda self: clock)
    monkeypatch.setattr(odom_node.Node, 'create_publisher', publisher)
    monkeypatch.setattr(odom_node.Node, 'create_subscription',
                        lambda self, kind, topic, cb, qos: subscriptions.append((kind, topic, cb, qos)))
    monkeypatch.setattr(odom_node.Node, 'create_timer',
                        lambda self, period, cb: timers.append((period, cb)))
    transforms = []
    monkeypatch.setattr(odom_node, 'TransformBroadcaster',
                        lambda self: SimpleNamespace(sendTransform=lambda t: transforms.append(deepcopy(t))))
    monkeypatch.setattr(odom_node.FloorCamera, 'load', lambda path: SimpleNamespace(W=2, H=2))
    monkeypatch.setattr(odom_node, 'FloorOdometry',
                        lambda cam, camera_x_m, **opts: SimpleNamespace(cam=cam, cam_x=camera_x_m))
    metadata = SimpleNamespace(evidence=EvidenceStamp(valid=True), calibration_id='offline')
    monkeypatch.setattr(odom_node, 'MatchedFrames', lambda *a, **kw: SimpleNamespace(metadata=metadata))
    result = odom_node.FloorOdomNode()
    result.test = SimpleNamespace(clock=clock, pubs=pubs, timers=timers,
                                  subscriptions=subscriptions, transforms=transforms)
    return result


def test_initial_odom_and_periodic_tf(odom):
    assert len(odom.test.timers) == 1
    period, heartbeat = odom.test.timers[0]
    assert period == .2
    heartbeat()
    initial = odom.test.pubs['/odom'].messages[-1]
    assert initial.header.frame_id == 'odom'
    assert initial.child_frame_id == 'base_footprint'
    assert (initial.pose.pose.position.x, initial.pose.pose.position.y) == (0., 0.)
    assert (initial.pose.pose.orientation.z, initial.pose.pose.orientation.w) == (0., 1.)
    assert odom.last_stamp == initial.header.stamp
    for _ in range(3):
        odom.test.clock.ns += 200_000_000
        heartbeat()
        tf = odom.test.transforms[-1]
        assert tf.header.frame_id == 'odom'
        assert tf.child_frame_id == 'base_footprint'
        assert tf.header.stamp == stamp(odom.test.clock.ns)
        assert (tf.transform.translation.x, tf.transform.translation.y) == (0., 0.)
        assert (tf.transform.rotation.z, tf.transform.rotation.w) == (0., 1.)
    assert len(odom.test.pubs['/odom'].messages) == 1


def command(phase, seq, *, start_ns=100 * 10**9, duration=.5, velocity=.2, omega=.4,
            attempt=2, dispatch=1, response=1):
    msg = CommandEvent(phase=phase, operation='app_rc_move')
    msg.evidence.session_id.uuid = [1] * 16
    msg.evidence.sequence = seq
    msg.evidence.source_time_known = True
    msg.evidence.source_clock = 1
    msg.evidence.source_at = stamp(start_ns)
    msg.outcome.attempt_id.uuid = [attempt] * 16
    msg.outcome.finished_at = stamp(start_ns + round(duration * 1e9))
    msg.outcome.dispatch = dispatch
    msg.outcome.response = response
    msg.requested_twist.linear.x = float(velocity)
    msg.requested_twist.angular.z = float(omega)
    duration_ns = round(duration * 1e9)
    msg.requested_duration.sec, msg.requested_duration.nanosec = divmod(duration_ns, 10**9)
    return msg


@pytest.mark.parametrize('omega,gain', [(.4, .8), (-.4, .6), (0., .8)])
def test_calibrated_command_arc_without_camera(odom, omega, gain):
    odom.on_calibration(CalibrationState(yaw_positive_known=True, yaw_positive_gain=.8,
                                       yaw_negative_known=True, yaw_negative_gain=.6,
                                       forward_known=True, forward_gain=1.2))
    assert any(topic == '/z70/commands' and qos == 256
               for _, topic, _, qos in odom.test.subscriptions)
    odom.x, odom.y, odom.th = 1., -1., .3
    odom.on_command(command(1, 1, omega=omega))
    odom.test.clock.ns += 500_000_000
    odom.on_command(command(2, 2, omega=omega))
    angle = omega * .5 * gain
    arc_x, arc_y = odom_node._arc(angle)
    local_x = .2 * .5 * 1.2 * arc_x
    local_y = .2 * .5 * 1.2 * arc_y
    assert (odom.x, odom.y, odom.th) == pytest.approx(
        (1. + math.cos(.3) * local_x - math.sin(.3) * local_y,
         -1. + math.sin(.3) * local_x + math.cos(.3) * local_y, .3 + angle))
    measured = odom.test.pubs['/odom'].messages[-1]
    assert [measured.pose.covariance[i] for i in (0, 7, 35)] == [1e6] * 3
    assert measured.header.stamp == stamp(odom.test.clock.ns)
    quality = odom.test.pubs['/z70/odom_status'].messages[-1]
    assert not quality.pair_accepted
    assert quality.validity.value == 2
    assert quality.rejection == quality.validity.reason == 'COMMAND_DEAD_RECKONED'
    assert quality.header.stamp == measured.header.stamp
    assert odom.prev is None
    before = (odom.x, odom.y, odom.th)
    odom.on_command(command(2, 2, omega=omega))
    odom.on_command(command(4, 3, omega=omega))
    assert (odom.x, odom.y, odom.th) == before


def image(odom, offset_s=0., value=0):
    msg = Image(encoding='bgr8', width=2, height=2, step=6, data=[value] * 12)
    odom.test.clock.ns = 100 * 10**9 + round(offset_s * 1e9)
    msg.header.stamp = stamp(odom.test.clock.ns)
    return msg


def motion(dx=.03, dy=-.01, angle=.1):
    return SimpleNamespace(dx=dx, dy=dy, dtheta=angle,
                           inliers=100, rms_m=.001)


def test_accepted_vo(odom):
    odom.vo.estimate = lambda prev, current: motion()
    odom.on_image(image(odom))
    odom.on_command(command(1, 1))
    odom.on_image(image(odom, .6))
    assert (odom.x, odom.y, odom.th) == pytest.approx((.03, -.01, .1))
    assert odom.last_vo_accept_ns == 100_600_000_000
    assert not odom.last_rejected
    assert odom.prev_stamp_ns == 100_600_000_000
    assert not any(odom.test.pubs['/odom'].messages[-1].pose.covariance)
    quality = odom.test.pubs['/z70/odom_status'].messages[-1]
    assert quality.pair_accepted and quality.validity.value == 1
    assert quality.rejection == ''
    odom.on_command(command(2, 2))
    assert (odom.x, odom.y, odom.th) == pytest.approx((.03, -.01, .1))


@pytest.mark.parametrize('reject_before_reply', [False, True])
def test_rejected_vo_falls_back_once_and_rebases_reference(odom, reject_before_reply):
    odom.vo.estimate = lambda prev, current: None
    odom.on_calibration(CalibrationState(forward_known=True, forward_gain=1.2))
    odom.on_image(image(odom))
    odom.on_command(command(1, 1, omega=0.))
    if reject_before_reply:
        odom.on_image(image(odom, .3))
    odom.test.clock.ns = 100_500_000_000
    odom.on_command(command(2, 2, omega=0.))
    if not reject_before_reply:
        assert odom.x == 0.
        odom.on_image(image(odom, .6, value=1))
        assert odom.prev is not None
    else:
        assert odom.prev is None
        odom.on_image(image(odom, .6, value=1))
    assert (odom.x, odom.y, odom.th) == pytest.approx((.12, 0., 0.))
    dead_reckoned = [m for m in odom.test.pubs['/z70/odom_status'].messages
                     if m.rejection == 'COMMAND_DEAD_RECKONED']
    assert len(dead_reckoned) == 1
    assert not dead_reckoned[0].pair_accepted and dead_reckoned[0].validity.value == 2
    model_odom = [m for m in odom.test.pubs['/odom'].messages if m.pose.pose.position.x == pytest.approx(.12)]
    assert model_odom[0].pose.covariance[0] == 1e6
    odom.vo.estimate = lambda prev, current: motion(dx=.02, dy=0., angle=0.)
    odom.on_image(image(odom, .8))
    assert (odom.x, odom.y, odom.th) == pytest.approx((.14, 0., 0.))


def test_reference_after_dead_reckoning_preserves_uncertainty(odom):
    odom.on_command(command(1, 1, omega=0.))
    odom.test.clock.ns += 500_000_000
    odom.on_command(command(2, 2, omega=0.))
    assert odom.prev is None
    odom.on_image(image(odom, .6))
    measured = odom.test.pubs['/odom'].messages[-1]
    assert [measured.pose.covariance[i] for i in (0, 7, 35)] == [1e6] * 3
    quality = odom.test.pubs['/z70/odom_status'].messages[-1]
    assert quality.rejection == 'INITIAL_REFERENCE' and not quality.pair_accepted
    odom.vo.estimate = lambda prev, current: motion(dx=.02, dy=0., angle=0.)
    odom.on_image(image(odom, .8))
    assert not any(odom.test.pubs['/odom'].messages[-1].pose.covariance)


def test_silent_camera_after_reply_falls_back_on_heartbeat(odom):
    odom.on_image(image(odom))
    odom.on_command(command(1, 1, omega=0.))
    odom.test.clock.ns += 500_000_000
    odom.on_command(command(2, 2, omega=0.))
    assert odom.x == 0.
    odom.test.clock.ns += 700_000_000
    odom.broadcast_tf()
    assert odom.x == pytest.approx(.1)
    assert odom.prev is None
    assert odom.test.pubs['/odom'].messages[-1].pose.covariance[0] == 1e6
    assert odom.test.transforms[-1].header.stamp == stamp(odom.test.clock.ns)
    odom.broadcast_tf()
    assert odom.x == pytest.approx(.1)


@pytest.mark.parametrize('reply_before_image', [False, True])
@pytest.mark.parametrize('camera_rejects', [False, True])
@pytest.mark.parametrize('finished_ns', [100_050_000_000, 100_800_000_000, 100_850_000_000])
def test_partial_vo_leaves_unobserved_pulse_tail(odom, reply_before_image, camera_rejects, finished_ns):
    odom.vo.estimate = lambda prev, current: motion(dx=.04, dy=0., angle=0.)
    odom.on_image(image(odom))
    odom.on_command(command(1, 1, duration=.8, omega=0.))
    reply = command(2, 2, duration=.8, omega=0.)
    reply.outcome.finished_at = stamp(finished_ns)
    if reply_before_image:
        odom.test.clock.ns = finished_ns
        odom.on_command(reply)
    odom.on_image(image(odom, .2))
    if not reply_before_image:
        odom.test.clock.ns = max(odom.test.clock.ns, finished_ns)
        odom.on_command(reply)
    assert len(odom.pending) == 1
    assert odom.pending[0]['motion_end_ns'] == 100_800_000_000
    assert odom.pending[0]['end_ns'] == max(finished_ns, 100_800_000_000)
    assert odom.pending[0]['start_ns'] == 100_200_000_000
    assert odom.pending[0]['duration_s'] == pytest.approx(.6)
    if camera_rejects:
        odom.vo.estimate = lambda prev, current: None
        odom.on_image(image(odom, .9))
    else:
        odom.test.clock.ns = 101_300_000_000
        odom.broadcast_tf()
    assert (odom.x, odom.y, odom.th) == pytest.approx((.16, 0., 0.))
    measured = odom.test.pubs['/odom'].messages[-1]
    assert [measured.pose.covariance[i] for i in (0, 7, 35)] == [1e6] * 3
    assert not odom.pending
    assert odom.test.pubs['/z70/odom_status'].messages[-1].rejection == 'COMMAND_DEAD_RECKONED'
    odom.broadcast_tf()
    assert odom.x == pytest.approx(.16)


@pytest.mark.parametrize('reply_ns', [100_050_000_000, 100_250_000_000])
@pytest.mark.parametrize('initial_state', ['absent', 'stale', 'rejected'])
def test_early_reply_and_returning_vo(odom, initial_state, reply_ns):
    if initial_state == 'stale':
        odom.on_image(image(odom, -1.5))
    elif initial_state == 'rejected':
        odom.on_image(image(odom, -.2))
        odom.vo.estimate = lambda prev, current: None
        odom.on_image(image(odom, -.1))
    odom.on_command(command(1, 1, duration=.8, omega=0.))
    reply = command(2, 2, duration=.8, omega=0.)
    reply.outcome.finished_at = stamp(reply_ns)
    if reply_ns < 100_200_000_000:
        odom.test.clock.ns = reply_ns
        odom.on_command(reply)
        assert odom.x == pytest.approx(0.)
        assert len(odom.pending) == 1
    odom.vo.estimate = lambda prev, current: None if initial_state != 'absent' else motion(dx=.04, dy=0., angle=0.)
    odom.on_image(image(odom, .2))
    if reply_ns >= 100_200_000_000:
        odom.test.clock.ns = reply_ns
        odom.on_command(reply)
    assert odom.x == pytest.approx(.04)
    assert len(odom.pending) == 1 and odom.pending[0]['duration_s'] == pytest.approx(.6)
    odom.vo.estimate = lambda prev, current: motion(dx=.12, dy=0., angle=0.)
    odom.on_image(image(odom, .8))
    assert odom.x == pytest.approx(.16)
    assert not odom.pending


def test_pending_pulses_applied_before_new_stale_camera_pulse(odom):
    odom.on_image(image(odom))
    odom.on_command(command(1, 1, velocity=0., omega=1., duration=.2))
    odom.test.clock.ns += 200_000_000
    odom.on_command(command(2, 2, velocity=0., omega=1., duration=.2))
    odom.on_command(command(1, 3, start_ns=100_700_000_000, attempt=3, omega=0.))
    odom.test.clock.ns = 101_200_000_000
    odom.on_command(command(2, 4, start_ns=100_700_000_000, attempt=3, omega=0.))
    assert (odom.x, odom.y, odom.th) == pytest.approx(
        (.1 * math.cos(.14), .1 * math.sin(.14), .14))
    odom.broadcast_tf()
    assert (odom.x, odom.y, odom.th) == pytest.approx(
        (.1 * math.cos(.14), .1 * math.sin(.14), .14))


@pytest.mark.parametrize('value,known', [(float('nan'), True), (float('inf'), True),
                                        (-1., True), (0., True), (2., False)])
def test_unusable_gains_restore_defaults(odom, value, known):
    odom.on_calibration(CalibrationState(yaw_positive_known=True, yaw_positive_gain=.8,
                                       yaw_negative_known=True, yaw_negative_gain=.6,
                                       forward_known=True, forward_gain=1.2))
    odom.on_calibration(CalibrationState(yaw_positive_known=known, yaw_positive_gain=value,
                                       yaw_negative_known=known, yaw_negative_gain=value,
                                       forward_known=known, forward_gain=value))
    assert (odom.yaw_pos_gain, odom.yaw_neg_gain, odom.forward_gain) == (.70, .65, 1.)


@pytest.mark.parametrize('changes', [dict(dispatch=0), dict(dispatch=2), dict(response=2),
                                     dict(velocity=float('nan')), dict(omega=float('inf')),
                                     dict(duration=0.), dict(duration=-.5)])
def test_invalid_commands_ignored(odom, changes):
    odom.on_command(command(1, 1))
    odom.on_command(command(2, 2, **changes))
    assert (odom.x, odom.y, odom.th) == (0., 0., 0.)
    assert not odom.test.pubs['/z70/odom_status'].messages


@pytest.mark.parametrize('missing', ['start', 'gap', 'clock', 'session', 'operation'])
def test_unpaired_command_ignored(odom, missing):
    begin = command(1, 1)
    end = command(2, 2)
    if missing == 'gap':
        end.evidence.sequence = 3
    elif missing == 'clock':
        begin.evidence.source_clock = 2
    elif missing == 'session':
        end.evidence.session_id.uuid = [3] * 16
    elif missing == 'operation':
        end.operation = 'app_rc_stop'
    if missing != 'start':
        odom.on_command(begin)
    odom.on_command(end)
    assert (odom.x, odom.y, odom.th) == (0., 0., 0.)


def test_old_completed_pulse_expired_on_late_rejection(odom):
    odom.vo.estimate = lambda prev, current: None
    odom.on_image(image(odom))
    odom.on_command(command(1, 1))
    odom.test.clock.ns += 500_000_000
    odom.on_command(command(2, 2))
    odom.on_image(image(odom, 3.))
    assert (odom.x, odom.y, odom.th) == (0., 0., 0.)
    assert odom.test.pubs['/z70/odom_status'].messages[-1].rejection == 'VO_REJECTED'
    assert odom.prev_stamp_ns == 100_000_000_000


def test_vo_acceptance_consumes_pending_pulse(odom):
    odom.vo.estimate = lambda prev, current: motion()
    odom.on_image(image(odom))
    odom.on_command(command(1, 1))
    odom.test.clock.ns += 500_000_000
    odom.on_command(command(2, 2))
    odom.on_image(image(odom, .6))
    odom.test.clock.ns += 1_200_000_000
    odom.broadcast_tf()
    assert (odom.x, odom.y, odom.th) == pytest.approx((.03, -.01, .1))
    assert not odom.test.pubs['/z70/odom_status'].messages[-1].rejection


_MOSAIC_CAM = FloorCamera.load()


def test_rotated_translated_views_align_in_odom():
    rng = np.random.default_rng(7)
    coarse = rng.integers(30, 225, (_MOSAIC_CAM.H // 16, _MOSAIC_CAM.W // 16, 3), dtype=np.uint8)
    img = cv2.resize(coarse, (_MOSAIC_CAM.W, _MOSAIC_CAM.H), interpolation=cv2.INTER_CUBIC)
    mosaic = FloorMosaic(_MOSAIC_CAM, resolution=.005, size_m=3, max_range=1.2, camera_x_m=.16)
    mosaic.update(img, (0, 0, 0))
    x, y, th = .03, .01, math.radians(8)
    tc = (x + .16 * (math.cos(th) - 1), y + .16 * math.sin(th), th)
    moved = render_floor_view(img, _MOSAIC_CAM, tc)
    stats = mosaic.update(moved, (x, y, th))
    assert stats['overlap_cells'] > 500
    assert stats['overlap_ncc'] > .95
    assert np.count_nonzero(mosaic.count) > 1000
    assert mosaic.image().dtype == np.uint8
    wrong = FloorMosaic(_MOSAIC_CAM, resolution=.005, size_m=3, max_range=1.2, camera_x_m=.16)
    wrong.update(img, (0, 0, 0))
    misaligned = wrong.update(moved, (0, 0, 0))
    assert misaligned['overlap_ncc'] < .9


def test_empty_view_and_outside_map_do_not_paint():
    mosaic = FloorMosaic(_MOSAIC_CAM)
    blank = np.zeros((_MOSAIC_CAM.H, _MOSAIC_CAM.W, 3), np.uint8)
    assert mosaic.update(blank, (0, 0, 0))['total_cells'] == 0
    assert mosaic.update(blank + 128, (20, 20, 0))['total_cells'] == 0
    with pytest.raises(ValueError):
        mosaic.update(blank[:10], (0, 0, 0))
