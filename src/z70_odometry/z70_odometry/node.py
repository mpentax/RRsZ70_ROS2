"""Floor VO, appearance-only floor mosaic, and online chassis calibration nodes."""
from __future__ import annotations

from collections import OrderedDict, deque
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import time
import uuid

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from z70_description.arm_model import config_path, package_root
from z70_driver.driver import MatchedFrames
from z70_interfaces.msg import CalibrationState, CommandEvent, OdometryStatus

_FALLBACK_ROOT = Path(__file__).resolve().parent.parent


def front_calibration_path():
    return config_path('front_camera.json')


def runtime_calibration_path():
    base = Path(os.environ.get('XDG_DATA_HOME') or Path.home() / '.local/share').expanduser()
    if not base.is_absolute():
        base = Path.home() / '.local/share'
    return base / 'RRsZ70_ROS2/calibration/chassis/online.json'


def _package_root():
    return package_root('z70_odometry', _FALLBACK_ROOT)


def initial_camera_x_m():
    return float(yaml.safe_load((_package_root() / 'config/odometry.yaml').read_text())['camera_x_m'])


def seed_directory():
    path = _package_root() / 'config/chassis'
    if not path.is_dir():
        raise FileNotFoundError(path)
    return path


def _number(value, nonnegative=False):
    return (isinstance(value, (float, int)) and not isinstance(value, bool)
            and math.isfinite(value) and (not nonnegative or value >= 0))


def calibrated_camera_x(state):
    """Only an explicitly usable fit may change the active floor camera offset."""
    if (state.evidence.valid and not state.evidence.stale and state.validity.value == 1
            and state.camera_x_ready and state.camera_x_known and state.camera_x_sigma_known
            and _number(state.camera_x_m) and _number(state.camera_x_sigma_m, True)):
        return float(state.camera_x_m)
    return None


def gains_from_state(state):
    """Executor-compatible gains; absence is unknown, not a guessed replacement."""
    if not state.evidence.valid or state.evidence.stale or state.validity.value != 1:
        return {}
    result = {}
    for name in ('yaw_positive', 'yaw_negative', 'forward'):
        value = getattr(state, name + '_gain')
        if getattr(state, name + '_known') and _number(value):
            sigma = getattr(state, name + '_sigma')
            result[name] = dict(gain=float(value), n=getattr(state, name + '_count'),
                                sigma=float(sigma) if getattr(state, name + '_sigma_known') and _number(sigma, True) else None)
    return result


def fill_state(msg, summary, calibration_id, updated_at):
    """Populate flat fields; missing values stay zero with explicit unknown flags."""
    msg.calibration_id = calibration_id
    msg.sample_count = int(summary['n'])
    msg.skipped_count = int(summary['skipped'])
    msg.model = summary['model']
    msg.header.frame_id = 'base_footprint'
    source_ns = round(updated_at * 10**9) if updated_at is not None else 0
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(source_ns, 10**9)
    msg.evidence.source_at = msg.header.stamp
    msg.evidence.source_time_known = updated_at is not None
    msg.evidence.source_clock = 1 if updated_at is not None else 0
    msg.evidence.timestamp_kind = 3
    msg.evidence.valid = summary['n'] > 0
    msg.evidence.reason = 'FIT_ESTIMATE' if summary['n'] else 'NO_SAMPLES'
    msg.validity.value = 1 if summary['n'] else 0
    msg.validity.reason = msg.evidence.reason
    for name in ('yaw_positive', 'yaw_negative', 'forward'):
        value = summary[name]
        setattr(msg, name + '_known', _number(value['gain']))
        setattr(msg, name + '_gain', float(value['gain']) if _number(value['gain']) else 0.)
        setattr(msg, name + '_sigma_known', _number(value['sigma'], True))
        setattr(msg, name + '_sigma', float(value['sigma']) if _number(value['sigma'], True) else 0.)
        setattr(msg, name + '_count', int(value['n']))
    camera = summary['camera_x']
    for source, field, known in [('value_m', 'camera_x_m', 'camera_x_known'),
                                 ('sigma_m', 'camera_x_sigma_m', 'camera_x_sigma_known'),
                                 ('residual_rms_m', 'camera_x_residual_rms_m', 'camera_x_residual_known')]:
        valid = _number(camera[source], source != 'value_m')
        setattr(msg, known, valid)
        setattr(msg, field, float(camera[source]) if valid else 0.)
    msg.camera_x_count = int(camera['n'])
    msg.camera_x_ready = bool(camera['ready'])
    return msg


class CalibrationPublication:
    """Cache original update evidence across periodic/late-subscriber publication."""
    def __init__(self):
        self.session = uuid.uuid4()
        self.sequence = 0
        self.message = None
        self.identity = None

    def snapshot(self, estimator, received_at, message_type=None):
        identity = estimator.calibration_id
        if self.message is None or self.identity != identity:
            if message_type is None:
                message_type = CalibrationState
            self.sequence += 1
            msg = fill_state(message_type(), estimator.summary(), identity, estimator.updated_at)
            msg.evidence.received_at = received_at
            msg.evidence.session_id.uuid = list(self.session.bytes)
            msg.evidence.sequence = self.sequence
            self.message, self.identity = msg, identity
        return self.message


def local_delta(a, b):
    dx = b['x'] - a['x']; dy = b['y'] - a['y']; th = a['yaw']
    return (math.cos(th) * dx + math.sin(th) * dy,
            -math.sin(th) * dx + math.cos(th) * dy,
            math.atan2(math.sin(b['yaw'] - th), math.cos(b['yaw'] - th)))


def quat_yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp_ns(stamp) -> int:
    s = getattr(stamp, 'header', stamp)
    s = getattr(s, 'stamp', s)
    return s.sec * 10**9 + s.nanosec


def _arc(angle: float) -> tuple[float, float]:
    return (math.sin(angle) / angle, (1. - math.cos(angle)) / angle) if abs(angle) > 1e-8 else (1., 0.)


def slope(rows, command, motion, sign=None):
    rows = [r for r in rows if r[command] and (sign is None or math.copysign(1, r[command]) == sign)]
    if not rows:
        return dict(gain=None, sigma=None, n=0)
    a = np.array([r[command] for r in rows]); b = np.array([r[motion] for r in rows]); denominator = float(a @ a)
    gain = float(a @ b / denominator); residual = b - a * gain
    sigma = float(math.sqrt(float(residual @ residual) / (len(rows) - 1) / denominator)) if len(rows) > 1 else None
    return dict(gain=gain, sigma=sigma, n=len(rows))


class OnlineCalibration:
    def __init__(self, path, settle_s=2, max_gap_s=1.5, window=500, min_camera_n=4, max_camera_sigma_m=.01):
        self.path = Path(path); self.settle_s = settle_s; self.max_gap_s = max_gap_s; self.window = window
        self.min_camera_n = min_camera_n; self.max_camera_sigma_m = max_camera_sigma_m
        self.samples = []; self.seen = []; self.seeded = []; self.skipped = 0; self.skip_reasons = {}; self.pending = []
        self.poses = deque(maxlen=1000); self.events = deque(maxlen=1000)
        self.updated_at = None; self.calibration_id = ''; self._saved_payload = None
        if self.path.exists():
            content = self.path.read_bytes(); data = json.loads(content)
            self.calibration_id = hashlib.sha256(content).hexdigest(); self.updated_at = data.get('updated_at')
            self._saved_payload = {k: v for k, v in data.items() if k != 'updated_at'}
            self.samples = data.get('samples', [])[-window:]; self.seen = data.get('seen', [])[-2048:]
            self.skipped = data.get('skipped', 0); self.skip_reasons = data.get('skip_reasons', {})
            self.seeded = data.get('seeded', [r['id'] for r in self.samples if r['id'].startswith('seed:')])

    def skip(self, reason):
        self.skipped += 1; self.skip_reasons[reason] = self.skip_reasons.get(reason, 0) + 1

    def add_pose(self, pose):
        if all(math.isfinite(pose[k]) for k in ['t', 'x', 'y', 'yaw']):
            self.poses.append(dict(pose))

    def add_command(self, command):
        ident = command.get('id')
        if not isinstance(ident, str) or ident in self.seen:
            return
        self.seen = (self.seen + [ident])[-2048:]
        self.events.append(command)
        if command.get('method') != 'app_rc_move':
            return
        try:
            p = command['params'][0]; start = float(command['sentAt']) / 1000; duration = float(p['duration']) / 1000
            velocity = float(p['velocity']); omega = float(p['omega'])
            if not all(math.isfinite(x) for x in [start, duration, velocity, omega]) or duration <= 0:
                raise ValueError()
        except (KeyError, TypeError, ValueError, IndexError):
            self.skip('invalid observation'); self.save(); return
        if command.get('dispatch') != 'sent' or command.get('response') != 'received':
            self.skip('uncertain/rejected dispatch'); self.save(); return
        self.pending.append(dict(id=ident, start=start, end=start + duration, velocity=velocity, omega=omega, duration=duration))

    def process(self, now):
        changed = False
        for command in list(self.pending):
            start = command['start']; settled = command['end'] + self.settle_s
            before = [p for p in self.poses if p['t'] <= start]
            after = [p for p in self.poses if p['t'] >= settled]
            if not after and now < settled + 5:
                continue
            self.pending.remove(command); changed = True
            if not before or not after:
                self.skip('silent/missing odom'); continue
            a = before[-1]; b = after[0]; points = [p for p in self.poses if a['t'] <= p['t'] <= b['t']]
            if start - a['t'] > self.max_gap_s or b['t'] - settled > self.max_gap_s or any(q['t'] - p['t'] > self.max_gap_s for p, q in zip(points, points[1:])):
                self.skip('silent/missing odom'); continue
            if any(p.get('rejected', False) for p in points):
                self.skip('rejected odom'); continue
            offsets = {p.get('camera_x_m') for p in points}
            if None in offsets and now < settled + 5:
                self.pending.append(command); continue
            if None in offsets or len(offsets) != 1:
                self.skip('unknown/changing odom camera offset'); continue
            interrupted = False
            for event in self.events:
                if event.get('id') == command['id']:
                    continue
                t = event.get('sentAt')
                if t is None:
                    continue
                t = float(t) / 1000; method = event.get('method')
                if method == 'app_rc_move' and t < start:
                    try:
                        if t + float(event['params'][0]['duration']) / 1000 + self.settle_s > start:
                            interrupted = True
                    except (KeyError, IndexError, TypeError, ValueError):
                        pass
                if a['t'] <= t <= b['t'] and (method == 'app_rc_move' or (method in ['app_rc_stop', 'app_rc_end'] and t < command['end'])):
                    interrupted = True
            if interrupted:
                self.skip('overlapping/interrupted motion'); continue
            dx, dy, yaw = local_delta(a, b); old = next(iter(offsets))
            yaw = sum(math.atan2(math.sin(q['yaw'] - p['yaw']), math.cos(q['yaw'] - p['yaw'])) for p, q in zip(points, points[1:]))
            self.add_measurement(dict(id=command['id'], command_yaw=command['omega'] * command['duration'],
                                      command_forward=command['velocity'] * command['duration'], yaw=yaw,
                                      camera_dx=dx + old * (math.cos(yaw) - 1), camera_dy=dy + old * math.sin(yaw),
                                      start=start, end=b['t']))
        if changed:
            self.save()

    def add_measurement(self, row):
        if any(r['id'] == row['id'] for r in self.samples):
            return
        self.samples = (self.samples + [row])[-self.window:]

    def summary(self):
        rows = [dict(r) for r in self.samples]; turn_count = sum(abs(r['yaw']) > 1e-5 for r in rows)
        x = sigma = residual = None; forward = dict(gain=None, sigma=None, n=0)
        if rows:
            matrix = []; target = []
            for r in rows:
                angle = r['yaw']; arc_x, arc_y = _arc(angle)
                matrix.extend([[math.cos(angle) - 1, r['command_forward'] * arc_x], [math.sin(angle), r['command_forward'] * arc_y]])
                target.extend([r['camera_dx'], r['camera_dy']])
            A = np.array(matrix); b = np.array(target); has_forward = any(r['command_forward'] for r in rows)
            if not has_forward:
                A = A[:, :1]
            coefficients, _, rank, _ = np.linalg.lstsq(A, b, rcond=None)
            error = b - A @ coefficients
            if rank == A.shape[1] and turn_count:
                x = float(coefficients[0]); residual = float(np.sqrt(np.mean(error**2)))
                if len(b) > rank and turn_count > 1:
                    covariance = float(error @ error) / (len(b) - rank) * np.linalg.pinv(A.T @ A)
                    sigma = float(np.sqrt(covariance[0, 0]))
            for r in rows:
                angle = r['yaw']; arc_x, arc_y = _arc(angle)
                dx = r['camera_dx'] - (x or 0) * (math.cos(angle) - 1); dy = r['camera_dy'] - (x or 0) * math.sin(angle)
                r['forward'] = (dx * arc_x + dy * arc_y) / (arc_x * arc_x + arc_y * arc_y) if arc_x * arc_x + arc_y * arc_y > 1e-10 else 0.
            forward = slope(rows, 'command_forward', 'forward')
        ready = x is not None and sigma is not None and turn_count >= self.min_camera_n and sigma <= self.max_camera_sigma_m
        return dict(n=len(rows), skipped=self.skipped, skip_reasons=dict(self.skip_reasons),
                    yaw_positive=slope(rows, 'command_yaw', 'yaw', 1),
                    yaw_negative=slope(rows, 'command_yaw', 'yaw', -1),
                    forward=forward,
                    camera_x=dict(value_m=x, sigma_m=sigma, n=turn_count, residual_rms_m=residual, ready=ready),
                    model='windowed_unicycle_arc_ls')

    def save(self):
        payload = dict(version=1, samples=self.samples, seen=self.seen, seeded=self.seeded, **self.summary())
        if payload == self._saved_payload:
            return
        updated_at = time.time()
        content = (json.dumps(dict(payload, updated_at=updated_at), indent=2, allow_nan=False) + '\n').encode()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(dir=self.path.parent, prefix=self.path.name + '.', suffix='.tmp', delete=False) as stream:
                temporary = Path(stream.name); stream.write(content); stream.flush(); os.fsync(stream.fileno())
            os.replace(temporary, self.path)
            directory = os.open(self.path.parent, os.O_RDONLY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
        self.updated_at = updated_at; self.calibration_id = hashlib.sha256(content).hexdigest()
        self._saved_payload = json.loads(content); self._saved_payload.pop('updated_at')

    def seed(self, root):
        root = Path(root)
        for path in sorted(root.glob('2026101*/fit.json')):
            data = json.loads(path.read_text())
            for row in data.get('pulses', []):
                measured = row.get('live_delta')
                if not measured:
                    continue
                ident = 'seed:' + path.parent.name + ':' + row['id']
                if ident in self.seeded:
                    continue
                self.seeded.append(ident)
                yaw = measured['dtheta_rad']; old = measured['camera_x_m']
                self.add_measurement(dict(id=ident, command_yaw=row['omega'] * row['duration_ms'] / 1000,
                                          command_forward=row['velocity'] * row['duration_ms'] / 1000,
                                          yaw=yaw, camera_dx=measured['dx_m'] + old * (math.cos(yaw) - 1),
                                          camera_dy=measured['dy_m'] + old * math.sin(yaw),
                                          source=str(path.relative_to(root))))
        self.save()


class CommandStream:
    """Pair dispatch-start and reply CommandEvents by attempt_id."""
    def __init__(self, estimator):
        self.estimator = estimator; self.session = None; self.sequence = None; self.attempts = {}

    def add(self, msg):
        session = bytes(msg.evidence.session_id.uuid); seq = msg.evidence.sequence
        if self.session == session and self.sequence is not None and seq <= self.sequence:
            return
        if self.session is not None and (self.session != session or seq != self.sequence + 1):
            self.attempts.clear(); self.estimator.pending.clear(); self.estimator.skip('command event gap')
        self.session = session; self.sequence = seq
        ident = session.hex() + ':' + bytes(msg.outcome.attempt_id.uuid).hex()
        if msg.phase == 1 and msg.evidence.source_time_known and msg.evidence.source_clock == 1:
            t = msg.evidence.source_at.sec * 1000 + msg.evidence.source_at.nanosec / 1e6
            self.attempts[ident] = t
            while len(self.attempts) > 256:
                self.attempts.pop(next(iter(self.attempts)))
        elif msg.phase == 2:
            start = self.attempts.pop(ident, None)
            if start is None:
                return
            duration = msg.requested_duration.sec * 1000 + msg.requested_duration.nanosec / 1e6
            self.estimator.add_command(dict(
                id=ident, method=msg.operation,
                params=[dict(velocity=msg.requested_twist.linear.x, omega=msg.requested_twist.angular.z, duration=duration)],
                sentAt=start, dispatch={0: 'not_sent', 1: 'sent', 2: 'unknown'}[msg.outcome.dispatch],
                response={0: 'none', 1: 'received', 2: 'vendor_error', 3: 'transport_error'}[msg.outcome.response],
            ))


class FloorCamera:
    """Vectorised pixel <-> floor mapping for the front camera."""

    def __init__(self, cal: dict):
        self.f, self.cx, self.cy = cal["f_px"], cal["cx"], cal["cy"]
        self.k1, self.k2 = cal["k1"], cal["k2"]
        self.R = np.asarray(cal["R_floor_to_cam"], float)
        self.h = cal["camera_height_mm"] / 1000.0
        self.W, self.H = cal["width"], cal["height"]

    @classmethod
    def load(cls, path=None):
        return cls(json.loads(Path(path or front_calibration_path()).read_text()))

    def pixels_to_floor(self, uv: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(N,2) pixels -> (N,2) floor xy metres, plus mask of pixels whose ray hits the floor."""
        xd = (uv[:, 0] - self.cx) / self.f
        yd = (uv[:, 1] - self.cy) / self.f
        x, y = xd.copy(), yd.copy()
        for _ in range(20):
            r2 = x * x + y * y
            s = 1 + self.k1 * r2 + self.k2 * r2 * r2
            x, y = xd / s, yd / s
        d = np.stack([x, y, np.ones_like(x)], 1) @ self.R
        ok = d[:, 2] < -1e-4
        t = np.where(ok, self.h / -np.where(ok, d[:, 2], -1), 0)
        return np.stack([d[:, 0] * t, d[:, 1] * t], 1), ok

    def floor_to_pixels(self, xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        p = np.c_[xy, np.zeros(len(xy))] - [0, 0, self.h]
        pc = p @ self.R.T
        ok = pc[:, 2] > 1e-6
        z = np.where(ok, pc[:, 2], 1)
        x, y = pc[:, 0] / z, pc[:, 1] / z
        r2 = x * x + y * y
        s = 1 + self.k1 * r2 + self.k2 * r2 * r2
        return np.stack([self.f * x * s + self.cx, self.f * y * s + self.cy], 1), ok

    def floor_mask(self, max_range=1.5, min_range=0.06) -> np.ndarray:
        """Image mask of pixels that map to the floor within a usable range."""
        u, v = np.meshgrid(np.arange(self.W), np.arange(self.H))
        xy, ok = self.pixels_to_floor(np.stack([u.ravel(), v.ravel()], 1).astype(float))
        r = np.hypot(xy[:, 0], xy[:, 1])
        return (ok & (r < max_range) & (r > min_range)).reshape(self.H, self.W)


def decode_bgr8(msg, cam) -> np.ndarray | None:
    if msg.encoding != "bgr8" or msg.width != cam.W or msg.height != cam.H:
        return None
    if msg.step < msg.width * 3 or len(msg.data) != msg.height * msg.step:
        return None
    return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)


@dataclass
class Motion:
    dx: float
    dy: float
    dtheta: float
    inliers: int
    matches: int
    rms_m: float


def rigid2d(a: np.ndarray, b: np.ndarray):
    """Least-squares R, t with b ~ R a + t (2-D Kabsch)."""
    ca, cb = a.mean(0), b.mean(0)
    H = (a - ca).T @ (b - cb)
    th = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    return R, cb - R @ ca, th


class FloorOdometry:
    def __init__(self, cam: FloorCamera, camera_x_m: float | None = None, max_range=1.5, n_features=3000,
                 ransac_thresh_m=0.008, seed=0, min_inliers=80, max_rms_m=0.003):
        self.cam, self.cam_x = cam, initial_camera_x_m() if camera_x_m is None else camera_x_m
        self.min_inliers, self.max_rms = min_inliers, max_rms_m
        self.mask = (cam.floor_mask(max_range) * 255).astype(np.uint8)
        self.orb = cv2.ORB_create(n_features, scaleFactor=1.2, nlevels=8, fastThreshold=10)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.th = ransac_thresh_m
        self.rng = np.random.default_rng(seed)

    def features(self, gray):
        return self.orb.detectAndCompute(gray, self.mask)

    def estimate(self, img0, img1) -> Motion | None:
        g0 = img0 if img0.ndim == 2 else cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY)
        g1 = img1 if img1.ndim == 2 else cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        k0, d0 = self.features(g0); k1, d1 = self.features(g1)
        if d0 is None or d1 is None or len(k0) < 10 or len(k1) < 10:
            return None
        pairs = self.bf.knnMatch(d0, d1, k=2)
        good = [m for m, n in (p for p in pairs if len(p) == 2) if m.distance < 0.8 * n.distance]
        if len(good) < 8:
            return None
        u0 = np.float64([k0[m.queryIdx].pt for m in good]); u1 = np.float64([k1[m.trainIdx].pt for m in good])
        f0, ok0 = self.cam.pixels_to_floor(u0); f1, ok1 = self.cam.pixels_to_floor(u1)
        ok = ok0 & ok1
        f0, f1 = f0[ok], f1[ok]
        n = len(f0)
        if n < 8:
            return None
        best = None
        for _ in range(400):
            i, j = self.rng.choice(n, 2, replace=False)
            if np.linalg.norm(f1[i] - f1[j]) < 0.02:
                continue
            R, t, _ = rigid2d(f1[[i, j]], f0[[i, j]])
            err = np.linalg.norm(f1 @ R.T + t - f0, axis=1)
            inl = err < self.th
            if best is None or inl.sum() > best.sum():
                best = inl
        if best is None or best.sum() < 6:
            return None
        for _ in range(3):
            R, t, th = rigid2d(f1[best], f0[best])
            residual = np.linalg.norm(f1 @ R.T + t - f0, axis=1)
            refined = residual < min(self.th, self.max_rms)
            if refined.sum() < self.min_inliers:
                break
            best = refined
        R, t, th = rigid2d(f1[best], f0[best])
        rms = float(np.sqrt(np.mean(np.sum((f1[best] @ R.T + t - f0[best]) ** 2, 1))))
        if best.sum() < self.min_inliers or rms > self.max_rms:
            return None
        c = np.array([self.cam_x, 0.0])
        tb = t + c - R @ c
        return Motion(float(tb[0]), float(tb[1]), th, int(best.sum()), n, rms)


def render_floor_view(img0, cam: FloorCamera, motion_cam: tuple[float, float, float]):
    """Synthesise the image seen after the camera moves by (dx, dy, dtheta) over a flat floor."""
    dx, dy, th = motion_cam
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    u, v = np.meshgrid(np.arange(cam.W, dtype=float), np.arange(cam.H, dtype=float))
    uv1 = np.stack([u.ravel(), v.ravel()], 1)
    f1, ok = cam.pixels_to_floor(uv1)
    f0 = f1 @ R.T + [dx, dy]
    uv0, ok0 = cam.floor_to_pixels(f0)
    mx = np.where(ok & ok0, uv0[:, 0], -1).reshape(cam.H, cam.W).astype(np.float32)
    my = np.where(ok & ok0, uv0[:, 1], -1).reshape(cam.H, cam.W).astype(np.float32)
    return cv2.remap(img0, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)


class FloorOdomNode(Node):
    def __init__(self):
        super().__init__("z70_floor_odometry")
        cal = self.declare_parameter("calibration", str(front_calibration_path())).value
        cam_x = float(self.declare_parameter("camera_x_m", initial_camera_x_m()).value)
        options = {name: self.declare_parameter(name, default).value for name, default in
                   [("max_range", 1.5), ("n_features", 3000), ("ransac_thresh_m", .008),
                    ("min_inliers", 80), ("max_rms_m", .003)]}
        self.vo = FloorOdometry(FloorCamera.load(cal), camera_x_m=cam_x, **options)
        self.x = self.y = self.th = 0.0
        self.prev = None
        self.ok = self.rejected = 0
        self.last_stamp = self.get_clock().now().to_msg()
        self.last_rejected = False
        self._pose_uncertain = False
        self._odom_published = False
        self.last_image_ns = 0
        self.last_vo_accept_ns = 0
        self.prev_stamp_ns = 0
        self.yaw_pos_gain, self.yaw_neg_gain, self.forward_gain = 0.70, 0.65, 1.0
        self.pending = []
        self._command_session = self._command_sequence = None
        self._attempts = {}
        self.pub = self.create_publisher(Odometry, "/odom", 10)
        self.pub_status = self.create_publisher(OdometryStatus, "/z70/odom_status", 10)
        self.tf = TransformBroadcaster(self)
        self.frames=MatchedFrames(self,'front',self.on_image,on_session=self.reset_reference)
        self.create_subscription(CalibrationState,'/z70/chassis_calibration',self.on_calibration,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(CommandEvent, '/z70/commands', self.on_command, 256)
        self.create_timer(0.2, self.broadcast_tf)

    def broadcast_tf(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        if self.prev is None or now.nanoseconds - self.last_image_ns > 1_000_000_000:
            self._apply_pending(now.nanoseconds)
        if not self._odom_published:
            self.publish(stamp)
        else:
            self._send_tf(stamp)

    def _send_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(self.th / 2), math.cos(self.th / 2)
        self.tf.sendTransform(t)

    def on_calibration(self, msg: CalibrationState):
        value = calibrated_camera_x(msg)
        if value is not None:
            self.vo.cam_x = value
        for field, attribute, default in [('yaw_positive', 'yaw_pos_gain', .70),
                                          ('yaw_negative', 'yaw_neg_gain', .65),
                                          ('forward', 'forward_gain', 1.0)]:
            gain = getattr(msg, field + '_gain')
            known = getattr(msg, field + '_known')
            setattr(self, attribute, float(gain) if known and math.isfinite(gain) and gain > 0 else default)

    def on_command(self, msg: CommandEvent):
        session, sequence = bytes(msg.evidence.session_id.uuid), msg.evidence.sequence
        if self._command_session == session and self._command_sequence is not None and sequence <= self._command_sequence:
            return
        if self._command_sequence is not None and (session != self._command_session or sequence != self._command_sequence + 1):
            self._attempts.clear()
            self.pending.clear()
        self._command_session, self._command_sequence = session, sequence
        ident = bytes(msg.outcome.attempt_id.uuid)
        if msg.phase == 1 and msg.operation == 'app_rc_move':
            if msg.evidence.source_time_known and msg.evidence.source_clock == 1:
                self._attempts[ident] = stamp_ns(msg.evidence.source_at)
                while len(self._attempts) > 256:
                    self._attempts.pop(next(iter(self._attempts)))
            return
        if msg.phase != 2:
            return
        start_ns = self._attempts.pop(ident, None)
        if start_ns is None or msg.operation != 'app_rc_move' or msg.outcome.dispatch != 1 or msg.outcome.response != 1:
            return
        velocity, omega = msg.requested_twist.linear.x, msg.requested_twist.angular.z
        duration_s = msg.requested_duration.sec + msg.requested_duration.nanosec / 1e9
        if not all(math.isfinite(v) for v in (velocity, omega, duration_s)) or duration_s <= 0:
            return
        motion_end_ns = start_ns + int(round(duration_s * 1e9))
        end_ns = max(stamp_ns(msg.outcome.finished_at), motion_end_ns)
        if self.last_vo_accept_ns >= motion_end_ns:
            return
        if start_ns < self.last_vo_accept_ns < motion_end_ns:
            duration_s = max(0.0, (motion_end_ns - self.last_vo_accept_ns) / 1e9)
            start_ns = self.last_vo_accept_ns
            if duration_s <= 1e-6:
                return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns < end_ns and self.prev is not None and start_ns < self.prev_stamp_ns and self.last_vo_accept_ns < self.prev_stamp_ns:
            pre_end_ns = min(self.prev_stamp_ns, motion_end_ns)
            pre_s = (pre_end_ns - start_ns) / 1e9
            if pre_s > 1e-6:
                self._dead_reckon(dict(velocity=velocity, omega=omega, duration_s=pre_s,
                                       stamp=msg.outcome.finished_at, evidence=msg.evidence),
                                  keep_reference=pre_end_ns == self.prev_stamp_ns)
            start_ns = pre_end_ns
            duration_s = max(0.0, (motion_end_ns - start_ns) / 1e9)
            if duration_s <= 1e-6:
                return
        pulse = dict(start_ns=start_ns, motion_end_ns=motion_end_ns, end_ns=end_ns,
                     velocity=velocity, omega=omega, duration_s=duration_s,
                     stamp=msg.outcome.finished_at, evidence=msg.evidence)
        if now_ns >= end_ns and (self.prev is None or now_ns - self.last_image_ns > 1_000_000_000 or self.last_rejected):
            self._apply_pending(now_ns)
            self._dead_reckon(pulse)
        else:
            self.pending.append(pulse)

    def _trim_pending(self, stamp_ns):
        uncovered = []
        for pulse in self.pending:
            motion_end_ns = pulse['motion_end_ns']
            if motion_end_ns <= stamp_ns:
                continue
            if pulse['start_ns'] < stamp_ns < motion_end_ns:
                pulse['duration_s'] = max(0.0, (motion_end_ns - stamp_ns) / 1e9)
                pulse['start_ns'] = stamp_ns
                if pulse['duration_s'] <= 1e-6:
                    continue
            uncovered.append(pulse)
        self.pending = uncovered

    def _apply_pending(self, now_ns, partial=False, stamp=None):
        self._trim_pending(self.last_vo_accept_ns)
        applied = False
        pending, self.pending = self.pending, []
        for pulse in pending:
            motion_end_ns = pulse['motion_end_ns']
            if partial and pulse['start_ns'] < now_ns < motion_end_ns:
                slice_s = (now_ns - pulse['start_ns']) / 1e9
                if slice_s > 1e-6:
                    self._dead_reckon({**pulse, 'duration_s': slice_s, 'stamp': stamp or pulse['stamp']})
                    applied = True
                rem_s = max(0.0, (motion_end_ns - now_ns) / 1e9)
                if rem_s > 1e-6:
                    pulse['start_ns'] = now_ns
                    pulse['duration_s'] = rem_s
                    self.pending.append(pulse)
                continue
            age_ns = now_ns - pulse['end_ns']
            if age_ns < 0:
                self.pending.append(pulse)
            elif age_ns <= 2_000_000_000:
                self._dead_reckon(pulse)
                applied = True
        return applied

    def _publish_status(self, stamp, evidence, validity_value, reason, pair_accepted=False, calibration_id=''):
        quality = OdometryStatus()
        quality.header.stamp = stamp
        quality.header.frame_id = 'odom'
        quality.evidence = evidence
        quality.calibration_id = calibration_id
        quality.pair_accepted = pair_accepted
        quality.validity.value = validity_value
        quality.rejection = quality.validity.reason = reason
        quality.covariance_calibrated = False
        quality.active_camera_x_known = math.isfinite(self.vo.cam_x)
        quality.active_camera_x_m = self.vo.cam_x if quality.active_camera_x_known else 0.
        self.pub_status.publish(quality)

    def _dead_reckon(self, pulse, keep_reference=False):
        omega, duration = pulse['omega'], pulse['duration_s']
        yaw_gain = self.yaw_pos_gain if omega >= 0 else self.yaw_neg_gain
        angle = omega * duration * yaw_gain
        distance = pulse['velocity'] * duration * self.forward_gain
        arc_x, arc_y = _arc(angle)
        c, s = math.cos(self.th), math.sin(self.th)
        self.x += distance * (c * arc_x - s * arc_y)
        self.y += distance * (s * arc_x + c * arc_y)
        self.th = math.atan2(math.sin(self.th + angle), math.cos(self.th + angle))
        if not keep_reference:
            self.reset_reference()
        self.last_rejected = True
        self._pose_uncertain = True
        self.publish(pulse['stamp'], rejected=True)
        self._publish_status(pulse['stamp'], pulse['evidence'], 2, 'COMMAND_DEAD_RECKONED')

    def reset_reference(self):
        self.prev = None
        self.prev_stamp_ns = 0

    def on_image(self, msg: Image):
        img = decode_bgr8(msg, self.vo.cam)
        if img is None:
            return
        now_stamp_ns = stamp_ns(msg)
        self.last_image_ns = now_stamp_ns
        state = "init"
        stationary = False
        def set_ref():
            self.prev = img.copy()
            self.prev_stamp_ns = now_stamp_ns
        if self.prev is None:
            if self._apply_pending(now_stamp_ns, partial=True, stamp=msg.header.stamp):
                set_ref()
                return
        else:
            m = self.vo.estimate(self.prev, img)
            if m is None:
                self.rejected += 1
                self.last_rejected = True
                if self._apply_pending(now_stamp_ns, partial=True, stamp=msg.header.stamp):
                    set_ref()
                    return
                if any(s < now_stamp_ns for s in self._attempts.values()):
                    set_ref()
                state = "rejected"
            else:
                self.last_vo_accept_ns = now_stamp_ns
                self.last_rejected = False
                self._pose_uncertain = False
                self._trim_pending(now_stamp_ns)
                stationary = math.hypot(m.dx, m.dy) < 5e-4 and abs(m.dtheta) < 1e-3
                if not stationary:
                    c, s = math.cos(self.th), math.sin(self.th)
                    self.x += c * m.dx - s * m.dy
                    self.y += s * m.dx + c * m.dy
                    self.th = math.atan2(math.sin(self.th + m.dtheta), math.cos(self.th + m.dtheta))
                self.ok += 1
                state = "ok"
        if state != "rejected":
            if self.prev is None or not stationary:
                self.prev = img.copy()
            self.prev_stamp_ns = now_stamp_ns
        self.publish(msg.header.stamp, rejected=(state == "rejected" or self._pose_uncertain))
        accepted = state == 'ok'
        self._publish_status(
            msg.header.stamp,
            self.frames.metadata.evidence,
            1 if accepted else (4 if state == 'rejected' else 0),
            '' if accepted else ('VO_REJECTED' if state == 'rejected' else 'INITIAL_REFERENCE'),
            pair_accepted=accepted,
            calibration_id=self.frames.metadata.calibration_id,
        )

    def publish(self, stamp, rejected=False):
        qz, qw = math.sin(self.th / 2), math.cos(self.th / 2)
        o = Odometry()
        o.header.stamp = stamp; o.header.frame_id = "odom"; o.child_frame_id = "base_footprint"
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        if rejected:
            for index in [0, 7, 35]:
                o.pose.covariance[index] = 1e6
        self.pub.publish(o)
        self.last_stamp = stamp
        self._odom_published = True
        self._send_tf(stamp)


def main():
    rclpy.init()
    node = FloorOdomNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node(); rclpy.try_shutdown()


class FloorMosaic:
    def __init__(self, cam, resolution=.005, size_m=4., max_range=1.2, camera_x_m=None):
        if camera_x_m is None: camera_x_m = initial_camera_x_m()
        if resolution <= 0 or size_m <= 0 or max_range <= .06:
            raise ValueError('positive map size/resolution and range > 0.06 required')
        self.cam, self.resolution, self.max_range, self.cam_x = cam, resolution, max_range, camera_x_m
        self.n = int(math.ceil(size_m / resolution))
        self.origin = (-self.n * resolution / 2, -self.n * resolution / 2)
        self.mean = np.zeros((self.n, self.n, 3), np.float32)
        self.count = np.zeros((self.n, self.n), np.uint32)
        yy, xx = np.meshgrid(np.arange(self.n), np.arange(self.n), indexing='ij')
        self.world = np.stack([self.origin[0]+(xx.ravel()+.5)*resolution,
                               self.origin[1]+(yy.ravel()+.5)*resolution], 1)

    def update(self, image, pose):
        if image.shape != (self.cam.H, self.cam.W, 3):
            raise ValueError('image dimensions must match camera calibration')
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        R = np.array([[c, -s], [s, c]])
        camera_origin = np.array([x, y]) + R @ np.array([self.cam_x, 0])
        local = (self.world - camera_origin) @ R
        r = np.linalg.norm(local, axis=1)
        candidate = np.flatnonzero((r > .06) & (r < self.max_range) & (local[:, 0] > 0))
        uv, ok = self.cam.floor_to_pixels(local[candidate])
        ok &= (uv[:, 0] >= 1) & (uv[:, 0] < self.cam.W - 2) & (uv[:, 1] >= 1) & (uv[:, 1] < self.cam.H - 2)
        ids = candidate[ok]; uv = uv[ok].astype(np.float32)
        sample = np.empty((len(ids), 3), np.float32)
        for start in range(0, len(ids), 30000):
            part = uv[start:start+30000]
            sample[start:start+len(part)] = cv2.remap(
                image, part[:, 0].reshape(-1, 1), part[:, 1].reshape(-1, 1), cv2.INTER_LINEAR).reshape(-1, 3)
        supported = np.any(sample != 0, axis=1)
        ids, sample = ids[supported], sample[supported]
        count = self.count.ravel(); mean = self.mean.reshape(-1, 3)
        overlap = count[ids] > 0
        ncc = None
        if overlap.sum() > 20:
            a = mean[ids[overlap]].mean(1); b = sample[overlap].mean(1)
            a = a - a.mean(); b = b - b.mean(); denom = np.linalg.norm(a) * np.linalg.norm(b)
            if denom > 1e-6: ncc = float(a @ b / denom)
        mean[ids] += (sample - mean[ids]) / (count[ids, None] + 1)
        count[ids] += 1
        return {'observed_cells': int(len(ids)), 'overlap_cells': int(overlap.sum()),
                'overlap_ncc': ncc, 'total_cells': int(np.count_nonzero(count))}

    def image(self):
        return np.clip(np.rint(self.mean), 0, 255).astype(np.uint8)


class FloorMosaicNode(Node):
    def __init__(self):
        super().__init__('z70_floor_mosaic')
        cam = FloorCamera.load(self.declare_parameter('calibration', str(front_calibration_path())).value)
        params = {name: float(self.declare_parameter(name, value).value) for name, value in
                  [('resolution', .005), ('size_m', 4.), ('max_range', 1.2), ('camera_x_m', initial_camera_x_m())]}
        self.mosaic = FloorMosaic(cam, **params)
        self.images = OrderedDict(); self.odoms = OrderedDict()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Image, '/z70/floor_mosaic', qos)
        self.mask_pub = self.create_publisher(Image, '/z70/floor_mosaic_mask', qos)
        self.info_pub = self.create_publisher(CameraInfo, '/z70/camera_info', qos)
        self.status_pub = self.create_publisher(String, '/z70/floor_mosaic_status', 10)
        self.frames = MatchedFrames(self, 'front', lambda m: self.add(self.images, m))
        self.create_subscription(Odometry, '/odom', lambda m: self.add(self.odoms, m), 10)
        self.create_subscription(CalibrationState, '/z70/chassis_calibration', self.on_calibration, qos)
        self._publish_outputs(self.get_clock().now().to_msg())

    def _camera_info(self, stamp):
        fx = fy = 0.01 / self.mosaic.resolution
        cx = cy = self.mosaic.n / 2.0
        info = CameraInfo()
        info.header.stamp = stamp; info.header.frame_id = 'odom'
        info.width = info.height = self.mosaic.n
        info.distortion_model = 'plumb_bob'
        info.d = [0., 0., 0., 0., 0.]
        info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
        info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        info.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
        return info

    def _publish_outputs(self, stamp):
        self.info_pub.publish(self._camera_info(stamp))
        for data, encoding, pub in [(self.mosaic.image(), 'bgr8', self.pub),
                                    ((self.mosaic.count > 0).astype(np.uint8) * 255, 'mono8', self.mask_pub)]:
            out = Image(); out.header.stamp = stamp; out.header.frame_id = 'odom'
            out.height, out.width = data.shape[:2]; out.encoding = encoding
            out.step = out.width * (3 if encoding == 'bgr8' else 1); out.data = data.tobytes(); pub.publish(out)

    def on_calibration(self, msg):
        value = calibrated_camera_x(msg)
        if value is not None: self.mosaic.cam_x = value

    def add(self, cache, msg):
        key = stamp_ns(msg); cache[key] = msg
        while len(cache) > 20: cache.popitem(last=False)
        if key not in self.images or key not in self.odoms: return
        image = self.images.pop(key); odom = self.odoms.pop(key)
        if odom.header.frame_id != 'odom' or odom.pose.covariance[0] > 1000: return
        view = decode_bgr8(image, self.mosaic.cam)
        if view is None: return
        pose = odom.pose.pose
        stats = self.mosaic.update(view, (pose.position.x, pose.position.y, quat_yaw(pose.orientation)))
        self._publish_outputs(image.header.stamp)
        stats.update(stamp_ns=key, resolution=self.mosaic.resolution, origin_xy=list(self.mosaic.origin),
                     frame_id='odom', width=self.mosaic.n, height=self.mosaic.n)
        self.status_pub.publish(String(data=json.dumps(stats)))


def mosaic_main():
    rclpy.init(); node = FloorMosaicNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally: node.destroy_node(); rclpy.try_shutdown()


def calibrator_main():
    class Calibrator(Node):
        def __init__(self):
            super().__init__('chassis_calibrator')
            path = self.declare_parameter('output_file', str(runtime_calibration_path())).value
            seeds = self.declare_parameter('seed_directory', str(seed_directory())).value
            self.estimator = OnlineCalibration(
                path, settle_s=float(self.declare_parameter('settle_s', 2.).value),
                max_gap_s=float(self.declare_parameter('max_gap_s', 1.5).value),
                min_camera_n=int(self.declare_parameter('min_camera_n', 4).value),
                max_camera_sigma_m=float(self.declare_parameter('max_camera_sigma_m', .01).value),
            )
            if seeds:
                self.estimator.seed(seeds)
            else:
                self.estimator.save()
            self.publication = CalibrationPublication()
            self.command_stream = CommandStream(self.estimator)
            self.status = {}
            self.pub = self.create_publisher(CalibrationState, '/z70/chassis_calibration', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(CommandEvent, '/z70/commands', self.commands, 256)
            self.create_subscription(Odometry, '/odom', self.odom, 100)
            self.create_subscription(OdometryStatus, '/z70/odom_status', self.quality, 100)
            self.create_timer(.2, self.update)

        def commands(self, msg):
            self.command_stream.add(msg)

        def odom(self, msg):
            p = msg.pose.pose; stamp = stamp_ns(msg); t = stamp / 1e9
            entry = dict(t=t, stamp_ns=stamp, x=p.position.x, y=p.position.y, yaw=quat_yaw(p.orientation),
                         rejected=msg.pose.covariance[0] > 1000, camera_x_m=None)
            status = self.status.get(stamp)
            if status:
                entry['camera_x_m'] = status.get('camera_x_m')
            self.estimator.add_pose(entry)

        def quality(self, msg):
            if not msg.evidence.valid or msg.evidence.stale:
                return
            stamp = stamp_ns(msg)
            camera_x = msg.active_camera_x_m if msg.active_camera_x_known and math.isfinite(msg.active_camera_x_m) else None
            status = dict(stamp_ns=stamp, camera_x_m=camera_x, state='ok' if msg.pair_accepted else 'rejected')
            self.status[stamp] = status
            while len(self.status) > 1000:
                self.status.pop(next(iter(self.status)))
            for p in reversed(self.estimator.poses):
                if p.get('stamp_ns') == stamp:
                    p['camera_x_m'] = status.get('camera_x_m'); p['rejected'] = p['rejected'] or status.get('state') == 'rejected'; break

        def update(self):
            self.estimator.process(self.get_clock().now().nanoseconds / 1e9)
            self.estimator.save()
            self.pub.publish(self.publication.snapshot(self.estimator, self.get_clock().now().to_msg()))

    rclpy.init(); node = Calibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.estimator.save(); node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    calibrator_main()
