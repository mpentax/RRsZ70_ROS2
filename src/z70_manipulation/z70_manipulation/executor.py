"""Discrete manipulation executor; all execution and observations use ROS."""
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import time
import cv2
import numpy as np
import requests
from z70_description.arm_model import LIMITS
from . import perception, planner

TRACK_GATE_M = .12
ACQUIRE_MAX_RANGE_M = 1.5
OBJECT_SIZE_M = .066   # the yellow dodecahedron, measured
ACQUIRE_MIN_PX = 30
CENTRE_TOL_M = .02


class UncertainMotion(RuntimeError):
    pass


class SafetyStop(RuntimeError):
    pass


def read_status(observations) -> dict:
    fn = getattr(observations, 'status', None)
    return fn() if callable(fn) else observations.get('/api/state')['telemetry']['status']['value']


def read_arm(observations) -> dict:
    fn = getattr(observations, 'arm_estimate', None)
    return fn() if callable(fn) else observations.get('/api/arm/estimate')


def yaw_gain(gains: dict | None, angle: float, default: float = 1.) -> float:
    return float((gains or {}).get('yaw_positive' if angle > 0 else 'yaw_negative', {}).get('gain') or default)


def forward_gain(gains: dict | None, default: float = 1.) -> float:
    return float((gains or {}).get('forward', {}).get('gain') or default)


def _safe_approach(position, target=None):
    try:
        return planner.approach(position, target)
    except ValueError:
        return None


def log_directory(repo):
    return Path(repo) / '.local/pickup' / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ') + '-ros')


class Executor:
    def __init__(self, client, observe, log_dir, sleep=time.sleep, hazard=lambda: None, odom=lambda: None, feedback=lambda _: None, calibration=None):
        self.client = client
        self.observe = observe
        self.directory = Path(log_dir)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.sleep = sleep
        self.hazard = hazard
        self.odom = odom
        self.feedback = feedback
        self.gains = calibration or {}
        self.seq = 0
        self.rc = False
        self.deployed = False
        self.gripped = False
        self.stopped = False
        self.last_motion = None
        self.pregrip_jaw = None
        self.last_floor_gone = False
        self.pregrip_object_in_hand = None
        self.evidence_prefix = ''
        self.placement_sequence = 0

    def log(self, event, **data):
        row = dict(t=datetime.now(timezone.utc).isoformat(), event=event, **data)
        with (self.directory / 'timeline.jsonl').open('a') as f:
            f.write(json.dumps(row, allow_nan=False) + '\n')
        self.feedback(event)

    def command(self, method, params=None, timeout_ms=5000):
        params = [] if params is None else params
        self.log('command', method=method, params=params)
        try:
            d = self.client.rpc(method, params, timeout_ms)
        except (UncertainMotion, requests.RequestException) as e:
            self.log('data_gap', method=method, kind=type(e).__name__)
            if method in ('app_rc_move', 'app_arm_move'):
                return None  # Observe before subsequent motion; never blind replay.
            raise
        self.log('reply', method=method, dispatch=d.get('dispatch'), response=d.get('response'), transportReason=d.get('transportReason'))
        return d

    def _status(self) -> dict:
        return read_status(self.client) or {}

    def _arm(self) -> dict:
        return read_arm(self.client)

    def _yaw_gain(self, angle: float, default: float = .68) -> float:
        return yaw_gain(self.gains, angle, default)

    def _forward_gain(self, default: float = .65) -> float:
        return forward_gain(self.gains, default)

    def safety(self):
        reason = self.hazard()
        if reason:
            raise SafetyStop(reason)
        try:
            status = self._status()
        except (requests.RequestException, LookupError):
            self.log('data_gap', kind='state')
            return
        if status.get('error_code', 0):
            raise SafetyStop('robot error: ' + str(status['error_code']))
        if status.get('in_cleaning') or status.get('in_returning'):
            raise SafetyStop('unexpected clean/dock')
        if status.get('battery', 100) <= int(os.environ.get('Z70_MIN_BATTERY', '15')):
            raise LookupError('battery at charging threshold; charge before pickup')

    def objects(self, camera, color):
        for _ in range(3):
            self.safety()
            try:
                objects = self.observe(camera, color)
                if objects is not None:
                    self.log('observation', camera=camera, color=color, objects=objects)
                    return objects
            except (requests.RequestException, ValueError, LookupError):
                self.log('data_gap', kind='camera', camera=camera)
            self.sleep(5.)
        raise LookupError('no fresh camera observation after retries')

    def target(self, camera, color, previous=None):
        for _ in range(12):
            objects = self.objects(camera, color)
            if objects:
                if previous is None:
                    sane = [b for b in objects if b.get('size_px', 0) >= ACQUIRE_MIN_PX and
                            ('position' not in b or math.hypot(*b['position'][:2]) <= ACQUIRE_MAX_RANGE_M)]
                    if sane:
                        again = [b for b in (self.objects(camera, color) or []) if b.get('size_px', 0) >= ACQUIRE_MIN_PX]
                        confirmed = [b for b in sane if 'position' not in b or any('position' in a and
                                     math.dist(a['position'][:2], b['position'][:2]) <= .05 for a in again)]
                        if confirmed:
                            return min(confirmed, key=lambda b: abs(b.get('size_m', OBJECT_SIZE_M) - OBJECT_SIZE_M) + .05 * math.hypot(*b.get('position', [0, 0])[:2]))
                        self.log('data_gap', kind='acquisition_unconfirmed', seen=[b.get('position') for b in sane][:4])
                        self.sleep(.5)
                        continue
                    self.log('data_gap', kind='no_plausible_target', seen=[(b.get('position'), b.get('size_px')) for b in objects][:4])
                    self.sleep(.5)
                    continue
                near = [b for b in objects if math.dist(b.get('position', [9, 9])[:2], previous[:2]) <= TRACK_GATE_M]
                if near:
                    return min(near, key=lambda b: math.dist(b['position'][:2], previous[:2]))
                self.log('data_gap', kind='target_outside_gate', predicted=previous[:2], seen=[b.get('position') for b in objects][:4])
            else:
                self.log('data_gap', kind='target_not_visible')
            self.sleep(.5)
        raise LookupError('target not visible; no blind movement')

    def fresh_frame(self):
        """Read a fresh camera frame with bounded retry."""
        for attempt in range(3):
            self.safety()
            try:
                return self.client.frame()
            except (requests.RequestException, ValueError, LookupError):
                self.log('data_gap', kind='camera_frame', attempt=attempt + 1)
                if attempt == 2:
                    raise
                self.sleep(5.)
        raise LookupError('no camera frame after retries')

    def snap(self, name):
        """Save the current frame as evidence (best effort, never fails the task)."""
        try:
            image, tag = self.fresh_frame()
            if image is not None:
                name = self.evidence_prefix + name
                cv2.imwrite(str(self.directory / f'{name}-{tag}.jpg'), image)
                self.log('snapshot', name=name, camera=tag)
        except SafetyStop:
            raise
        except Exception as e:
            self.log('data_gap', kind='snapshot', error=repr(e)[:80])

    def predict(self, p):
        """Target position in the current base frame after the last pulse (odometry), or unchanged without odom."""
        return perception.predict(p, self.last_motion) if getattr(self, 'last_motion', None) else list(p)

    def start_rc(self):
        if not self.rc:
            self.command('app_rc_start')
            self.rc = True
            self.sleep(5.)

    def end_rc(self):
        if self.rc:
            try:
                self.command('app_rc_stop')
            finally:
                try:
                    self.command('app_rc_end')
                finally:
                    self.rc = False
                    if not self.deployed and not self.gripped:
                        try:
                            self.command('app_pick_up_enter_exit', {'flag': 0})
                        except Exception:
                            pass

    def pulse(self, velocity=0., omega=0., duration=500):
        self.safety()
        before = self.odom()
        self.start_rc()
        self.safety()
        self.seq += 1
        self.command('app_rc_move', [dict(velocity=velocity, omega=omega, duration=int(duration), seqnum=self.seq)])
        self.sleep(duration / 1000 + .7)
        self.command('app_rc_stop')
        after = self.odom()
        self.last_motion = None
        if before and after:
            c, sn = math.cos(before['yaw']), math.sin(before['yaw'])
            dx, dy = after['x'] - before['x'], after['y'] - before['y']
            dth = math.atan2(math.sin(after['yaw'] - before['yaw']), math.cos(after['yaw'] - before['yaw']))
            self.last_motion = (c * dx + sn * dy, -sn * dx + c * dy, dth)
            self.log('odom_delta', distance_m=math.hypot(dx, dy), yaw_rad=abs(dth))
        else:
            self.log('data_gap', kind='odom')

    def align_front(self, color, target=None):
        try:
            for _ in range(40):
                blob = self.target('front', color, self.predict(target['position']) if target else None)
                target = blob
                plan = planner.approach(blob['position'])
                self.log('approach', **plan)
                if abs(plan['bearing_error_deg']) <= 2 and abs(plan['range_error_m']) <= .015:
                    blob['bearing_error_deg'] = plan['bearing_error_deg']
                    blob['range_error_m'] = plan['range_error_m']
                    return blob
                if abs(plan['bearing_error_deg']) > 2:
                    gain = self._yaw_gain(plan['yaw'], .72 if plan['yaw'] > 0 else .64)
                    omega = math.copysign(.22, plan['yaw'])
                    ms = min(700, max(120, abs(plan['yaw']) / (abs(omega) * gain) * 1000))
                    self.pulse(omega=omega, duration=ms)
                else:
                    gain = self._forward_gain(.65)
                    velocity = math.copysign(.07, plan['forward'])
                    ms = min(800, max(120, abs(plan['forward']) / (abs(velocity) * gain) * 1000))
                    self.pulse(velocity=velocity, duration=ms)
            raise LookupError('approach did not converge; observations retained')
        finally:
            self.end_rc()

    def deploy(self, settle=True):
        status = self._status()
        common = int(status.get('common_status', 0))
        if ((common >> 6) & 15) in (2, 3):
            self.deployed = True
            self.last_motion = None
            self.log('deploy_already_ready', common_status=common)
            return dict(ready=True, state='WORKING_POS')
        self.pre_deploy_odom = self.odom()
        self.command('app_pick_up_enter_exit', {'flag': 1})
        self.sleep(4.)
        self.command('app_arm_out_compartment', {})
        for i in range(60):
            self.safety()
            status = self._status()
            common = int(status.get('common_status', 0))
            action_bits = (common >> 6) & 15
            grab_mode = (common >> 10) & 3
            if (common & 256) or (action_bits in (1, 2, 3, 4) and grab_mode in (1, 2)):
                self.deployed = True
                break
            if i == 16 and action_bits == 0:
                # Vendor clutter check refused unstow ("move the robot to a less cluttered area").
                # Back up ~15 cm to clear foreground clutter and reissue unstow.
                self.log('deploy_clutter_backoff_and_reissue', common_status=common)
                self.pulse(velocity=-0.08, duration=800)
                self.pulse(velocity=-0.08, duration=800)
                self.end_rc()
                self.command('app_pick_up_enter_exit', {'flag': 1})
                self.sleep(2.)
                self.command('app_arm_out_compartment', {})
            self.sleep(.5)
        else:
            raise LookupError('arm readiness not observed')
        if settle:
            self.sleep(1.)
            self.safety()
        self.last_motion = None
        return dict(ready=True, state='WORKING_POS')

    def post_deploy_align(self, color):
        self.last_motion = None
        target = self.target('front', color)
        after = self.odom()
        before = getattr(self, 'pre_deploy_odom', None)
        yaw = None if not before or not after else math.atan2(math.sin(after['yaw'] - before['yaw']), math.cos(after['yaw'] - before['yaw']))
        plan = planner.approach(target['position'])
        visual_yaw = None
        if getattr(self, 'pre_deploy_target', None):
            a = self.pre_deploy_target
            b = target['position']
            visual_yaw = math.atan2(math.sin(math.atan2(a[1], a[0]) - math.atan2(b[1], b[0])), math.cos(math.atan2(a[1], a[0]) - math.atan2(b[1], b[0])))
        self.log('post_deploy_alignment', yaw_offset_rad=yaw, visual_yaw_offset_rad=visual_yaw, target=target['position'],
                 bearing_error_deg=plan['bearing_error_deg'], range_error_m=plan['range_error_m'],
                 prior_reset=True)
        self.snap('0-post-deploy')
        return self.align_front(color, target)

    def jog(self, step):
        """Step arm joints using kinematic model + RPC receipt (no mandatory camera gate)."""
        self.safety()
        reply = self.command('app_arm_move', {'m': step['m']})
        if reply is None:
            raise LookupError('arm step outcome unknown; observe before continuing')
        self.sleep(0.85)
        self.log('arm_jog', state=step['state'], m=step['m'])

    def move_arm(self, goal_joints, start_joints=None):
        """Plan and execute coordinated 10-degree M3/M4/M5 steps within the legal kinematic envelope."""
        goal = (round(goal_joints[0]), round(goal_joints[1]), round(goal_joints[2]) if len(goal_joints) > 2 else 0)
        if start_joints is None:
            est = self._arm()
            if est.get('state') in ('UNKNOWN', 'IN_ROBOT'):
                raise LookupError(f"cannot plan arm steps from state {est.get('state')}")
            j = est.get('joints') or {}
            start = (round(j.get('m3', 150)), round(j.get('m4', 45)), round(j.get('m5', 0)))
        else:
            start = (round(start_joints[0]), round(start_joints[1]), round(start_joints[2]) if len(start_joints) > 2 else 0)
        steps = planner.trajectory(start, goal)
        for step in steps:
            self.jog(step)
        return dict(state=list(goal), steps_executed=len(steps))

    def front_measure(self, color, previous):
        """Object and jaw are measured from the same fresh front exposure when visible."""
        image, tag = self.fresh_frame()
        if tag != 'front':
            raise LookupError('front camera not selected')
        jaw = perception.front_jaw(image)
        blobs = [b for b in perception.front_objects(image) if b['color'] == color and math.dist(b['position'][:2], previous[:2]) <= TRACK_GATE_M]
        if not blobs:
            raise LookupError('target missing from jaw measurement frame')
        blob = min(blobs, key=lambda b: math.dist(b['position'][:2], previous[:2]))
        return dict(image=image, jaw=jaw, object=blob)

    def front_servo(self, color, current, floor_spot):
        """Fine-align using front camera if marker ID68 is visible; do not abort if marker is out of frame."""
        current = tuple(current[:2])
        confirmed = 0
        for index in range(16):
            self.safety()
            try:
                sample = self.front_measure(color, floor_spot)
            except LookupError:
                self.log('data_gap', kind='front_jaw_unobservable', state=list(current))
                self.end_rc()
                return dict(position=floor_spot), current, floor_spot
            jaw = np.array(sample['jaw']['position'])
            blob = sample['object']
            floor_spot = list(blob['position'])
            equator = np.array(floor_spot)
            equator[2] = OBJECT_SIZE_M / 2
            error = equator - jaw
            clearance = planner.hand.lowest(planner.hand.transform_bodies(sample['jaw']['hand_pose_base'], 90)) - sample['jaw']['uncertainty_bound_m']
            distance = float(np.linalg.norm(error))
            cv2.imwrite(str(self.directory / f'{self.evidence_prefix}jaw-servo-{index:02d}-front.jpg'), sample['image'])
            self.log('front_jaw_check', state=list(current), jaw=jaw.tolist(), object_equator=equator.tolist(), error_m=error.tolist(), distance_m=distance, clearance_bound_m=clearance)
            self.pregrip_jaw = sample['jaw']['position']
            hand_pose = np.asarray(sample['jaw']['hand_pose_base'])
            self.pregrip_object_in_hand = (hand_pose[:3,:3].T @ (equator-hand_pose[:3,3])).tolist()
            self.log('pregrip_geometry', object_in_hand_m=self.pregrip_object_in_hand,
                     source='same_exposure_floor_equator_and_ID68', uncertainty_m=sample['jaw']['uncertainty_bound_m'])
            if math.hypot(error[0], error[1]) <= .015 and abs(error[2]) <= .020:
                confirmed += 1
                if confirmed >= 2:
                    self.end_rc()
                    return blob, current, floor_spot
                continue
            confirmed = 0
            horizontal = math.hypot(error[0], error[1]) > .012
            if horizontal and abs(error[1]) > .004:
                angle = math.atan2(error[1], max(equator[0], .1))
                gain = self._yaw_gain(angle, .68)
                self.pulse(omega=math.copysign(.16, angle), duration=min(350, max(100, abs(angle) / (.16 * gain) * 1000)))
                floor_spot = self.predict(floor_spot)
                continue
            if horizontal and abs(error[0]) > .004:
                gain = self._forward_gain(.65)
                self.pulse(velocity=math.copysign(.03, error[0]), duration=min(400, max(100, min(.008, abs(error[0])) / (.03 * gain) * 1000)))
                floor_spot = self.predict(floor_spot)
                continue
            choices = []
            for (d3, c3) in ((0, 0), (10, 1), (-10, 2)):
                for (d4, c4) in ((0, 0), (10, 1), (-10, 2)):
                    if c3 == 0 and c4 == 0:
                        continue
                    st = (current[0] + d3, current[1] + d4)
                    if not planner.safe(st) or not planner.safe_edge(current, st):
                        continue
                    candidate = jaw + np.array(planner.operational_point(st)) - np.array(planner.operational_point(current))
                    if candidate[2] - .015 < .015:
                        continue
                    err = float(np.linalg.norm(equator - candidate))
                    if err >= distance - .003:
                        continue
                    choices.append((err, st, [c3, c4, 0]))
            if not choices:
                self.end_rc()
                return blob, current, floor_spot
            _, st, m = min(choices)
            self.end_rc()
            self.jog(dict(m=m, state=list(st)))
            current = st
        self.end_rc()
        return dict(position=floor_spot), current, floor_spot

    def held_front(self, color, floor_spot):
        """Verify colour lies at elevated observed jaws or pregrip lift column, and floor spot is empty."""
        image, tag = self.fresh_frame()
        if tag != 'front':
            raise LookupError('front verification frame expected')
        cam = perception.front_camera()
        jaw_observed = True
        try:
            J = np.array(perception.front_jaw(image)['position'])
        except LookupError:
            if getattr(self, 'pregrip_jaw', None) is None:
                J = np.array(planner.measured_point(planner.GRASP, planner._CALIBRATION['effective_jaw_from_elbow_m']))
            else:
                J = np.array(self.pregrip_jaw)
            jaw_observed = False
        held = []
        camera_x = perception.calibration()['front_camera_x_m']
        for b in perception.detect(image):
            if b['color'] != color:
                continue
            ray = cam.pixel_to_ray(*b['pixel'])
            C = np.array([camera_x, 0, cam.h / 1000])
            if ray[0] <= 0:
                continue
            X = C + ray * (J[0] - camera_x) / ray[0]
            associated = (J[2] > .070 and np.linalg.norm(X - J) < .055) if jaw_observed else (abs(X[1] - J[1]) < .04 and .05 < X[2] - J[2] < .16)
            if X[2] > .066 and associated:
                held.append(dict(pixel=b['pixel'], object_at_jaw_depth=X.tolist(), distance_to_jaw_m=float(np.linalg.norm(X - J))))
        cv2.imwrite(str(self.directory / f'held-{len(list(self.directory.glob("held-*.jpg"))):02d}-front.jpg'), image)
        floor = perception.front_objects(image)
        floor_gone = not any(b['color'] == color and math.dist(b['position'][:2], floor_spot[:2]) < .055 for b in floor)
        self.last_floor_gone = floor_gone
        self.log('front_held_check', jaw=J.tolist(), jaw_observed=jaw_observed, depth_source='observed_jaw' if jaw_observed else 'pregrip_lift_column', held=bool(held), candidates=held, floor_gone=floor_gone)
        return bool(held) and floor_gone

    def grasp_and_lift(self, lift_steps=2, verify_color='', floor_spot=None):
        """Close jaws, run upward hand steps, and optionally verify hold in the front camera."""
        self.end_rc()
        self.snap('1-pregrip')
        if self.pregrip_jaw is None:
            self.pregrip_jaw = planner.measured_point(planner.GRASP, planner._CALIBRATION['effective_jaw_from_elbow_m'])
        self.gripped = True
        try:
            self.command('app_grip', {}, timeout_ms=500)
        except (UncertainMotion, requests.RequestException):
            self.log('data_gap', kind='grip_outcome_observe_before_deciding')
        self.sleep(2.)
        for lift_step in range(int(lift_steps)):
            try:
                self.command('app_arm_direction_move', {'direction': 1}, timeout_ms=3000)
            except (UncertainMotion, requests.RequestException):
                self.log('data_gap', kind='lift_outcome')
            self.sleep(2.5)
            self.snap('2-lift-step-' + str(lift_step + 1))
        self.snap('2-lifted')
        verified = False
        if verify_color:
            spot = floor_spot if floor_spot is not None else planner.grasp_point()
            held_ok = self.held_front(verify_color, spot)
            self.snap('3-front-verify')
            self.log('hold_start', initial_held=held_ok, odom=self.odom())
            self.sleep(10.)
            held_after = self.held_front(verify_color, spot)
            self.snap('4-held-10s')
            verified = bool(held_ok and held_after)
            self.log('hold_10s', held=verified, final_held=held_after, odom=self.odom())
            self.log('verification', held=verified, floor_gone=self.last_floor_gone, verified=verified, floor_spot=spot)
        return dict(gripped=True, lift_steps=int(lift_steps), verified=verified)

    def current_arm_joints(self):
        """Usable ROS pose, otherwise a fresh marker-backed conditional fit."""
        estimate = self._arm()
        j = estimate.get('joints') or {}
        if estimate.get('state') not in ('UNKNOWN','IN_ROBOT') and all(k in j for k in ('m3','m4','m5')):
            return [j['m3'],j['m4'],j['m5']]
        image, tag = self.fresh_frame()
        if tag != 'front':
            raise LookupError('unknown arm pose; front marker observation needed')
        fit = planner.observed_joints(perception.front_jaw(image)['hand_pose_base'])
        self.log('arm_pose_recovery', fit=fit, estimator_anchored=False)
        if fit is None:
            raise LookupError('observed hand pose cannot resolve current nominal joints')
        return fit['joints']

    def placement_sample(self, color, support_color, base_color, tower_spot, *, released=False, after=0.):
        """Capture and measure one front-camera placement frame."""
        if after:
            self.fresh_frame()  # discard the possibly pre-command buffered image
        for _ in range(4):
            image, tag = self.fresh_frame()
            received = getattr(self.client, 'frame_received_at', None)
            if after and (received is None or received <= after):
                self.log('data_gap', kind='placement_frame_precedes_command', received_at=received, after=after)
                continue
            if tag != 'front':
                raise LookupError('front placement frame expected')
            sample = planner.measure(image, color, support_color, base_color, tower_spot,
                                       getattr(self, 'pregrip_object_in_hand', None), released=released)
            sample['frame_received_at'] = received
            sample['exposure_time_known'] = getattr(self.client, 'frame_exposure_known', False)
            sample['timing_basis'] = 'host_receipt_after_settle_and_buffer_drain' if after else 'same_exposure'
            name = f'{getattr(self, "evidence_prefix", "")}placement-{self.placement_sequence:03d}'
            self.placement_sequence += 1
            cv2.imwrite(str(self.directory / f'{name}-front.jpg'), image)
            self.log('placement_measurement', **sample)
            return sample
        raise LookupError('no placement frame received after command/settling barrier')

    def supported_place(self, color, support_color, base_color, tower_spot, tier):
        """Align the held column over the support object, descend to contact, release, and verify stability."""
        sample = None
        def incomplete(detail):
            result = dict(verified=False, detached=False, reason='supported-placement-unresolved',
                          detail=detail, tier=tier, evidence=sample)
            self.log('placement_incomplete', **result)
            return result
        try:
            contacts = 0
            previous_descent = None
            motion_after = 0.
            for index in range(48):
                self.safety()
                sample = self.placement_sample(color, support_color, base_color, tower_spot, after=motion_after)
                motion_after = 0.
                tower_spot = list(sample['tower_spot'])
                if previous_descent is not None:
                    observed_drop = previous_descent['gap_m']-sample['gap_m']
                    self.log('placement_step_observed', predicted_m=previous_descent['descent_m'], observed_gap_change_m=observed_drop)
                    if observed_drop < .002:
                        return incomplete('descent outcome unresolved; no blind replay')
                    previous_descent = None
                error = np.asarray(sample['lateral_error_m'])
                gap = sample['gap_m']
                if gap < -.005:
                    return incomplete('silhouettes overlap; support depth/contact ambiguous')
                if np.linalg.norm(error) > planner.LATERAL_TOL_M:
                    contacts = 0
                    if gap < .008:
                        return incomplete('lateral correction requires clearance from support')
                    if abs(error[1]) > .006:
                        angle = -error[1]/max(sample['support_top_m'][0], .1)
                        gain = self._yaw_gain(angle, .68)
                        self.pulse(omega=math.copysign(.14, angle), duration=min(400,max(80,abs(angle)/(.14*gain)*1000)))
                    else:
                        gain = self._forward_gain(.65)
                        self.pulse(velocity=math.copysign(.03,-error[0]), duration=min(500,max(80,abs(error[0])/(.03*gain)*1000)))
                    tower_spot = self.predict(tower_spot)
                    self.end_rc()
                    motion_after = time.time()
                    continue
                if abs(gap) <= planner.CONTACT_GAP_M:
                    contacts += 1
                    if contacts >= 2:
                        break
                    self.sleep(.8)
                    continue
                contacts = 0
                fit = planner.observed_joints(sample['jaw']['hand_pose_base'])
                self.log('placement_pose_fit', fit=fit)
                step = planner.descent_step(sample, fit['joints'] if fit else None)
                if step is None:
                    return incomplete('no legal real step fits remaining visual gap')
                self.log('placement_descent', **step, gap_m=gap)
                try:
                    self.command(step['method'], step['params'], timeout_ms=3000)
                except (UncertainMotion, requests.RequestException):
                    self.log('data_gap', kind='placement_descent_unknown_observe_no_replay')
                previous_descent = dict(gap_m=gap, descent_m=step['descent_m'])
                self.sleep(2.)
                motion_after = time.time()
            else:
                return incomplete('held-column alignment/descent did not converge')
            self.end_rc()
            before_separation = sample['jaw_separation_m']
            try:
                self.open_jaws()
            except (UncertainMotion, requests.RequestException):
                self.log('data_gap', kind='release_unknown_verify_without_replay')
            # No verification capture is requested concurrently with release.
            self.sleep(3.)
            self.command('app_arm_direction_move', {'direction':1}, timeout_ms=3000)
            self.sleep(2.)
            barrier = time.time()
            stable = []
            for _ in range(3):
                sample = self.placement_sample(color, support_color, base_color, tower_spot, released=True, after=barrier)
                if (abs(sample['gap_m']) > .005 or np.linalg.norm(sample['lateral_error_m']) > .012
                        or sample['jaw_separation_m'] < max(.045,before_separation+.015)):
                    return incomplete('detach or stack-supported geometry not observed')
                center = np.asarray(sample['held_center_m'])
                if stable and np.linalg.norm(center-stable[0]) > .005:
                    return incomplete('placed object moved during stability observation')
                stable.append(center)
                self.sleep(2.)
                barrier = time.time()
            self.gripped = False
            result = dict(verified=True, detached=True, stable_observations=len(stable), tier=tier,
                          evidence=sample)
            self.log('placement_verified', **result)
            return result
        except (LookupError, ValueError, UncertainMotion, requests.RequestException) as e:
            return incomplete(str(e))
        finally:
            self.end_rc()

    def open_jaws(self):
        """Confirm manual grab mode and dispatch in-place jaw release."""
        self.command('app_pick_up_enter_exit', {'flag': 1})
        for _ in range(12):
            self.safety()
            status = self._status()
            mode = status.get('grabMode', status.get('grab_mode'))
            if mode is None and 'common_status' in status:
                mode = (int(status['common_status']) >> 10) & 3
            if mode == 1:
                reply = self.command('app_put_down', {'flag': 0})
                self.log('in_place_open_dispatched', mode=mode)
                return dict(release_dispatched=bool(reply), detached=False)
            self.sleep(.5)
        raise LookupError('manual grabMode 1 not observed; no put_down dispatched')

    def put_down(self):
        self.log('release_start', odom=self.odom())
        self.gripped = False
        try:
            self.command('app_put_down', {'flag': 0})
        except (UncertainMotion, requests.RequestException):
            self.log('data_gap', kind='put_down_outcome_observe')
        for _ in range(40):
            self.safety()
            status = self._status()
            common = int(status.get('common_status', 0))
            if ((common >> 6) & 15) in (0, 3):
                break
            self.sleep(.5)
        else:
            raise LookupError('put_down readiness not observed')
        self.sleep(1.)
        self.safety()
        self.snap('5-put-down')
        self.log('release_settled', settle_seconds=1, odom=self.odom())

    def stow(self):
        self.command('app_arm_in_compartment', {})
        self.deployed = False
        for _ in range(20):
            try:
                estimate = self._arm()
                if estimate.get('state') == 'IN_ROBOT':
                    self.log('stow_estimate', state='IN_ROBOT')
                    for _ in range(40):
                        reason = self.hazard()
                        if reason:
                            raise SafetyStop(reason)
                        status = self._status()
                        if status.get('error_code', 0):
                            raise SafetyStop('robot error: ' + str(status['error_code']))
                        if 'common_status' not in status or ((int(status['common_status']) >> 6) & 15) == 0:
                            break
                        self.sleep(.5)
                    self.sleep(1.)
                    self.snap('6-stowed')
                    self.log('stow_observation', odom=self.odom())
                    return dict(stowed=True, state='IN_ROBOT')
            except (requests.RequestException, LookupError):
                self.log('data_gap', kind='stow_estimate')
            self.sleep(.3)
        raise LookupError('stow dispatched but estimate not reconciled; no blind replay')

    def release_and_stow(self, put_down=True):
        self.end_rc()
        if put_down:
            self.put_down()
        return self.stow()

    def _emergency_stop(self, reason, **extra):
        self.stopped = True
        self.log('stop', reason=str(reason), **extra)
        commands = getattr(self.client, 'commands', None)
        if commands is not None:
            commands.stop()
        else:
            self.command('app_rc_stop')
            self.command('app_rc_end')
            if self.deployed:
                self.command('app_stop_grasp')
        self.rc = False

    def pick(self, color, dry_run=False):
        self.log('start', color=color, dry_run=dry_run)
        if dry_run:
            self._status()
            estimate = self._arm()
            blobs = self.observe('front', None) or self.observe('front', color) or []
            candidates = [b for b in blobs if b['color'] == color]
            target = max(candidates, key=lambda b: b['area_px']) if candidates else None
            try:
                descent = planner.grasp_goal(target['position'], (150, 45))['steps'] if target else None
            except ValueError:
                descent = None
            if descent is None:
                descent = planner.grasp_goal(planner.grasp_point(), (150, 45))['steps']
            result = dict(
                dry_run=True,
                objects=blobs,
                target=target,
                approach=_safe_approach(target['position']) if target else None,
                hover=dict(state=[150, 45]),
                descent=descent,
                log_dir=str(self.directory),
                arm_state=estimate.get('state'),
            )
            self.log('dry_run', **result)
            return result
        try:
            self.safety()
            target = self.align_front(color)
            floor_spot = list(target['position'])
            size = OBJECT_SIZE_M
            self.pre_deploy_target = list(target['position'])
            self.deploy(settle=True)
            self.client.select('front')
            self.log('camera_select', camera='front')
            target = self.post_deploy_align(color)
            floor_spot = list(target['position'])
            self.open_jaws()
            descent = dict(planner.grasp_goal(floor_spot, (150, 45), size))
            self.log('model_descent', **descent)
            descent_state = [int(x) for x in list(descent['state'])]
            self.move_arm((descent_state[0], descent_state[1], 0), (150, 45, 0))
            current = (descent_state[0], descent_state[1])
            _, current, floor_spot = self.front_servo(color, current, floor_spot)
            if self.pregrip_jaw is None:
                self.pregrip_jaw = planner.measured_point(current, planner._CALIBRATION['effective_jaw_from_elbow_m'])
            lift_res = self.grasp_and_lift(lift_steps=2, verify_color=color, floor_spot=floor_spot)
            verified = lift_res['verified']
            self.release_and_stow(put_down=True)
            self.log('complete', verified=verified, odom=self.odom())
            return dict(verified=verified, log_dir=str(self.directory))
        except BaseException as e:
            self._emergency_stop(e)
            raise
        finally:
            if not self.stopped:
                self.end_rc()

    def stack(self, base_color, middle_color, top_color, dry_run=False):
        colors = [base_color, middle_color, top_color]
        if len(set(colors)) != 3:
            raise ValueError('stack requires 3 distinct colors')
        self.log('stack_start', base_color=base_color, middle_color=middle_color, top_color=top_color, dry_run=dry_run)
        if dry_run:
            self._status()
            estimate = self._arm()
            all_blobs = list(self.observe('front', None) or [])
            if not all_blobs:
                seen_ids = set()
                for c in colors:
                    for b in (self.observe('front', c) or []):
                        key = (b.get('color'), tuple(b.get('bbox', ())))
                        if key not in seen_ids:
                            seen_ids.add(key)
                            all_blobs.append(b)
            targets = {}
            for c in colors:
                candidates = [b for b in all_blobs if b.get('color') == c]
                targets[c] = max(candidates, key=lambda b: b.get('area_px', 0)) if candidates else None
            tiers = []
            for idx, c in enumerate(colors, start=1):
                t = targets[c]
                tier_plan = dict(planner.stack_tier_plan(idx, OBJECT_SIZE_M))
                op_xy = list(tier_plan['operational_point_m'])[:2]  # type: ignore[arg-type]
                tiers.append({
                    **tier_plan,
                    'color': c,
                    'target': t,
                    'approach': _safe_approach(t['position'], op_xy + [0.]) if t and 'position' in t else None,
                })
            all_found = all(targets[c] is not None for c in colors)
            result = dict(
                dry_run=True,
                base_color=base_color,
                middle_color=middle_color,
                top_color=top_color,
                objects=all_blobs,
                targets=targets,
                tiers=tiers,
                all_targets_visible=all_found,
                log_dir=str(self.directory),
                arm_state=estimate.get('state'),
            )
            self.log('stack_dry_run', **result)
            return result
        tiers_completed = 0
        def stack_incomplete(**extra):
            result = dict(tiers_completed=tiers_completed, stack_verified=False,
                          reason='supported-placement-unresolved', holding=self.gripped,
                          log_dir=str(self.directory), **extra)
            self.log('stack_incomplete', **result)
            return result
        try:
            self.safety()
            base_target = self.target('front', base_color)
            tower_spot = list(base_target['position'])
            tiers_completed = 1
            self.log('tier_1_base_localized', color=base_color, position=tower_spot)
            # Deploy BEFORE driving up to the objects so the vendor LiDAR/front clearance check passes
            if not self.deployed:
                self.pre_deploy_target = list(tower_spot)
                self.deploy(settle=True)
                self.log('camera_stream_retained', camera='front')
            for tier_idx, move_color in ((2, middle_color), (3, top_color)):
                self.evidence_prefix = f'tier-{tier_idx}-attempt-1-'
                self.pregrip_jaw = None
                self.pregrip_object_in_hand = None
                self.last_floor_gone = False
                tier_plan = dict(planner.stack_tier_plan(tier_idx, OBJECT_SIZE_M))
                self.log('tier_start', tier=tier_idx, color=move_color, plan=tier_plan)
                target = self.align_front(move_color)
                floor_spot = list(target['position'])
                start_joints = self.current_arm_joints()
                self.open_jaws()
                descent = dict(planner.grasp_goal(floor_spot, (190, 65), OBJECT_SIZE_M))
                self.log('model_descent', tier=tier_idx, **descent)
                descent_state = [int(x) for x in list(descent['state'])]
                self.move_arm((190, 65, 0), start_joints)
                self.move_arm((descent_state[0], descent_state[1], 0), (190, 65, 0))
                current = (descent_state[0], descent_state[1])
                _, current, floor_spot = self.front_servo(move_color, current, floor_spot)
                lift_steps = int(tier_plan['lift_steps'])
                lift_res = self.grasp_and_lift(lift_steps=lift_steps, verify_color=move_color, floor_spot=floor_spot)
                if not lift_res['verified']:
                    raise LookupError(f'tier {tier_idx} ({move_color}) lift verification unresolved')
                tower_spot = list(self.target('front', base_color)['position'])
                support_color = base_color if tier_idx == 2 else middle_color
                placed = self.supported_place(move_color, support_color, base_color, tower_spot, tier_idx)
                if not placed['verified']:
                    return stack_incomplete(placement=placed)
                tiers_completed = tier_idx
            self.stow()
            self.log('stack_complete', tiers_completed=tiers_completed, stack_verified=tiers_completed == 3, odom=self.odom())
            return dict(tiers_completed=tiers_completed, stack_verified=tiers_completed == 3, log_dir=str(self.directory))
        except (LookupError, ValueError, UncertainMotion, requests.RequestException) as e:
            return stack_incomplete(detail=str(e))
        except BaseException as e:
            self._emergency_stop(e, tiers_completed=tiers_completed)
            raise
        finally:
            if not self.stopped:
                self.end_rc()
