"""Arm pose dead-reckoning estimator, read-only dashboard topics, and ROS diagnostics."""
from copy import deepcopy
import json
import math
import os
from pathlib import Path as FilePath
import tempfile
import time
from typing import Any
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Bool, Float32, Int32, String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from z70_description.arm_model import (
    POSES, LIMITS, JOINT_STEP_DEG, HAND_STEP_M, vendor_forward, vendor_inverse, vendor_check,
)
from z70_interfaces.msg import CalibrationState, OdometryStatus, CommandEvent
from .commands import LATCHED

ARM_METHODS = {'app_arm_out_compartment', 'app_arm_in_compartment', 'app_open_gripper',
    'app_grip', 'app_arm_move', 'app_arm_direction_move', 'app_start_program',
    'app_pick_up', 'app_put_down', 'app_stop_grasp', 'app_stop', 'app_arm_recover'}
STATES = {'IN_ROBOT', 'WORKING_POS', 'TRANSFER_POS', 'IN_PROCESS_POS', 'UNKNOWN'}


class ArmEstimator:
    def __init__(self, path=None, now=time.time):
        self.path = FilePath(path) if path else None
        self.now = now
        self.seen = set()
        self.grab_mode = None
        self.status_at = -math.inf
        self.data: dict[str, Any] = dict(state='UNKNOWN', joints=dict(POSES['IN_ROBOT']), wrist=None,
            sigmaDeg=dict(m3=0., m4=0., m5=0.), stepsSinceAnchor=0,
            anchoredAt=None, updatedAt=int(now()*1000), restored=False,
            lastFault=None, unknownReason='NO_SAVED_SESSION', history=[],
            source='dead-reckoning', model='vendor-app')
        if self.path:
            try:
                saved = json.loads(self.path.read_text())
                d = saved['estimate']
                if saved['version'] != 1 or d['state'] not in STATES: raise ValueError()
                if not all(type(d['joints'][j]) in (float, int) and math.isfinite(d['joints'][j]) for j in ('m3','m4','m5','m6')): raise ValueError()
                if not all(type(d['sigmaDeg'][j]) in (float, int) and math.isfinite(d['sigmaDeg'][j]) and d['sigmaDeg'][j] >= 0 for j in ('m3','m4','m5')): raise ValueError()
                if type(d['updatedAt']) not in (float, int) or not math.isfinite(d['updatedAt']): raise ValueError()
                if type(d['stepsSinceAnchor']) is not int or d['stepsSinceAnchor'] < 0: raise ValueError()
                if d.get('anchoredAt') is not None and (type(d['anchoredAt']) not in (int,float) or not math.isfinite(d['anchoredAt'])): raise ValueError()
                if not isinstance(d.get('history'),list): raise ValueError()
                self.data.update(d)
                self.data['restored'] = True
                self.data['unknownReason'] = 'RESTORED_UNRECONCILED'
                self._wrist()
            except (OSError, ValueError, TypeError, KeyError):
                pass

    def snapshot(self):
        return deepcopy(self.data)

    def _wrist(self):
        d = self.data
        if d['state'] in ('UNKNOWN', 'IN_ROBOT') or d['restored']:
            d['wrist'] = None
        else:
            x, y = vendor_forward(d['joints']['m3'], d['joints']['m4'])[1]
            d['wrist'] = dict(x=x, y=y)

    def _commit(self, method, effect):
        d = self.data
        d['updatedAt'] = max(int(self.now()*1000), d['updatedAt']+1)
        d['history'] = (d.get('history', [])+[dict(at=d['updatedAt'], method=method, effect=effect)])[-30:]
        self._wrist()
        if self.path:
            name = None
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                with tempfile.NamedTemporaryFile(mode='w', dir=self.path.parent, prefix='.arm-', delete=False) as f:
                    name = f.name
                    json.dump(dict(version=1, estimate=d), f, allow_nan=False)
                    f.flush(); os.fsync(f.fileno())
                os.replace(name, self.path)
                directory=os.open(self.path.parent,os.O_RDONLY)
                try: os.fsync(directory)
                finally: os.close(directory)
                d.pop('persistenceError',None)
            except OSError:
                d['persistenceError']=True
            finally:
                if name and os.path.exists(name): os.unlink(name)
        return True

    def invalidate(self, reason='UNOBSERVED_MOTION'):
        self.data['state'] = 'UNKNOWN'
        self.data['unknownReason'] = reason if reason in {'STOP_UNVERIFIED','SESSION_CHANGED','COMMAND_STREAM_GAP','UNOBSERVED_MOTION','UNKNOWN_DISPATCH','MODE_EXIT','OPERATOR_ANCHOR','RESTORED_UNRECONCILED','GRIP_POSE_UNVERIFIED','PROGRAM_POSE_UNVERIFIED','INVALID_COMMAND'} else 'UNOBSERVED_MOTION'
        return self._commit('invalidate', self.data['unknownReason'])

    def set_anchor(self, anchor, reason='OPERATOR_ANCHOR'):
        if anchor not in ('unknown', 'stowed', 'deployed'): raise ValueError('INVALID_INPUT')
        if anchor == 'unknown': self.invalidate('OPERATOR_ANCHOR')
        else: self._anchor('IN_ROBOT' if anchor == 'stowed' else 'WORKING_POS', 'anchor')
        return self.snapshot()

    def _anchor(self, state, method):
        self.data.update(state=state, joints=dict(POSES[state]), sigmaDeg=dict(m3=0.,m4=0.,m5=0.),
            stepsSinceAnchor=0, anchoredAt=int(self.now()*1000), restored=False,
            unknownReason=None, lastFault=None)
        return self._commit(method, 'COMMAND_DERIVED_ANCHOR')

    def _movable(self):
        return self.data['state'] not in ('UNKNOWN','IN_ROBOT') and not self.data['restored']

    def _within_reach(self, joints):
        current = self.data['joints']
        new_x = vendor_forward(joints['m3'], joints['m4'])[1][0]
        return not (new_x > .367 and new_x > vendor_forward(current['m3'], current['m4'])[1][0])

    def _apply(self, joints, n, method):
        if not self._within_reach(joints): return False
        self.data.update(joints=joints, state='IN_PROCESS_POS', stepsSinceAnchor=self.data['stepsSinceAnchor']+n)
        fault = vendor_check(joints['m3'],joints['m4'],joints['m5'])
        self.data['lastFault'] = None if fault == 'SUCCESS' else fault
        return self._commit(method, 'NOMINAL_STEP')

    def _joints(self, values, n, method):
        joints = dict(self.data['joints']); moved = []
        for key, sel in zip(('m3','m4','m5'), values):
            direction = (1 if sel == 1 else -1 if sel == 2 else 0) if type(sel) in (int,float) else 0
            if direction:
                lo, hi = LIMITS[key]
                joints[key] = max(lo, min(hi, joints[key]+direction*JOINT_STEP_DEG*n)); moved.append(key)
        if not moved: return False
        if not self._movable():
            if self.data['state'] != 'IN_ROBOT' and float(self.data['joints'].get('m3', 0)) >= LIMITS['m3'][0] and self._within_reach(joints):
                self.data['joints'] = joints
            return False
        if not self._apply(joints,n,method): return False
        for key in moved: self.data['sigmaDeg'][key] = math.hypot(self.data['sigmaDeg'][key],2.5*math.sqrt(n))
        return self._commit(method, 'HEURISTIC_UNCERTAINTY')

    def _hand(self, direction, n, method):
        if not self._movable() and (self.data['state'] == 'IN_ROBOT' or float(self.data['joints'].get('m3', 0)) < LIMITS['m3'][0]):
            return False
        delta = {1:(0,1),2:(0,-1),3:(1,0),4:(-1,0)}.get(direction) if type(direction) in (int,float) else None
        if delta is None:
            return self.invalidate('INVALID_COMMAND') if self._movable() else False
        joints = dict(self.data['joints'])
        for _ in range(n):
            x,y = vendor_forward(joints['m3'],joints['m4'])[1]
            solution = vendor_inverse(x+delta[0]*HAND_STEP_M,y+delta[1]*HAND_STEP_M,(joints['m3'],joints['m4']))
            if solution is None:
                if not self._movable(): return False
                self.data['lastFault']='FAULT_IK'; return self._commit(method,'UNREACHABLE_NOMINAL')
                joints['m3'],joints['m4'] = solution
            joints['m3'],joints['m4'] = solution
        if not self._movable():
            if self._within_reach(joints):
                self.data['joints'] = joints
            return False
        if not self._apply(joints,n,method): return False
        for key in ('m3','m4'): self.data['sigmaDeg'][key] = math.hypot(self.data['sigmaDeg'][key],2.5*math.sqrt(n))
        return self._commit(method, 'HEURISTIC_UNCERTAINTY')

    def _set_jaws(self, m6, method, allow_unknown=True):
        if not self._movable():
            if allow_unknown and self.data['state'] != 'IN_ROBOT':
                self.data['joints']['m6'] = m6
            return False
        self.data['joints']['m6'] = m6
        return self._commit(method, 'GRIPPER_COMMAND_ONLY')

    def observe(self, method, params, outcome, command_id=None):
        if command_id:
            if command_id in self.seen: return False
            self.seen.add(command_id)
            if len(self.seen)>4096: self.seen = {command_id}
        if outcome in ('not_sent','vendor_error'): return False
        p = params[0] if isinstance(params,list) and len(params)==1 else params
        if method not in ARM_METHODS:
            if method == 'app_pick_up_enter_exit':
                if outcome == 'received' and isinstance(p,dict) and type(p.get('flag')) is int and p['flag'] in (0,1,2):
                    self.grab_mode = p['flag']
                return False
            if method.startswith(('get_','app_get_','app_rc_')) or method == 'app_change_camera': return False
            if method in ('app_goto_target','stop_goto_target','app_charge','app_pause','app_stop','app_stop_grasp') and self.data['state']=='IN_ROBOT': return False
            return self.invalidate('UNOBSERVED_MOTION')
        if method == 'app_grip':
            if self.grab_mode == 1 and isinstance(p,dict) and p.get('flag') == 0:
                if outcome == 'unknown': return False
                return self._set_jaws(0, method)
            return self.invalidate('GRIP_POSE_UNVERIFIED')
        if outcome == 'unknown':
            if method == 'app_open_gripper': return False
            return self.invalidate('UNKNOWN_DISPATCH')
        if method == 'app_arm_in_compartment': return self._anchor('IN_ROBOT',method)
        if method == 'app_arm_out_compartment':
            if self.data['state']=='IN_ROBOT' and not self.data['restored']:
                return self._anchor('WORKING_POS',method)
            self.data['joints'] = dict(POSES['WORKING_POS'])
            return False
        if method == 'app_open_gripper':
            return self._set_jaws(90, method, allow_unknown=False)
        if method == 'app_put_down' and self.grab_mode == 1 and isinstance(p,dict) and p.get('flag') == 0:
            return self._set_jaws(90, method)
        if method == 'app_arm_move':
            if not isinstance(p,dict) or not isinstance(p.get('m'),list) or len(p['m']) not in (3,5): return self.invalidate('INVALID_COMMAND')
            return self._joints(p['m'][:3],1,method)
        if method == 'app_arm_direction_move':
            return self._hand(p.get('direction'),1,method) if isinstance(p,dict) else self.invalidate('INVALID_COMMAND')
        if method == 'app_start_program':
            if not isinstance(p,dict) or not isinstance(p.get('cmd_ids'),list): return self.invalidate('INVALID_COMMAND')
            changed = False
            for c in p['cmd_ids']:
                if not isinstance(c,dict): continue
                n = c.get('step',1); n = n if type(n) is int and 0 < n <= 10000 else 1
                cmd = c.get('cmd_id')
                if cmd == 'arm_in': changed = self._anchor('IN_ROBOT',method) or changed
                elif cmd == 'arm_out' and self.data['state']=='IN_ROBOT' and not self.data['restored']: changed = self._anchor('WORKING_POS',method) or changed
                elif cmd in ('arm_grasp','arm_putdown'): changed = self.invalidate('PROGRAM_POSE_UNVERIFIED') or changed
                elif cmd == 'arm_joints':
                    v = c.get('value', [])
                    if isinstance(v, list):
                        if any(type(x) in (int, float) and x in (1, 2) for x in v[:3]):
                            changed = self._joints(v[:3], n, method) or changed
                        if len(v) >= 4 and type(v[3]) in (int, float) and v[3] in (1, 2) and self._movable():
                            self.data['joints']['m6'] = 90 if v[3] == 1 else 0; changed = self._commit(method, 'GRIPPER_COMMAND_ONLY') or changed
                elif cmd == 'arm_move': changed = self._hand(c.get('direction'),n,method) or changed
                elif cmd not in ('delay','sound','chassis_move','chassis_rotate'): changed=self.invalidate('PROGRAM_POSE_UNVERIFIED') or changed
            return changed
        if method in ('app_stop', 'app_stop_grasp') and self.data['state']=='IN_ROBOT': return False
        return self.invalidate('UNOBSERVED_MOTION')

    def observe_status(self, status, observed_at_ms):
        s = status[0] if isinstance(status,list) and len(status)==1 else status
        if not isinstance(s,dict) or not isinstance(observed_at_ms,(float,int)) or not math.isfinite(observed_at_ms) or observed_at_ms < max(self.data['updatedAt'],self.status_at): return False
        self.status_at=observed_at_ms
        previous=self.grab_mode; bits=s.get('common_status')
        mode=(bits>>10)&3 if type(bits) is int and 0<=bits<=0xffffffff else None
        if mode is not None: self.grab_mode=mode
        if self.data['state'] in ('IN_ROBOT','UNKNOWN'): return False
        state=s.get('state')
        if type(state) is int and state>0 and state not in (3,4,39,42): return self.invalidate('MODE_EXIT')
        if state not in (3,4,42) and mode==0 and previous not in (None,0): return self.invalidate('MODE_EXIT')
        return False

STATE_NAMES = {
    0: 'UNKNOWN', 1: 'STARTING', 2: 'CHARGER_DISCONNECTED', 3: 'IDLE',
    4: 'REMOTE_CONTROL_ACTIVE', 5: 'CLEANING', 6: 'RETURNING_HOME',
    7: 'MANUAL_MODE', 8: 'CHARGING', 9: 'CHARGING_PROBLEM', 10: 'PAUSED',
    11: 'SPOT_CLEANING', 12: 'ERROR', 13: 'SHUTTING_DOWN', 14: 'UPDATING',
    15: 'DOCKING', 16: 'GOING_TO_TARGET', 17: 'ZONED_CLEANING',
    18: 'SEGMENT_CLEANING', 22: 'EMPTYING_THE_BIN', 23: 'WASHING_THE_MOP',
    25: 'WASHING_THE_MOP', 26: 'GOING_TO_WASH_THE_MOP', 28: 'IN_CALL',
    29: 'MAPPING', 30: 'EGG_ATTACK', 32: 'PATROL', 33: 'ATTACHING_THE_MOP',
    34: 'DETACHING_THE_MOP', 39: 'REMOTE_PICKUP', 41: 'ARM_RESETTING',
    42: 'PROGRAM_MODE', 100: 'CHARGING_COMPLETE', 101: 'DEVICE_OFFLINE', 103: 'LOCKED',
}
ERROR_NAMES = dict(enumerate([
    'NONE', 'LIDAR_BLOCKED', 'BUMPER_STUCK', 'WHEELS_SUSPENDED', 'CLIFF_SENSOR_ERROR',
    'MAIN_BRUSH_JAMMED', 'SIDE_BRUSH_JAMMED', 'WHEELS_JAMMED', 'ROBOT_TRAPPED',
    'NO_DUSTBIN', 'STRAINER_ERROR', 'COMPASS_ERROR', 'LOW_BATTERY', 'CHARGING_ERROR',
    'BATTERY_ERROR', 'WALL_SENSOR_DIRTY', 'ROBOT_TILTED', 'SIDE_BRUSH_ERROR', 'FAN_ERROR',
    'DOCK_POWER_ERROR', 'OPTICAL_FLOW_SENSOR_DIRTY', 'VERTICAL_BUMPER_PRESSED',
    'DOCK_LOCATOR_ERROR', 'RETURN_TO_DOCK_FAIL', 'NOGO_ZONE_DETECTED', 'VISUAL_SENSOR_ERROR',
    'LIGHT_TOUCH_ERROR', 'VIBRARISE_JAMMED', 'ROBOT_ON_CARPET', 'FILTER_BLOCKED',
    'INVISIBLE_WALL_DETECTED', 'CANNOT_CROSS_CARPET', 'INTERNAL_ERROR',
]))
ERROR_NAMES.update({34: 'COLLECT_DUST_ERROR', 35: 'AUTO_EMPTY_DOCK_VOLTAGE_ERROR',
                    38: 'CHECK_CLEAN_WATER_TANK', 39: 'CHECK_DIRTY_WATER_TANK',
                    41: 'CLEAN_WATER_EMPTY', 42: 'DOCK_WATER_FILTER_ERROR',
                    44: 'DOCK_WATER_FILTER_BLOCKED', 48: 'UP_WATER_EXCEPTION',
                    49: 'DRAIN_WATER_EXCEPTION', 51: 'TEMPERATURE_PROTECTION',
                    53: 'CLEANING_TANK_FULL_OR_BLOCKED', 56: 'AUDIO_ERROR'})


def number(value):
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) else math.nan


def decode_command(envelope, now):
    commands = [c for c in envelope.get('commands', []) if isinstance(c, dict)]
    latest = max(commands, key=lambda c: number(c.get('requestedAt')) if math.isfinite(number(c.get('requestedAt'))) else -1, default={})
    stamp = number(latest.get('sentAt'))
    if not math.isfinite(stamp):
        stamp = number(latest.get('requestedAt'))
    return dict(age_s=max(0., now - stamp / 1000.) if math.isfinite(stamp) else math.nan,
                method=latest.get('method', 'none'),
                outcome=f"{latest.get('dispatch', 'none')}/{latest.get('response', 'none')}")


def decode_arm(estimate):
    state = estimate.get('state', 'UNKNOWN')
    joints, sigma = estimate.get('joints') or {}, estimate.get('sigmaDeg') or {}
    values = dict(state=state, unknown_reason=estimate.get('unknownReason') or '',
                  steps_since_anchor=number(estimate.get('stepsSinceAnchor')))
    for joint in ('m3', 'm4', 'm5', 'm6'):
        values[joint + '_deg'] = number(joints.get(joint)) if state not in ('UNKNOWN', 'IN_ROBOT') else math.nan
        values['sigma_' + joint + '_deg'] = number(sigma.get(joint))
    return values


def decode_calibration(calibration, odom):
    camera = calibration.get('camera_x') or {}
    values = dict(camera_x_m=number(camera.get('value_m')), camera_x_sigma_m=number(camera.get('sigma_m')),
                  camera_x_ready=bool(camera.get('ready')), samples=number(calibration.get('n')),
                  active_camera_x_m=number(odom.get('camera_x_m')),
                  vo_accepted=number(odom.get('ok')), vo_rejected=number(odom.get('rejected')),
                  vo_state=odom.get('state', 'unknown'))
    for key in ('yaw_positive', 'yaw_negative', 'forward'):
        values[key + '_gain'] = number((calibration.get(key) or {}).get('gain'))
    return values


def decode_status(envelope, now):
    entry = envelope.get('telemetry', {}).get('status', {})
    status = entry.get('value') or {}
    if isinstance(status, list):
        status = status[0] if status and isinstance(status[0], dict) else {}
    if not isinstance(status, dict):
        status = {}
    state = status.get('state')
    error = status.get('error_code')
    age = max(0., now - entry['observedAt'] / 1000.) if math.isfinite(number(entry.get('observedAt'))) else math.nan
    valid = bool(status) and entry.get('error') is None and math.isfinite(age) and age < 10
    code = state if isinstance(state, int) and not isinstance(state, bool) else -1
    error_code = error if isinstance(error, int) and not isinstance(error, bool) else -1
    return dict(battery=number(status.get('battery')) if valid else math.nan, state_code=code,
                state_name=STATE_NAMES.get(code, f'UNKNOWN({code})'), error_code=error_code,
                error_name=ERROR_NAMES.get(error_code, f'UNKNOWN({error_code})'),
                cleaning=bool(status.get('in_cleaning')) if valid and 'in_cleaning' in status else None,
                returning=bool(status.get('in_returning')) if valid and 'in_returning' in status else None,
                charging=code == 8 if valid and code >= 0 else None,
                connection_state=envelope.get('connection', {}).get('state', 'unknown'),
                connected=envelope.get('connection', {}).get('state') == 'connected',
                status_valid=valid, status_age_s=age)


STATUS_TYPES = {'battery': Float32, 'state_code': Int32, 'state_name': String,
                'error_code': Int32, 'error_name': String, 'cleaning': Bool,
                'returning': Bool, 'charging': Bool, 'connected': Bool,
                'connection_state': String, 'status_valid': Bool, 'status_age_s': Float32}
ARM_TYPES = {'state': String, 'unknown_reason': String, 'steps_since_anchor': Float32,
             **{j + '_deg': Float32 for j in ('m3', 'm4', 'm5', 'm6')},
             **{'sigma_' + j + '_deg': Float32 for j in ('m3', 'm4', 'm5', 'm6')}}
CAL_TYPES = {key: Float32 for key in ('camera_x_m', 'camera_x_sigma_m', 'samples',
             'active_camera_x_m', 'vo_accepted', 'vo_rejected', 'yaw_positive_gain',
             'yaw_negative_gain', 'forward_gain')}
CAL_TYPES.update(camera_x_ready=Bool, vo_state=String)


class Dashboard:
    def __init__(self, node):
        self.node = node
        self.state = {}; self.arm = {}; self.commands = {}; self.calibration = {}; self.odom = {}
        self.state_received = self.arm_received = self.commands_received = self.cal_received = self.odom_received = 0.
        self.publishers = {}
        self.types = {}
        for prefix, types in [('/z70/', STATUS_TYPES), ('/z70/arm/', ARM_TYPES),
                              ('/z70/calibration/', CAL_TYPES),
                              ('/z70/last_command/', {'age_s': Float32, 'outcome': String, 'method': String})]:
            for key, kind in types.items():
                topic = prefix + key
                self.publishers[topic] = node.create_publisher(kind, topic, 10)
                self.types[topic] = kind
        self.diagnostics = node.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.path = Path()
        self.pub_path = node.create_publisher(Path, '/z70/odom_path', 10)
        node.create_subscription(Odometry, '/odom', self.on_odometry, 10)
        node.create_subscription(CalibrationState, '/z70/chassis_calibration', self.on_calibration, LATCHED)
        node.create_subscription(OdometryStatus, '/z70/odom_status', self.on_odom, 10)
        node.create_subscription(CommandEvent, '/z70/commands', self.on_command, 256)
        node.create_timer(1., self.publish)

    def on_odometry(self, msg):
        if self.path.header.frame_id != msg.header.frame_id:
            self.path.poses = []
        self.path.header = msg.header
        pose = PoseStamped(header=msg.header, pose=msg.pose.pose)
        if self.path.poses:
            prev = self.path.poses[-1].pose.position
            if math.hypot(pose.pose.position.x - prev.x, pose.pose.position.y - prev.y) < 0.002:
                self.path.poses[-1] = pose
                self.pub_path.publish(self.path)
                return
        self.path.poses = (self.path.poses + [pose])[-2000:]
        self.pub_path.publish(self.path)

    def on_calibration(self, msg):
        if not msg.evidence.valid or msg.evidence.stale:
            self.calibration = {}; self.cal_received = 0.; return
        self.calibration = {'n': msg.sample_count, 'camera_x': {
            'value_m': msg.camera_x_m if msg.camera_x_known else None,
            'sigma_m': msg.camera_x_sigma_m if msg.camera_x_sigma_known else None,
            'ready': msg.camera_x_ready}}
        for name in ('yaw_positive', 'yaw_negative', 'forward'):
            self.calibration[name] = {'gain': getattr(msg, name+'_gain') if getattr(msg, name+'_known') else None}
        self.cal_received = time.time()

    def on_odom(self, msg):
        if not msg.evidence.valid or msg.evidence.stale:
            self.odom_received=0.
            return
        self.odom['camera_x_m']=msg.active_camera_x_m if msg.active_camera_x_known and math.isfinite(msg.active_camera_x_m) else None
        if msg.validity.value==0:
            self.odom['state']='init'
            self.odom_received=time.time()
            return
        self.odom = {'ok': 0, 'rejected': 0, **self.odom, 'state': 'accepted' if msg.pair_accepted else 'rejected'}
        key = 'ok' if msg.pair_accepted else 'rejected'
        self.odom[key] = self.odom.get(key, 0)+1
        self.odom_received = time.time()

    def on_command(self, msg):
        stamp = msg.outcome.requested_at
        self.commands = {'commands': [{'method': msg.operation,
            'requestedAt': stamp.sec*1000+stamp.nanosec/1e6,
            'dispatch': 'enum:'+str(msg.outcome.dispatch), 'response': 'enum:'+str(msg.outcome.response)}]}
        self.commands_received = time.time()

    def values(self, now):
        return (decode_status(self.state, now), decode_arm(self.arm),
                decode_command(self.commands, now), decode_calibration(self.calibration, self.odom))

    def message(self, now):
        state, arm, command, calibration = self.values(now)
        fresh = bool(self.state_received) and now - self.state_received < 5
        robot_fresh = fresh and state['status_valid'] and math.isfinite(state['status_age_s']) and state['status_age_s'] < 10
        statuses = [
            ('Connection', 0 if fresh and state['connected'] else 3, state['connection_state'],
             {'http_age_s': now-self.state_received if self.state_received else 'unknown', 'connected': state['connected']}),
            ('Robot', 3 if not robot_fresh else 2 if state['error_code'] != 0 else 1 if state['battery'] < 20 else 0,
             state['state_name'] + ' / ' + state['error_name'], state),
            ('Arm estimate', 3 if not self.arm_received or now-self.arm_received >= 5 else 1 if arm['state'] == 'UNKNOWN' else 0,
             arm['state'] + (' — ' + arm['unknown_reason'] if arm['unknown_reason'] else ''), arm),
            ('Last RC command', 3 if not self.commands_received else 1 if command['outcome'] not in ('sent/received', 'none/none', 'enum:1/enum:1', 'enum:0/enum:0') else 0,
             command['method'] + ' / ' + command['outcome'], command),
            ('Calibration', 3 if not self.cal_received else 0 if calibration['camera_x_ready'] else 1,
             'camera offset fit ready' if calibration['camera_x_ready'] else 'camera offset fit not ready', calibration),
            ('Visual odometry', 3 if not self.odom_received or now-self.odom_received >= 5 else 1 if calibration['vo_state'] == 'rejected' else 0,
             calibration['vo_state'], {k: v for k, v in calibration.items() if k.startswith('vo_') or k == 'active_camera_x_m'}),
        ]
        result = DiagnosticArray()
        result.header.stamp = self.node.get_clock().now().to_msg()
        result.header.frame_id = 'base_footprint'
        result.status = [DiagnosticStatus(name='Z70/' + name, hardware_id='Saros Z70', level=bytes([level]),
                         message=message, values=[KeyValue(key=k, value=str(v)) for k,v in values.items()])
                         for name, level, message, values in statuses]
        return result

    def publish(self, now=None):
        now = time.time() if now is None else now
        state, arm, command, calibration = self.values(now)
        for prefix, values, received, ttl in [('/z70/', state, self.state_received, 5),
                        ('/z70/arm/', arm, self.arm_received, 5),
                        ('/z70/last_command/', command, self.commands_received, math.inf),
                        ('/z70/calibration/', calibration, self.cal_received or self.odom_received, math.inf)]:
            stale = not received or now - received >= ttl
            for key, value in values.items():
                topic = prefix + key
                kind = self.types[topic]
                if kind == Float32 and stale: value = math.nan
                if prefix == '/z70/' and stale:
                    if key in ('connected', 'status_valid'): value = False
                    elif key == 'connection_state': value = 'unavailable'
                    elif key in ('cleaning','returning','charging'): continue
                if kind == Bool and value is None: continue
                self.publishers[topic].publish(kind(data=value))
        self.diagnostics.publish(self.message(now))
