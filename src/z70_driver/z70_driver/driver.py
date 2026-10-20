"""ROS 2 driver node, arm pose dead-reckoning estimator, read-only dashboard topics, and diagnostics."""
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import select
import subprocess
import tempfile
import threading
import time
from typing import Any
import uuid

import cv2
import numpy as np
import requests
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState
from std_msgs.msg import Bool, Float32, Int32, String
from unique_identifier_msgs.msg import UUID
from z70_interfaces.msg import (EvidenceStamp, RobotState, CameraState, FrameMetadata,
                                ArmEstimate, MapSnapshot, RoomLabel, CommandEvent, CommandOutcome,
                                CalibrationState, OdometryStatus)
from z70_interfaces.srv import SelectCamera, RefreshMap
from z70_description.arm_model import (
    HAND_STEP_M, JOINT_STEP_DEG, LIMITS, POSES, vendor_check, vendor_forward, vendor_inverse,
)
from z70_mapping.node import decode_map, room_names
from .commands import CommandSurfaces, LATCHED, host_ms, host_time, uid

STOW_VISUAL = dict(m3=90., m4=170., m5=0., m6=0.)
CAMERAS = {'front': 1, 'gripper': 2}
ARM_METHODS = {'app_arm_out_compartment', 'app_arm_in_compartment', 'app_open_gripper',
    'app_grip', 'app_arm_move', 'app_arm_direction_move', 'app_start_program',
    'app_pick_up', 'app_put_down', 'app_stop_grasp', 'app_stop', 'app_arm_recover'}
STATES = {'IN_ROBOT', 'WORKING_POS', 'TRANSFER_POS', 'IN_PROCESS_POS', 'UNKNOWN'}


class ArmEstimator:
    def __init__(self, path=None, now=time.time):
        self.path = Path(path) if path else None
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
                directory = os.open(self.path.parent, os.O_RDONLY)
                try: os.fsync(directory)
                finally: os.close(directory)
                d.pop('persistenceError', None)
            except OSError:
                d['persistenceError'] = True
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
                self.data['lastFault'] = 'FAULT_IK'; return self._commit(method, 'UNREACHABLE_NOMINAL')
            joints['m3'], joints['m4'] = solution
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
        self.status_at = observed_at_ms
        previous = self.grab_mode; bits = s.get('common_status')
        mode = (bits>>10)&3 if type(bits) is int and 0<=bits<=0xffffffff else None
        if mode is not None: self.grab_mode = mode
        if self.data['state'] in ('IN_ROBOT','UNKNOWN'): return False
        state = s.get('state')
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
        self.path = NavPath()
        self.pub_path = node.create_publisher(NavPath, '/z70/odom_path', 10)
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
            self.odom_received = 0.
            return
        self.odom['camera_x_m'] = msg.active_camera_x_m if msg.active_camera_x_known and math.isfinite(msg.active_camera_x_m) else None
        if msg.validity.value == 0:
            self.odom['state'] = 'init'
            self.odom_received = time.time()
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


def worker_env(**extra) -> dict[str, str]:
    env = dict(os.environ)
    for key in ('PYTHONPATH', 'PYTHONHOME', 'DYLD_LIBRARY_PATH'):
        env.pop(key, None)
    env.update(extra)
    return env


def read_pipe_exact(proc, count: int, stop_event, timeout_s: float = 120.) -> bytes:
    if proc is None or proc.stdout is None:
        raise EOFError('Inference worker unavailable')
    data = bytearray()
    deadline = time.monotonic() + timeout_s
    while len(data) < count and not stop_event.is_set():
        if time.monotonic() > deadline:
            raise TimeoutError('Inference deadline')
        ready, _, _ = select.select([proc.stdout], [], [], .2)
        if not ready:
            continue
        chunk = os.read(proc.stdout.fileno(), count - len(data))
        if not chunk:
            raise EOFError('Inference worker ended')
        data.extend(chunk)
    if len(data) != count:
        raise EOFError('Inference interrupted')
    return bytes(data)


def stop_worker_process(proc, worker, stop_event) -> None:
    stop_event.set()
    if proc is not None:
        if proc.stdin is not None:
            try:
                proc.stdin.close()
            except OSError:
                pass
        try:
            proc.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
    if worker is not None:
        worker.join(timeout=5.)
    if proc is not None and proc.stdout is not None:
        proc.stdout.close()


def frame_key(msg):
    return (msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id)


class MatchedFrames:
    key = staticmethod(frame_key)

    def __init__(self, node, camera, callback, compressed=False, on_session=None):
        self.callback = callback; self.on_session = on_session
        self.frame = None; self.metadata = None; self.used = None; self.session = None
        self.group = MutuallyExclusiveCallbackGroup()
        topic = f'/camera/{camera}'
        self.subscriptions = [
            node.create_subscription(CompressedImage if compressed else Image,
                topic + ('/image_raw/compressed' if compressed else '/image_raw'), self.image, 2, callback_group=self.group),
            node.create_subscription(FrameMetadata, topic + '/metadata', self.info, 2, callback_group=self.group)]

    def image(self, msg):
        self.frame = msg; self.match()

    def info(self, msg):
        self.metadata = msg; self.match()

    def match(self):
        frame, meta = self.frame, self.metadata
        if frame is None or meta is None or self.key(frame) != self.key(meta): return
        e = meta.evidence
        ident = (bytes(e.session_id.uuid), e.sequence)
        if ident == self.used: return
        self.used = ident
        age = time.time() - (e.received_at.sec + e.received_at.nanosec / 1e9)
        if not e.valid or e.stale or not meta.calibration_id or age < 0 or age > 5.: return
        if self.session is not None and self.session != ident[0] and self.on_session: self.on_session()
        self.session = ident[0]
        self.callback(frame)


def estimate_to_app(estimate):
    joints = estimate.get('joints') or {}
    if (estimate.get('state', 'UNKNOWN') == 'IN_ROBOT'
            or float(joints.get('m3', 0)) < LIMITS['m3'][0]
            or float(joints.get('m4', 0)) < LIMITS['m4'][0]):
        return dict(STOW_VISUAL)
    return {j: float(joints[j]) for j in STOW_VISUAL}


def app_to_urdf(m3, m4, m5, m6):
    f = max(0., min(math.radians(m6) / 2, math.pi / 4))
    return dict(m3_joint=math.radians(m3-270), m4_joint=math.radians(180-m4),
                m5_joint=-math.radians(m5), m6_left_joint=f, m6_right_joint=f)


class Driver(CommandSurfaces, Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('z70_driver', parameter_overrides=parameter_overrides or [])
        def param(name, default):
            return self.declare_parameter(name, default).value
        self.origin = param('origin', 'http://127.0.0.1:57010').rstrip('/')
        token_file = param('token_file', '')
        self.auth = {'Authorization': 'Bearer ' + Path(token_file).read_text().strip()} if token_file else {}
        self.camera_hz = float(param('camera_hz', 3.))
        self.state_hz = float(param('state_hz', 1.))
        self.map_interval = float(param('map_refresh_s', 60.))
        self.state_stale = float(param('state_stale_after_s', 5.))
        self.camera_stale = float(param('camera_stale_after_s', 5.))
        config = Path(get_package_share_directory('z70_description')) / 'config'
        self.calibrations = {}
        for camera, default in [('front', config/'z70_front_camera_info.yaml'),
                                ('gripper', config/'z70_gripper_camera_info.yaml')]:
            path = param('camera_info' if camera == 'front' else 'gripper_camera_info', str(default))
            if path and Path(path).is_file():
                content = Path(path).read_bytes()
                self.calibrations[camera] = (self.load_camera_info(yaml.safe_load(content)), hashlib.sha256(content).hexdigest())
        self.driver_session = uuid.uuid4()
        self.session = uid(self.driver_session)
        self.backend_session = None
        self.sequences = {}
        self.samples = {}
        self.arm_payload = None
        self.display_sample = None
        self.estimate_source = 1
        self.good_at = {}
        self.stop_event = threading.Event()
        self.acquisition_generation = 0
        self.camera_lock = threading.Lock()
        self.map_lock = threading.Lock()
        self.sample_lock = threading.RLock()
        self.workers = []
        self.worker_lock = threading.Lock()
        self.stream_response = None
        self.event_session = None
        self.event_sequence = None
        self.pub_state = self.create_publisher(RobotState, '/z70/state', LATCHED)
        self.pub_camera_state = self.create_publisher(CameraState, '/z70/camera/state', LATCHED)
        self.pub_arm = self.create_publisher(ArmEstimate, '/z70/arm_estimate', LATCHED)
        self.pub_map = self.create_publisher(MapSnapshot, '/z70/driver/map_snapshot', LATCHED)
        self.pub_js = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_display = self.create_publisher(JointState, '/z70/display/joint_states', 10)
        self.pub_events = self.create_publisher(CommandEvent, '/z70/commands', 256)
        self.pub_diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.dashboard = Dashboard(self)
        self.pub_img = {}; self.pub_raw = {}; self.pub_info = {}; self.pub_metadata = {}
        for camera in CAMERAS:
            prefix = '/camera/' + camera
            self.pub_img[camera] = self.create_publisher(CompressedImage, prefix+'/image_raw/compressed', LATCHED)
            self.pub_raw[camera] = self.create_publisher(Image, prefix+'/image_raw', 2)
            self.pub_info[camera] = self.create_publisher(CameraInfo, prefix+'/camera_info', LATCHED)
            self.pub_metadata[camera] = self.create_publisher(FrameMetadata, prefix+'/metadata', LATCHED)
        group = ReentrantCallbackGroup()
        self.create_service(SelectCamera, '/z70/camera/select', self.select, callback_group=group)
        self.create_service(RefreshMap, '/z70/map/refresh', self.refresh, callback_group=group)
        self.create_timer(.25, self.check_stale)
        estimate_default = str(Path(os.environ.get('XDG_DATA_HOME', str(Path.home()/'.local/share')))/'RRsZ70_ROS2/arm-estimate.json')
        self.estimator = ArmEstimator(path=param('arm_estimate_path', estimate_default))
        self.init_commands()
        self.publish_estimate(self.estimator.snapshot())
        self.start_worker('state', self.state_loop)
        self.start_worker('events', self.events_loop)
        if self.camera_hz > 0:
            self.start_worker('camera', self.camera_loop)
        if self.map_interval > 0:
            self.start_worker('map', self.map_loop)

    @staticmethod
    def load_camera_info(d):
        m = CameraInfo(width=d['image_width'], height=d['image_height'], distortion_model=d['distortion_model'])
        for field, key in [('k', 'camera_matrix'), ('d', 'distortion_coefficients'),
                           ('r', 'rectification_matrix'), ('p', 'projection_matrix')]:
            setattr(m, field, [float(x) for x in d[key]['data']])
        return m

    def acquisition_session(self, backend_epoch):
        # Source headers identify a backend PROCESS, not a device preview.
        # Namespace by this observer so driver restarts cannot reuse sequences.
        return uid(uuid.uuid5(self.driver_session, str(backend_epoch)))

    def start_worker(self, name, fn):
        worker = threading.Thread(target=fn, name='z70_driver_'+name)
        with self.worker_lock:
            self.workers = [w for w in self.workers if w.is_alive()]
            self.workers.append(worker)
            worker.start()

    def request(self, method, path, timeout=5, **kw):
        allow_error = kw.pop('allow_error', False)
        if (not self.dispatcher.session_enabled and path not in
                ('/api/driver/control/session', '/api/driver/rpc', '/api/commands/events')):
            raise requests.RequestException('SESSION_STOPPED')
        r = requests.request(method, self.origin+path, headers=self.auth, timeout=timeout, **kw)
        if not allow_error: r.raise_for_status()
        return r

    def evidence(self, stream, source_ms=None, kind=0, session=None, sequence=None):
        with self.sample_lock:
            selected_session = session or self.session
            key = (stream, bytes(selected_session.uuid))
            if sequence is None:
                self.sequences[key] = self.sequences.get(key, 0)+1
                sequence = self.sequences[key]
                while len(self.sequences)>128: self.sequences.pop(next(iter(self.sequences)))
            e = EvidenceStamp(received_at=host_time(time.time()), valid=True,
                              session_id=selected_session, sequence=sequence,
                              timestamp_kind=kind)
        if source_ms is not None:
            e.source_at = host_ms(source_ms)
            e.source_time_known = True; e.source_clock = 1
        return e

    def header(self, msg, frame='base_footprint'):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame

    def diagnostic(self, category, code, level=1):
        d = DiagnosticStatus(name='z70_driver/'+category, level=bytes([level]),
                             message=code, hardware_id='z70', values=[KeyValue(key='code', value=code)])
        m = DiagnosticArray(status=[d]); self.header(m, 'base_footprint')
        self.pub_diagnostics.publish(m)

    def remember(self, name, msg, pub):
        with self.sample_lock:
            self.samples[name] = (msg, pub)
            self.good_at[name] = time.monotonic()
            pub.publish(msg)

    def gap(self, name):
        with self.sample_lock:
            if name in self.samples:
                msg, pub = self.samples[name]
                msg.evidence.valid = False; msg.evidence.stale = True; msg.evidence.reason = 'OBSERVATION_GAP'
                if name == 'state':
                    msg.connected = False; msg.transport_state = 3
                if name == 'arm':
                    msg.validity.value = 4; msg.validity.reason = 'OBSERVATION_GAP'
                pub.publish(msg)
        self.diagnostic(name, 'OBSERVATION_GAP')

    def _publish_display(self, sample=None):
        with self.sample_lock:
            if sample is not None:
                self.display_sample = sample
                fresh = sample
            elif self.display_sample is not None:
                fresh = deepcopy(self.display_sample)
                fresh.header.stamp = self.get_clock().now().to_msg()
            else:
                return
        self.pub_js.publish(fresh)
        self.pub_display.publish(fresh)

    def check_stale(self):
        self._publish_display()
        with self.sample_lock:
            for name in list(self.samples):
                limit = self.camera_stale if name == 'camera' else self.state_stale
                if name == 'map':
                    continue  # Source freshness is unknown, independent of cache receipt age.
                if time.monotonic()-self.good_at[name] > limit:
                    self.gap(name)

    def state_loop(self):
        while not self.stop_event.is_set():
            state_available = False
            try:
                d = self.request('GET', '/api/state', timeout=8).json()
                self.dashboard.state = d
                self.dashboard.state_received = time.time()
                if d.get('observationSession') and d['observationSession'] != self.backend_session:
                    if self.backend_session is not None:
                        with self.estimator_lock: self.estimator.invalidate('SESSION_CHANGED')
                    with self.sample_lock:
                        if self.backend_session is not None:
                            self.diagnostic('state', 'SESSION_CHANGED')

                            for name in ('state','selection','arm','map'): self.gap(name)
                        self.backend_session = d['observationSession']
                        self.session = self.acquisition_session(self.backend_session)
                telemetry = d.get('telemetry', {})
                entry = telemetry.get('status', {})
                values = entry.get('value')
                m = RobotState(); self.header(m, 'base_footprint')
                m.evidence = self.evidence('state', entry.get('observedAt'))
                connection = d.get('connection', {}).get('state', 'disconnected')
                m.connected = connection == 'connected'
                m.transport_state = {'disconnected': 0, 'connecting': 1, 'connected': 2}.get(connection, 3)
                if (not isinstance(values, dict) or entry.get('observedAt') is None or
                    not 0 <= time.time()-entry['observedAt']/1000 <= self.state_stale):
                    self.gap('state')
                else:
                    for source, field in [('state','robot_state'), ('error_code','error_code'),
                                          ('common_status','common_status'), ('grab_status','grab_status')]:
                        v = values.get(source)
                        if isinstance(v, int) and not isinstance(v, bool) and 0 <= v <= 0xffffffff:
                            setattr(m, field, v); setattr(m, field+'_known', True)
                    if m.common_status_known and not m.grab_status_known:
                        m.grab_status = (m.common_status >> 6) & 0xf
                        m.grab_status_known = True
                    battery = values.get('battery')
                    if isinstance(battery, (float, int)) and 0 <= battery <= 100:
                        m.battery_known = True; m.battery_fraction = float(battery/100)
                    for source, known in [('in_cleaning','cleaning_known'),('in_returning','returning_known')]:
                        value=values.get(source)
                        if isinstance(value,(bool,int)) and value in (False,True,0,1):
                            setattr(m,known,True);setattr(m,source,bool(value))
                    self.remember('state', m, self.pub_state)
                    state_available = m.connected
                    with self.estimator_lock:
                        common = values.get('common_status')
                        if (isinstance(common, int)
                                and ((common & 256) or (((common >> 6) & 15) in (1, 2, 3, 4) and ((common >> 10) & 3) in (1, 2)))
                                and self.estimator.data.get('state') in ('UNKNOWN', 'IN_ROBOT')
                                and self.estimator.data.get('unknownReason') != 'GRIP_POSE_UNVERIFIED'):
                            if float(self.estimator.data.get('joints', {}).get('m3', 0)) < LIMITS['m3'][0]:
                                self.estimator._anchor('WORKING_POS', 'common_status_deployed')
                            else:
                                self.estimator.data.update(state='IN_PROCESS_POS', restored=False, unknownReason=None)
                                self.estimator._commit('common_status_deployed', 'RECONCILED_DEPLOYED')
                        self.estimator.observe_status(values, entry.get('requestedAt',entry['observedAt']))
                ce = telemetry.get('camera', {})
                cv = ce.get('value')
                # Status responses can be a one-element numeric array or a named numeric field.
                if isinstance(cv, list) and len(cv) == 1: cv = cv[0]
                if isinstance(cv, dict): cv = cv.get('camera_status')
                if (isinstance(cv, int) and ce.get('observedAt') is not None and
                    0 <= time.time()-ce['observedAt']/1000 <= self.state_stale):
                    c = CameraState(selected_camera=2 if cv & 32768 else 1, selection_confirmed=True)
                    self.header(c); c.evidence = self.evidence('camera_state', ce['observedAt'])
                    self.remember('selection', c, self.pub_camera_state)
                else: self.gap('selection')
            except (requests.RequestException, ValueError, TypeError, KeyError):
                self.gap('state'); self.gap('selection')
            try:
                if state_available:
                    with self.estimator_lock:
                        self.publish_estimate(self.estimator.snapshot())
                else:
                    self.gap('arm')
            except (requests.RequestException, ValueError, TypeError, KeyError):
                self.gap('arm')
            self.stop_event.wait(1/max(.1, self.state_hz))

    def publish_estimate(self, d):
        self.dashboard.arm = d
        self.dashboard.arm_received = time.time()
        if d.get('persistenceError'): self.diagnostic('arm','PERSISTENCE_FAILED',2)
        updated = d.get('updatedAt')
        with self.sample_lock:
            old = self.samples.get('arm')
            same_update = (old is not None and self.arm_payload == d and
                           old[0].evidence.session_id == self.session and
                           old[0].evidence.source_time_known and old[0].evidence.source_at == host_ms(updated or 0))
            if same_update and old[0].evidence.valid and not old[0].evidence.stale:
                # Polling an unchanged estimate must not mint a new update stamp/identity.
                self.good_at['arm'] = time.monotonic()
                self._publish_display()
                return
        m = ArmEstimate(source=self.estimate_source, model_id='vendor-app', restored=bool(d.get('restored')))
        self.header(m, 'vendor_arm_planar'); m.evidence = self.evidence('arm', updated, 3)
        if same_update:
            # Recovery of the same cached model update changes availability, not
            # its command-derived timestamp or identity.
            m.header = deepcopy(old[0].header); m.evidence = deepcopy(old[0].evidence)
            m.evidence.valid = True; m.evidence.stale = False; m.evidence.reason = ''
        usable = d.get('state') not in ('UNKNOWN','IN_ROBOT') and not m.restored
        m.validity.value = 1 if usable else 0
        m.validity.reason = 'COMMAND_DEAD_RECKONING' if usable else 'UNRECONCILED_OR_UNKNOWN'
        if d.get('state') == 'IN_ROBOT' and not m.restored:
            m.validity.value = 3; m.validity.reason = 'COMMAND_DERIVED_STOW'
        m.unknown_reason = '' if usable else 'UNRECONCILED_OR_UNKNOWN'
        q = app_to_urdf(**estimate_to_app(d))
        display = JointState(); display.header = deepcopy(m.header)
        display.name = [*q, 'shoulder_lift', 'shoulder_yaw', 'arm_lid']
        display.position = [*q.values(), 0., 0., 0.]
        if usable:
            # Only the three arm joints have even heuristic uncertainty in ArmEstimate.
            m.joints = JointState(header=deepcopy(m.header), name=list(q)[:3], position=list(q.values())[:3])
            sigma = d.get('sigmaDeg', {})
            if all(isinstance(sigma.get(j), (int,float)) and math.isfinite(sigma[j]) and sigma[j]>=0 for j in ('m3','m4','m5')):
                m.joint_uncertainty_known = True
                m.joint_sigma_rad = [math.radians(sigma[j]) for j in ('m3','m4','m5')]
        self._publish_display(display)
        m.steps_since_anchor = int(d.get('stepsSinceAnchor', 0))
        if d.get('anchoredAt') is not None:
            m.anchor_time_known = True; m.anchored_at = host_ms(d['anchoredAt'])
        # Legacy wrist is vendor 2-D app geometry, not a measured ROS point/covariance.
        self.arm_payload = deepcopy(d)
        self.remember('arm', m, self.pub_arm)

    def camera_loop(self):
        while not self.stop_event.is_set():
            start = time.monotonic()
            try:
                with self.camera_lock:
                    if self.stop_event.is_set(): break
                    generation = self.acquisition_generation
                    r = self.request('POST', '/api/camera/frame', json={}, timeout=20)
                    if self.dispatcher.session_enabled and generation == self.acquisition_generation:
                        self.publish_frame(r)
            except (requests.RequestException, ValueError, TypeError, KeyError):
                self.gap('camera')
            self.stop_event.wait(max(0., 1/self.camera_hz-(time.monotonic()-start)))

    def publish_frame(self, r):
        camera = r.headers.get('X-Roboclaw-Camera')
        if camera not in CAMERAS:
            raise ValueError('unlabelled frame')
        image = cv2.imdecode(np.frombuffer(r.content, np.uint8), cv2.IMREAD_COLOR)
        if image is None: raise ValueError('invalid JPEG')
        height, width = image.shape[:2]
        session = self.acquisition_session(r.headers['X-Roboclaw-Camera-Session']) if r.headers.get('X-Roboclaw-Camera-Session') else self.session
        meta = FrameMetadata(camera=CAMERAS[camera], width=width, height=height,
                             session_scope=1 if r.headers.get('X-Roboclaw-Camera-Session') else 0)
        self.header(meta, 'camera_optical_frame' if camera == 'front' else 'gripper_camera_optical_frame')
        meta.evidence = self.evidence('frame', session=session)
        # Backend received-at is host receipt, never exposure time.
        if r.headers.get('X-Roboclaw-Received-At'):
            meta.evidence.received_at = host_ms(r.headers['X-Roboclaw-Received-At'])
        info = self.calibrations.get(camera)
        if info and (info[0].width, info[0].height) == (width, height):
            calibrated = deepcopy(info[0]); calibrated.header = deepcopy(meta.header)
            meta.calibration_id = info[1]; self.pub_info[camera].publish(calibrated)
        else:
            meta.evidence.valid = False; meta.evidence.reason = 'CALIBRATION_MISMATCH'
            self.diagnostic('camera', 'CALIBRATION_MISMATCH')
        compressed = CompressedImage(header=meta.header, format='jpeg', data=r.content)
        raw = Image(header=meta.header, height=height, width=width, encoding='bgr8',
                    step=width*3, data=image.tobytes())
        self.pub_img[camera].publish(compressed); self.pub_raw[camera].publish(raw)
        self.remember('camera', meta, self.pub_metadata[camera])
        state = CameraState(header=meta.header, selected_camera=CAMERAS[camera], selection_confirmed=True,
                            evidence=deepcopy(meta.evidence))
        # Lens tagging valid even if the calibration is missing/mismatched.
        state.evidence.valid = True; state.evidence.reason = ''
        self.remember('selection', state, self.pub_camera_state)

    def service_outcome(self, operation):
        now = host_time(time.time())
        return CommandOutcome(command_id=uid(uuid.uuid4()), attempt_id=uid(uuid.uuid4()),
                              requested_at=now, error_code='NONE')

    def select(self, request, response):
        outcome = self.service_outcome('select_camera')
        if not self.dispatcher.session_enabled:
            outcome.error_code = 'SESSION_STOPPED'; response.outcome = outcome
            return response
        if request.camera not in (1,2):
            outcome.error_code = 'INVALID_INPUT'; response.outcome = outcome
            return response
        # Do not queue indefinitely behind a hung image response.
        if not self.camera_lock.acquire(timeout=.1):
            outcome.error_code = 'BUSY'; response.outcome = outcome
            return response
        try:
            camera = next(k for k,v in CAMERAS.items() if v == request.camera)
            result = self.request('POST', '/api/driver/camera/select', json={'camera':camera}, timeout=19).json()
            if result.get('camera') != camera: raise ValueError('unconfirmed lens')
            state = CameraState(selected_camera=request.camera, selection_confirmed=True)
            self.header(state); state.evidence = self.evidence('camera_state')
            self.remember('selection', state, self.pub_camera_state)
            rows=result.get('outcomes',[])
            wire=next((r['outcome'] for r in rows if r['method']=='app_change_camera'),None)
            if wire is not None: outcome=self.backend_outcome(wire)
            else:
                outcome=self.local_outcome() # no selector write was needed
            # A confirmed bit flip is selection evidence, not a fabricated RPC reply.
            # Expected firmware timeout remains UNKNOWN/TIMEOUT in the outcome.
            response.state = state
        except (requests.RequestException, ValueError):
            outcome.dispatch = 2; outcome.response = 3; outcome.error_code = 'OBSERVATION_GAP'
            self.gap('selection')
        finally: self.camera_lock.release()
        outcome.finished_at = host_time(time.time()); response.outcome = outcome
        return response

    def read_map(self, refresh=False):
        if not self.map_lock.acquire(blocking=False): raise BlockingIOError('map acquisition busy')
        try:
            if refresh: self.request('POST', '/api/map/cache/refresh', json={}, timeout=30)
            r = self.request('GET', '/api/map/cache/raw', timeout=3)
            m = MapSnapshot(raw=list(r.content), map_index=int(r.headers['X-Map-Index']),
                            map_sequence=int(r.headers['X-Map-Sequence']))
            self.header(m, 'map')
            session=self.acquisition_session(r.headers['X-Roboclaw-Observation-Session']) if r.headers.get('X-Roboclaw-Observation-Session') else self.session
            m.evidence = self.evidence('map', kind=2, session=session)
            # X-Map-Saved-At is CACHE receipt; NOT vendor capture/source time.
            m.evidence.received_at = host_ms(r.headers['X-Map-Saved-At'])
            m.evidence.reason = 'SOURCE_FRESHNESS_UNKNOWN'
            try:
                rooms = self.request('GET', '/api/home/rooms', timeout=2).json().get('rooms', [])
                _, mapping = self.driver_rpc('get_room_mapping', [], 2000)
                m.rooms = [RoomLabel(segment_id=seg, name=name[:128])
                           for seg, name in room_names(mapping or [], rooms).items()]
                if m.rooms and m.map_index >= 0 and m.map_sequence > 0:
                    decoded = decode_map(bytes(m.raw))
                    with self.sample_lock:
                        cur_state = self.samples.get('state')
                    status_vals = (self.dashboard.state or {}).get('telemetry', {}).get('status', {}).get('value') or {}
                    locating = bool(status_vals.get('is_locating', 0)) or bool(status_vals.get('is_exploring', 0))
                    r_state = cur_state[0].robot_state if cur_state and cur_state[0].robot_state_known else status_vals.get('state')
                    if (decoded.robot is not None and len(decoded.robot) == 3 and
                            all(v is not None and math.isfinite(v) for v in decoded.robot) and
                            bool(decoded.rooms) and len(decoded.pixels) > 64 and
                            not locating and r_state not in (22,)):
                        m.active_floor_verified = True
                        m.current_pose_verified = True
            except (requests.RequestException, ValueError, TypeError, KeyError):
                self.diagnostic('map', 'ROOM_LABELS_UNAVAILABLE')
            with self.sample_lock:
                old = self.samples.get('map')
                if (not refresh and old and old[0].evidence.valid and not old[0].evidence.stale and
                    old[0].evidence.session_id == m.evidence.session_id and
                    old[0].raw == m.raw and old[0].rooms == m.rooms):
                    return old[0]  # Preserve original evidence when cache did not change.
            self.remember('map', m, self.pub_map)
            return m
        finally: self.map_lock.release()

    def map_loop(self):
        first = True
        while not self.stop_event.is_set():
            try: self.read_map(refresh=not first)
            except BlockingIOError: pass
            except (requests.RequestException, ValueError, KeyError, TypeError):
                if first: self.gap('map')
            first = False
            self.stop_event.wait(self.map_interval)

    def refresh(self, request, response):
        outcome = self.service_outcome('refresh_map')
        try:
            response.snapshot = self.read_map(refresh=True).evidence
            outcome.dispatch = 1; outcome.response = 1
        except (BlockingIOError, requests.RequestException, ValueError, KeyError, TypeError):
            with self.sample_lock:
                cached = self.samples.get('map')
            if cached and cached[0].evidence.valid and not cached[0].evidence.stale:
                m = deepcopy(cached[0])
                self.header(m, 'map')
                m.evidence = self.evidence('map', kind=2, session=m.evidence.session_id)
                m.evidence.received_at = cached[0].evidence.received_at
                m.evidence.reason = cached[0].evidence.reason
                self.remember('map', m, self.pub_map)
                response.snapshot = m.evidence
                outcome.dispatch = 1; outcome.response = 1
            else:
                self.gap('map'); outcome.dispatch = 2; outcome.response = 3; outcome.error_code = 'OBSERVATION_GAP'
                if cached: response.snapshot = cached[0].evidence
        outcome.finished_at = host_time(time.time()); response.outcome = outcome
        return response

    def publish_event(self, row):
        session, sequence = row['sessionId'], int(row['sequence'])
        if session == self.event_session and self.event_sequence is not None:
            if sequence <= self.event_sequence: return
            if sequence != self.event_sequence+1:
                self.diagnostic('events', 'EVENT_GAP')
                with self.estimator_lock: self.estimator.invalidate('COMMAND_STREAM_GAP')
        elif self.event_session is not None:
            self.diagnostic('events', 'SESSION_CHANGED')
            with self.estimator_lock: self.estimator.invalidate('SESSION_CHANGED')
        self.event_session, self.event_sequence = session, sequence
        msg = CommandEvent(operation=row['operation'])
        msg.evidence = self.evidence('events', source_ms=row.get('attemptedAt'), session=uid(session), sequence=sequence)
        msg.phase = {'accepted':0,'dispatched':1,'replied':2,'finished':4}[row['phase']]
        msg.outcome = self.backend_outcome(row)
        p = row.get('params')
        if p and row['operation'] == 'app_rc_move':
            msg.requested_twist.linear.x = float(p['velocity']); msg.requested_twist.angular.z = float(p['omega'])
            ns = int(float(p['duration'])*1e6)
            msg.requested_duration.sec = ns//10**9; msg.requested_duration.nanosec = ns%10**9
        self.pub_events.publish(msg)

    def events_loop(self):
        while not self.stop_event.is_set():
            try:
                with self.request('GET', '/api/commands/events', timeout=(3, 2), stream=True) as r:
                    self.stream_response = r
                    for line in r.iter_lines(chunk_size=1):
                        if self.stop_event.is_set(): break
                        if len(line) > 8192: raise ValueError('oversized event')
                        if line: self.publish_event(json.loads(line))
            except (requests.RequestException, ValueError, TypeError, KeyError, AttributeError):
                if not self.stop_event.is_set():
                    self.diagnostic('events', 'OBSERVATION_GAP')
                    with self.estimator_lock:
                        self.estimator.invalidate('COMMAND_STREAM_GAP')
                        self.publish_estimate(self.estimator.snapshot())
            finally: self.stream_response = None
            self.stop_event.wait(1.)

    def close(self):
        self.stop_event.set()
        # Network reads have finite deadlines; let their owning thread close the response.
        deadline = time.monotonic()+130
        while time.monotonic() < deadline:
            with self.worker_lock:
                workers = list(self.workers)
            alive = [w for w in workers if w.is_alive()]
            if not alive: break
            for worker in alive:
                worker.join(timeout=max(0., min(.2, deadline-time.monotonic())))
        if any(w.is_alive() for w in self.workers):
            raise RuntimeError('driver worker failed to stop within network deadline')

    def destroy_node(self):
        for server in self.action_servers:
            server.destroy()
        self.action_servers.clear()
        return super().destroy_node()


def main():
    rclpy.init(); node = Driver(); executor = MultiThreadedExecutor(num_threads=8); executor.add_node(node)
    try: executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        node.close(); executor.shutdown(); node.destroy_node(); rclpy.try_shutdown()
