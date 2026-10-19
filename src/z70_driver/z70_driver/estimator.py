"""Command-derived nominal vendor geometry, never encoder/visual feedback.

Persistence is a versioned ROS-owned session. Restored joints are history only
until an explicit anchor; official-app joint jogs are intrinsically unobservable.
Measured physical transforms and folded display poses do not enter this model.
"""
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import tempfile
import time
from typing import Any
from z70_description.arm_model import (
    POSES, LIMITS, JOINT_STEP_DEG, HAND_STEP_M, vendor_forward, vendor_inverse, vendor_check,
)

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
                # Do not trust serialized geometry; derive only from nominal joints.
                self._wrist()
            except (OSError, ValueError, TypeError, KeyError):
                pass

    def snapshot(self):
        return deepcopy(self.data)

    def _wrist(self):
        # Legacy wire key "wrist": app end_pos, NOT the CAD arm_wrist_link TF.
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
                # A robot command may already have run. Do not lose its live
                # outcome because storage failed or pretend a restart is safe.
                d['persistenceError']=True
            finally:
                if name and os.path.exists(name): os.unlink(name)
        return True

    def invalidate(self, reason='UNOBSERVED_MOTION'):
        self.data['state'] = 'UNKNOWN'
        # Only bounded internal categories are retained, never caller exception text.
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

    def _apply(self, joints, n, method):
        current = self.data['joints']
        new_x = vendor_forward(joints['m3'], joints['m4'])[1][0]
        # Predictive firmware-limit model, not evidence that a particular jog ran.
        if new_x > .367 and new_x > vendor_forward(current['m3'], current['m4'])[1][0]: return False
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
            if self.data['state'] != 'IN_ROBOT' and float(self.data['joints'].get('m3', 0)) >= LIMITS['m3'][0]:
                current = self.data['joints']
                new_x = vendor_forward(joints['m3'], joints['m4'])[1][0]
                if not (new_x > .367 and new_x > vendor_forward(current['m3'], current['m4'])[1][0]):
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
        if not self._movable():
            current = self.data['joints']
            new_x = vendor_forward(joints['m3'], joints['m4'])[1][0]
            if not (new_x > .367 and new_x > vendor_forward(current['m3'], current['m4'])[1][0]):
                self.data['joints'] = joints
            return False
        if not self._apply(joints,n,method): return False
        for key in ('m3','m4'): self.data['sigmaDeg'][key] = math.hypot(self.data['sigmaDeg'][key],2.5*math.sqrt(n))
        return self._commit(method, 'HEURISTIC_UNCERTAINTY')

    def observe(self, method, params, outcome, command_id=None):
        if command_id:
            if command_id in self.seen: return False
            self.seen.add(command_id)
            if len(self.seen)>4096: self.seen = {command_id}
        if outcome in ('not_sent','vendor_error'): return False
        if method not in ARM_METHODS:
            if method == 'app_pick_up_enter_exit':
                p = params[0] if isinstance(params,list) and len(params)==1 else params
                if outcome == 'received' and isinstance(p,dict) and type(p.get('flag')) is int and p['flag'] in (0,1,2):
                    self.grab_mode = p['flag']
                return False
            if method.startswith(('get_','app_get_','app_rc_')) or method == 'app_change_camera': return False
            # Advanced unknown programs/operations cannot silently preserve a
            # planning pose. This does not reject or retry the vendor command.
            return self.invalidate('UNOBSERVED_MOTION')
        if method == 'app_grip': return self.invalidate('GRIP_POSE_UNVERIFIED') # close is NOT a lift
        if outcome == 'unknown':
            if method == 'app_open_gripper': return False # wrist unchanged; no finger certainty
            return self.invalidate('UNKNOWN_DISPATCH')
        p = params[0] if isinstance(params,list) and len(params)==1 else params
        if method == 'app_arm_in_compartment': return self._anchor('IN_ROBOT',method)
        if method == 'app_arm_out_compartment':
            if self.data['state']=='IN_ROBOT' and not self.data['restored']:
                return self._anchor('WORKING_POS',method)
            self.data['joints'] = dict(POSES['WORKING_POS'])
            return False
        if method == 'app_open_gripper':
            if not self._movable(): return False
            self.data['joints']['m6']=90; return self._commit(method,'GRIPPER_COMMAND_ONLY')
        if method == 'app_put_down' and self.grab_mode == 1 and isinstance(p,dict) and p.get('flag') == 0:
            if not self._movable():
                if self.data['state'] != 'IN_ROBOT': self.data['joints']['m6'] = 90
                return False
            self.data['joints']['m6']=90; return self._commit(method,'GRIPPER_COMMAND_ONLY')
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
                    v=c.get('value',[]); idx=next((i for i,x in enumerate(v[:4]) if type(x) in (int,float) and x in (1,2)),None) if isinstance(v,list) else None
                    if idx is not None and idx<3: changed = self._joints([v[idx] if i==idx else 0 for i in range(3)],n,method) or changed
                    elif idx==3 and self._movable():
                        self.data['joints']['m6']=90 if v[3]==1 else 0; changed=self._commit(method,'GRIPPER_COMMAND_ONLY') or changed
                elif cmd == 'arm_move': changed = self._hand(c.get('direction'),n,method) or changed
                elif cmd not in ('delay','sound','chassis_move','chassis_rotate'): changed=self.invalidate('PROGRAM_POSE_UNVERIFIED') or changed
            return changed
        if method == 'app_stop' and self.data['state']=='IN_ROBOT': return False
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
