"""Fail-closed native Pin-and-Go. Safe-grid publication is not firmware enforcement.

A single-hop detour is permitted only in a physically/vendor-barricaded connected
interior. An open exterior anywhere in that component makes native detours
unverifiable, even if our own A* could choose a safe path.
"""
import math
import time
from collections import deque

import numpy as np
from z70_mapping.boundary import BoundaryConfig, compute_safe_interior, check_corridor_containment


class NavigationRejected(Exception):
    pass


def cell(result, xy):
    x, y = np.floor((np.asarray(xy)-result.origin)/result.resolution).astype(int)
    return (int(x), int(y))


def inside(result, xy, margin=0.):
    x, y = cell(result, xy)
    return (0 <= x < result.width and 0 <= y < result.height
            and result.reachable_mask[y, x]
            and result.clearance_m[y, x]+1e-9 >= result.config.clearance_m+margin)


def route_contained(m, result, start, target):
    if check_corridor_containment(result, start, target, radius_m=0.):
        return True
    # Four-connected reachability, with no diagonal corner cutting. Reject any
    # open exterior connected to this interior: firmware may choose that shortcut.
    seed = cell(result, start); end = cell(result, target)
    if not inside(result, start) or not inside(result, target): return False
    pending=deque([seed]); seen={seed}; found=False
    while pending:
        x,y=pending.popleft()
        found |= (x,y)==end
        for nx,ny in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
            if (0 <= nx < result.width and 0 <= ny < result.height
                    and result.reachable_mask[ny,nx] and (nx,ny) not in seen):
                seen.add((nx,ny)); pending.append((nx,ny))
    if not found: return False
    # Flood raw interior from the route's component up to its actual barriers.
    pending=deque([seed]); seen={seed}
    while pending:
        x,y=pending.popleft()
        for nx,ny in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
            if not (0 <= nx < result.width and 0 <= ny < result.height): return False
            if m.occupancy[ny,nx] == 100 or result.forbidden_mask[ny,nx]: continue
            if not result.interior_mask[ny,nx]: return False
            if (nx,ny) not in seen:
                seen.add((nx,ny)); pending.append((nx,ny))
    return True


class PinAndGoNavigator:
    def __init__(self, observations, commands=None, *, boundary_config=None,
                 max_pose_age_s=5., arrival_tolerance_m=.25, stopping_margin_m=.05,
                 poll_s=.1, settle_s=.5, cancelled=lambda:False, hazard=lambda:None,
                 odom=lambda:None, feedback=lambda **_:None, clock=time.monotonic, sleep=time.sleep):
        self.observations=observations; self.commands=commands
        self.config=boundary_config or BoundaryConfig()
        self.max_pose_age_s=max_pose_age_s; self.arrival_tolerance_m=arrival_tolerance_m
        self.stopping_margin_m=stopping_margin_m; self.poll_s=poll_s; self.settle_s=settle_s
        self.cancelled=cancelled; self.hazard=hazard; self.odom=odom; self.feedback=feedback
        self.clock=clock; self.sleep=sleep; self.position=[0.,0.]; self.target=(0.,0.)
        self.latest_observation=None

    def result(self, code, detail=''):
        return dict(status_code=code, arrived=code=='ARRIVED', dry_run_valid=code=='DRY_RUN_VALID',
                    final_position_m=list(self.position), position_error_m=math.dist(self.position,self.target),
                    detail=(detail or code)[:128])

    def status(self):
        return self.observations.get('/api/state')['telemetry']['status']['value']

    def stowed(self, status):
        common=status.get('common_status')
        return (self.observations.get('/api/arm/estimate').get('state')=='IN_ROBOT'
                and type(common) is int and ((common >> 6)&15)==0
                and status.get('error_code')==0 and status.get('battery',0)>15
                and status.get('in_cleaning') is False and status.get('in_returning') is False
                and status.get('state') in (3,16))

    def snapshot(self, refresh=False):
        s=self.observations.navigation_snapshot(refresh=refresh, max_age_s=self.max_pose_age_s)
        m=s.map
        if (not s.active_floor_verified or not s.current_pose_verified
                or not 0 <= s.cache_age_s <= self.max_pose_age_s
                or m.robot is None or len(m.robot)!=3 or not all(v is not None and math.isfinite(v) for v in m.robot)
                or not m.rooms):
            raise NavigationRejected('UNLOCALIZED_MAP')
        self.latest_observation=s.evidence
        self.position=list(m.robot[:2])
        return s

    def validate(self, s):
        m=s.map
        b=compute_safe_interior(m,s.labels,self.config,m.robot)
        if b.selection_method not in ('robot_safe_cell','nearest_safe_cell') or not inside(b,m.robot[:2]):
            raise NavigationRejected('UNLOCALIZED_MAP')
        if not inside(b,self.target): raise NavigationRejected('TARGET_OUTSIDE_INTERIOR')
        if not route_contained(m,b,m.robot[:2],self.target):
            raise NavigationRejected('ROUTE_CONTAINMENT_UNVERIFIED')
        return b

    def monitor_boundary(self, baseline, boundary, snapshot):
        m=baseline.map; current=snapshot.map
        same_frame=(getattr(snapshot,'map_index',None)==getattr(baseline,'map_index',None)
                    and getattr(snapshot,'session_id',None)==getattr(baseline,'session_id',None)
                    and current.origin==m.origin and current.width==m.width
                    and current.height==m.height and current.resolution==m.resolution)
        try:
            valid=(same_frame and inside(boundary,self.position,self.stopping_margin_m)
                   and inside(self.validate(snapshot),self.position,self.stopping_margin_m))
        except NavigationRejected:
            valid=False
        if not valid: raise NavigationRejected('BOUNDARY_ABORT')

    def stop(self):
        # Raw stop has its own driver priority bypass and is permitted after a
        # task cancellation; Commands.rpc deliberately rejects cancelled motion.
        try: self.commands.rpc('stop_goto_target',[],5000)
        except (InterruptedError,LookupError,TimeoutError): pass
        finally: self.commands.stop()

    def execute(self, goal):
        self.target=(goal.target_x_m,goal.target_y_m)
        if (not all(math.isfinite(v) and -100 <= v <=100 for v in self.target)
                or not math.isfinite(goal.max_duration_s) or not 1 <= goal.max_duration_s <=600
                or type(goal.dry_run) is not bool): return self.result('INVALID_INPUT')
        moving=False; deadline=None
        try:
            if not self.stowed(self.status()): return self.result('ARM_NOT_STOWED')
            baseline=self.snapshot(refresh=not goal.dry_run)
            boundary=self.validate(baseline)
            if goal.dry_run: return self.result('DRY_RUN_VALID')
            if self.cancelled(): return self.result('CANCELLED')
            if self.hazard(): return self.result('BOUNDARY_ABORT')
            if not self.stowed(self.status()): return self.result('ARM_NOT_STOWED')
            initial=list(self.position); initial_odom=self.odom()
            started=self.clock(); deadline=started+goal.max_duration_s
            moving=True  # Includes uncertainty and exceptions during dispatch.
            try:
                reply=self.commands.rpc('app_goto_target',[round(v*1000) for v in self.target],5000)
            except TimeoutError:
                self.stop(); moving=False; return self.result('UNKNOWN_DISPATCH')
            if reply.get('dispatch')=='unknown':
                self.stop(); moving=False; return self.result('UNKNOWN_DISPATCH')
            if reply.get('response')=='vendor_error':
                moving=False; return self.result('VENDOR_REJECTED')
            if reply.get('dispatch')!='sent' or reply.get('response')!='received':
                self.stop(); moving=False; return self.result('DISPATCH_FAILED')
            saw_motion=False
            while self.clock()<deadline:
                if self.cancelled():
                    timed_out=self.clock()>=deadline
                    self.stop(); moving=False; return self.result('TIMED_OUT' if timed_out else 'CANCELLED')
                if self.hazard():
                    self.stop(); moving=False; return self.result('BOUNDARY_ABORT')
                status=self.status()
                if not self.stowed(status):
                    self.stop(); moving=False; return self.result('BOUNDARY_ABORT')
                s=self.snapshot(refresh=True)
                # Freeze baseline geometry/identity. Never silently establish a
                # different coordinate frame during native navigation.
                self.monitor_boundary(baseline,boundary,s)
                state=status.get('state'); saw_motion |= state==16 or math.dist(initial,self.position)>.03
                self.feedback(phase=3,reason='MONITORING',current_position_m=list(self.position),
                              remaining_distance_m=math.dist(self.position,self.target))
                if state==3 and (saw_motion or self.clock()-started>=self.settle_s):
                    # A service completion or 16 -> 3 transition isn't arrival.
                    final=self.snapshot(refresh=True)
                    if final.sequence<=s.sequence: raise NavigationRejected('UNLOCALIZED_MAP')
                    self.monitor_boundary(baseline,boundary,final)
                    if self.cancelled() or self.clock()>=deadline:
                        timed_out=self.clock()>=deadline
                        self.stop(); moving=False; return self.result('TIMED_OUT' if timed_out else 'CANCELLED')
                    final_status=self.status()
                    if self.hazard() or final_status.get('state')!=3 or not self.stowed(final_status):
                        self.stop(); moving=False; return self.result('BOUNDARY_ABORT')
                    error=math.dist(self.position,self.target)
                    current_odom=self.odom()
                    corroborated=True
                    if initial_odom is not None and current_odom is not None:
                        displacement=math.dist((initial_odom['x'],initial_odom['y']), (current_odom['x'],current_odom['y']))
                        corroborated=abs(displacement-math.dist(initial,self.position))<=self.arrival_tolerance_m
                    moving=False
                    return self.result('ARRIVED' if error<=self.arrival_tolerance_m and corroborated else 'STOPPED_SHORT')
                self.sleep(self.poll_s)
            self.stop(); moving=False; return self.result('TIMED_OUT')
        except InterruptedError:
            timed_out=deadline is not None and self.clock()>=deadline
            if moving: self.stop()
            return self.result('BOUNDARY_ABORT' if self.hazard() else 'TIMED_OUT' if timed_out else 'CANCELLED')
        except (NavigationRejected,LookupError,TimeoutError,ValueError,TypeError,KeyError) as exc:
            if moving: self.stop()
            return self.result(str(exc) if isinstance(exc,NavigationRejected) else 'UNLOCALIZED_MAP')
