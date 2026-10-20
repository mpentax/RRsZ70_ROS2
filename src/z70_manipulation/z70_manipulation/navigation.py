"""Pin-and-Go navigation over Roborock's native app_goto_target RPC."""
import math
import time

import numpy as np
from z70_mapping.boundary import BoundaryConfig, compute_safe_interior
from .observations import read_arm, read_status


class NavigationRejected(Exception):
    pass


def cell(result, xy):
    x, y = np.floor((np.asarray(xy) - result.origin) / result.resolution).astype(int)
    return (int(x), int(y))


def inside(result, xy, margin=0.):
    x, y = cell(result, xy)
    return (0 <= x < result.width and 0 <= y < result.height
            and bool(result.reachable_mask[y, x])
            and float(result.clearance_m[y, x]) + 1e-9 >= result.config.clearance_m + margin)


class PinAndGoNavigator:
    def __init__(self, observations, commands=None, *, boundary_config=None,
                 max_pose_age_s=5., arrival_tolerance_m=.25, stopping_margin_m=.05,
                 poll_s=.1, settle_s=.5, cancelled=lambda: False, hazard=lambda: None,
                 odom=lambda: None, feedback=lambda **_: None, clock=time.monotonic, sleep=time.sleep):
        self.observations = observations
        self.commands = commands
        self.config = boundary_config or BoundaryConfig()
        self.max_pose_age_s = max_pose_age_s
        self.arrival_tolerance_m = arrival_tolerance_m
        self.stopping_margin_m = stopping_margin_m
        self.poll_s = poll_s
        self.settle_s = settle_s
        self.cancelled = cancelled
        self.hazard = hazard
        self.odom = odom
        self.feedback = feedback
        self.clock = clock
        self.sleep = sleep
        self.position = [0., 0.]
        self.target = (0., 0.)
        self.latest_observation = None

    def result(self, code, detail=''):
        return dict(status_code=code, arrived=code == 'ARRIVED', dry_run_valid=code == 'DRY_RUN_VALID',
                    final_position_m=list(self.position), position_error_m=math.dist(self.position, self.target),
                    detail=(detail or code)[:128])

    def status(self) -> dict:
        return read_status(self.observations)

    def _arm(self) -> dict:
        return read_arm(self.observations)

    def stowed(self, status):
        common = status.get('common_status')
        return (self._arm().get('state') == 'IN_ROBOT'
                and type(common) is int and ((common >> 6) & 15) == 0
                and status.get('error_code') == 0 and status.get('battery', 0) > 15
                and status.get('in_cleaning') is False and status.get('in_returning') is False
                and status.get('state') in (3, 8, 15, 16))

    def snapshot(self, refresh=False):
        s = self.observations.navigation_snapshot(refresh=refresh, max_age_s=self.max_pose_age_s)
        m = s.map
        if (not s.active_floor_verified or not s.current_pose_verified
                or not 0 <= s.cache_age_s <= self.max_pose_age_s
                or m.robot is None or len(m.robot) != 3 or not all(v is not None and math.isfinite(v) for v in m.robot)
                or not m.rooms):
            raise NavigationRejected('UNLOCALIZED_MAP')
        self.latest_observation = s.evidence
        self.position = list(m.robot[:2])
        return s

    def validate(self, s):
        m = s.map
        b = compute_safe_interior(m, s.labels, self.config, m.robot)
        if b.selection_method not in ('robot_safe_cell', 'nearest_safe_cell') or not inside(b, m.robot[:2]):
            raise NavigationRejected('UNLOCALIZED_MAP')
        if not inside(b, self.target):
            raise NavigationRejected('TARGET_OUTSIDE_INTERIOR')
        return b

    def monitor_boundary(self, baseline, boundary, snapshot):
        m, current = baseline.map, snapshot.map
        same_frame = (getattr(snapshot, 'map_index', None) == getattr(baseline, 'map_index', None)
                      and getattr(snapshot, 'session_id', None) == getattr(baseline, 'session_id', None)
                      and current.origin == m.origin and current.width == m.width
                      and current.height == m.height and current.resolution == m.resolution)
        try:
            valid = (same_frame and inside(boundary, self.position, self.stopping_margin_m)
                     and inside(self.validate(snapshot), self.position, self.stopping_margin_m))
        except NavigationRejected:
            valid = False
        if not valid:
            raise NavigationRejected('BOUNDARY_ABORT')

    def stop(self):
        try:
            self.commands.rpc('stop_goto_target', [], 5000)
        except (InterruptedError, LookupError, TimeoutError):
            pass
        finally:
            self.commands.stop()

    def execute(self, goal):
        self.target = (goal.target_x_m, goal.target_y_m)
        if (not all(math.isfinite(v) and -100 <= v <= 100 for v in self.target)
                or not math.isfinite(goal.max_duration_s) or not 1 <= goal.max_duration_s <= 600
                or type(goal.dry_run) is not bool):
            return self.result('INVALID_INPUT')
        moving, deadline = False, None
        def abort(code):
            nonlocal moving
            if moving: self.stop(); moving = False
            return self.result(code)
        try:
            if not self.stowed(self.status()):
                return self.result('ARM_NOT_STOWED')
            baseline = self.snapshot(refresh=not goal.dry_run)
            boundary = self.validate(baseline)
            if goal.dry_run:
                return self.result('DRY_RUN_VALID')
            if self.cancelled():
                return self.result('CANCELLED')
            if self.hazard():
                return self.result('BOUNDARY_ABORT')
            if not self.stowed(self.status()):
                return self.result('ARM_NOT_STOWED')
            initial = list(self.position)
            started = self.clock()
            deadline = started + goal.max_duration_s
            moving = True
            try:
                reply = self.commands.rpc('app_goto_target', [round(v * 1000) for v in self.target], 5000)
            except TimeoutError:
                return abort('UNKNOWN_DISPATCH')
            if reply.get('dispatch') == 'unknown':
                return abort('UNKNOWN_DISPATCH')
            if reply.get('response') == 'vendor_error':
                moving = False; return self.result('VENDOR_REJECTED')
            if reply.get('dispatch') != 'sent' or reply.get('response') != 'received':
                return abort('DISPATCH_FAILED')
            saw_motion = False
            while self.clock() < deadline:
                if self.cancelled():
                    return abort('TIMED_OUT' if self.clock() >= deadline else 'CANCELLED')
                if self.hazard() or not self.stowed(status := self.status()):
                    return abort('BOUNDARY_ABORT')
                s = self.snapshot(refresh=True)
                self.monitor_boundary(baseline, boundary, s)
                state = status.get('state')
                saw_motion |= state in (15, 16) or math.dist(initial, self.position) > .03
                self.feedback(phase=3, reason='MONITORING', current_position_m=list(self.position),
                              remaining_distance_m=math.dist(self.position, self.target))
                if state == 3 and (saw_motion or self.clock() - started >= self.settle_s):
                    final = self.snapshot(refresh=True)
                    self.monitor_boundary(baseline, boundary, final)
                    if self.cancelled() or self.clock() >= deadline:
                        return abort('TIMED_OUT' if self.clock() >= deadline else 'CANCELLED')
                    final_status = self.status()
                    if self.hazard() or final_status.get('state') != 3 or not self.stowed(final_status):
                        return abort('BOUNDARY_ABORT')
                    moving = False
                    error = math.dist(self.position, self.target)
                    return self.result('ARRIVED' if error <= self.arrival_tolerance_m else 'STOPPED_SHORT')
                self.sleep(self.poll_s)
            return abort('TIMED_OUT')
        except InterruptedError:
            timed_out = deadline is not None and self.clock() >= deadline
            if moving:
                self.stop()
            return self.result('BOUNDARY_ABORT' if self.hazard() else 'TIMED_OUT' if timed_out else 'CANCELLED')
        except (NavigationRejected, LookupError, TimeoutError, ValueError, TypeError, KeyError) as exc:
            if moving:
                self.stop()
            return self.result(str(exc) if isinstance(exc, NavigationRejected) else 'UNLOCALIZED_MAP')
