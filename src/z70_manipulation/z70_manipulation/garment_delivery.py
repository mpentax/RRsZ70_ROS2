"""Pin-and-Go navigation, garment approach/grasp/delivery, and patrol collection executive."""
from dataclasses import dataclass, replace
import heapq
import json
import math
import os
from pathlib import Path
import time
from types import SimpleNamespace
import uuid

import cv2
import numpy as np
from z70_mapping.boundary import BoundaryConfig, SessionWorldMap, check_corridor_containment, compute_safe_interior, mobility_from_suitability
from . import perception, planner
from .executor import forward_gain, read_arm, read_status, yaw_gain
from .perception import transform_xy


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


def dynamic_obstacles(observations, snapshot, pose_xy=None, world=None):
    dyn = getattr(snapshot, 'dynamic_obstacles', None)
    if dyn is not None:
        return dyn
    if hasattr(observations, 'dynamic_obstacles'):
        return observations.dynamic_obstacles(pose_xy)
    if world is not None and world.width > 0 and world.height > 0:
        return world.obstacle_mask(protect_xy=pose_xy)
    return None


def map_identity(snapshot):
    m = snapshot.map
    return [
        getattr(snapshot, 'map_index', None),
        getattr(snapshot, 'map_sequence', getattr(snapshot, 'sequence', None)),
        list(m.origin), m.width, m.height, m.resolution,
    ]


def same_map_frame(a, b) -> bool:
    return a[:1] + a[2:] == b[:1] + b[2:]


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
        dyn = dynamic_obstacles(self.observations, s, m.robot[:2] if m.robot else None)
        b = compute_safe_interior(m, s.labels, self.config, m.robot, dynamic_obstacles=dyn)
        if b.selection_method not in ('robot_safe_cell', 'nearest_safe_cell') or not inside(b, m.robot[:2]):
            raise NavigationRejected('UNLOCALIZED_MAP')
        if not inside(b, self.target):
            raise NavigationRejected('TARGET_OUTSIDE_INTERIOR')
        return b

    def monitor_boundary(self, baseline, boundary, snapshot):
        same_frame = (same_map_frame(map_identity(baseline), map_identity(snapshot))
                      and getattr(snapshot, 'session_id', None) == getattr(baseline, 'session_id', None))
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


@dataclass(frozen=True)
class TowingQualificationConfig:
    towing_qualified: bool = False
    qualified_categories: tuple[str, ...] = ()
    artifact_uri: str = ''


def transport_policy(garment, allow_towing, config=None):
    config = config or TowingQualificationConfig()
    suitability = garment.transport_suitability
    if suitability == 1:
        return None
    if allow_towing:
        if (config.towing_qualified and suitability == 2 and config.artifact_uri
                and garment.category in config.qualified_categories):
            return None if config.artifact_uri.startswith('sim:') else 'TOWING_REQUIRES_SUPERVISED_TRIAL'
        return 'TOWING_UNQUALIFIED_SKIP'
    return 'UNSUPPORTED_GARMENT_SKIP'


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        temporary.write_text(json.dumps(value, allow_nan=False, sort_keys=True))
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def loaded_route(boundary, start, target):
    """A* on 4-connected eroded reachable cells; verify every actual segment."""
    start, target = tuple(start), tuple(target)
    if not inside(boundary, start) or not inside(boundary, target): return None
    seed, end = cell(boundary, start), cell(boundary, target)
    queue = [(0, seed)]; cost = {seed: 0}; parent = {seed: None}
    while queue:
        _, current = heapq.heappop(queue)
        if current == end: break
        x, y = current
        for neighbor in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
            nx, ny = neighbor
            if not (0 <= nx < boundary.width and 0 <= ny < boundary.height and boundary.reachable_mask[ny,nx]): continue
            candidate = cost[current] + 1
            if candidate < cost.get(neighbor, math.inf):
                cost[neighbor] = candidate; parent[neighbor] = current
                heapq.heappush(queue, (candidate + abs(nx-end[0]) + abs(ny-end[1]), neighbor))
    if end not in parent: return None
    cells=[]; current=end
    while current is not None:
        cells.append(current); current=parent[current]
    route=[start]+[(boundary.origin[0]+(x+.5)*boundary.resolution,
                   boundary.origin[1]+(y+.5)*boundary.resolution) for x,y in cells[::-1]]+[target]
    if not all(check_corridor_containment(boundary,a,b,0.) for a,b in zip(route,route[1:])): return None
    return route[1:]


class DeliveryRejected(RuntimeError):
    pass


def track_key(garment) -> str:
    if garment is None:
        return ''
    tid = (garment.get('track_id') or garment.get('id') or '') if isinstance(garment, dict) else getattr(garment, 'track_id', garment)
    if isinstance(tid, dict) and isinstance(tid.get('uuid'), (list, tuple)):
        return bytes(int(b) & 0xff for b in tid['uuid']).hex()
    if hasattr(tid, 'uuid'):
        return bytes(tid.uuid).hex()
    return str(tid)


def graspable_garments(garments, category='any', key=None, world=None) -> list:
    excluded = (
        {e.track_id for e in getattr(world, 'entries', {}).values() if e.track_id and e.relevance in ('completed', 'deferred', 'ignored')}
        if key is None and world is not None else set()
    )
    return [
        g for g in garments
        if (category in ('', 'any') or g.category == category)
        and all(getattr(g, f) for f in ('reachable_for_grasp', 'inside_geofence', 'geofence_verified', 'grasp_valid', 'contact_valid'))
        and (key is None or track_key(g) == key)
        and track_key(g) not in excluded
    ]


def tf_jumped(a, b) -> bool:
    return (math.dist(a[:2], b[:2]) > .15
            or abs(math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2]))) > .15)


class GarmentDeliverer:
    """Execute garment approach, grasp, loaded corridor transit, and release."""
    def __init__(self, observations, executor, *, boundary_config=None, towing_config=None,
                 pose=None, map_odom=None, target_track_id=None, held_check=None, support_check=None, feedback=lambda **_:None,
                 journal=lambda **_:None, reserve_check=lambda:False, clock=time.monotonic):
        self.observations=observations; self.executor=executor
        self.config=replace(boundary_config or BoundaryConfig(), deployed_arm_extension_m=.10)
        self.towing=towing_config or TowingQualificationConfig()
        self.pose=pose; self.map_odom=map_odom; self.baseline_tf=None; self.payload_uncertain=False
        self.held_check=held_check or self._held_evidence
        self.support_check=support_check or self._support_evidence
        self.feedback=feedback; self.journal=journal; self.reserve_check=reserve_check
        self.clock=clock; self.deadline=math.inf; self.latest_observation=None
        self.initial_identity=None; self.target_track_id=target_track_id
        self.position=[0.,0.]; self.track=None; self.frames=[]; self.stowed=False
        self.attempt_id=str(uuid.uuid4()); self.holding=False

    def result(self, code, detail=''):
        value=dict(status_code=code, delivered=code=='DELIVERED',stowed=self.stowed,
            dry_run_valid=code=='DRY_RUN_VALID',final_position_m=list(self.position),
            evidence_uri=self.executor.directory.resolve().as_uri() if self.executor else '',
            detail=(detail or code)[:128])
        if self.track is not None: value['track_id']=self.track.track_id
        return value

    def state(self, stage):
        self.feedback(phase=3,stage=stage,reason='NONE',current_position_m=list(self.position))

    def guard(self):
        if self.clock()>=self.deadline: raise DeliveryRejected('TIMED_OUT')
        if self.executor and self.executor.hazard(): raise DeliveryRejected('BOUNDARY_ABORT')

    def snapshot(self, refresh=True):
        s=self.observations.navigation_snapshot(refresh=refresh,max_age_s=5.)
        if (not s.active_floor_verified or not s.current_pose_verified or not 0 <= s.cache_age_s <= 5.
                or s.map.robot is None): raise DeliveryRejected('UNLOCALIZED_MAP')
        self.latest_observation=s.evidence
        identity=map_identity(s)
        if self.initial_identity is not None and not same_map_frame(self.initial_identity, identity):
            raise DeliveryRejected('MAP_CONTEXT_CHANGED')
        if self.map_odom and self.baseline_tf is not None and tf_jumped(self.map_odom(), self.baseline_tf):
            raise DeliveryRejected('MAP_CONTEXT_CHANGED')
        p=self.pose() if self.pose else s.map.robot
        if p is None or len(p)<3 or not np.isfinite(p[:3]).all(): raise DeliveryRejected('UNLOCALIZED_MAP')
        self.position=list(p[:2]); self.current_pose=p
        return s

    def context(self, refresh=True):
        s=self.snapshot(refresh)
        b=compute_safe_interior(s.map,s.labels,self.config,self.current_pose,dynamic_obstacles=dynamic_obstacles(self.observations,s,self.position))
        if not inside(b,self.position): raise DeliveryRejected('LOADED_ROUTE_BLOCKED')
        return s,b

    def acquire(self, category, after=None, key=None):
        batch=self.observations.garment_snapshot(after=after)
        self.latest_observation=batch.evidence
        wm=self.observations.world_map() if hasattr(self.observations,'world_map') else None
        candidates=graspable_garments(batch.garments, category, key, world=wm)
        return (min(candidates,key=track_key) if candidates else None),batch

    def operation(self, name, fn):
        self.guard()
        command_id=str(uuid.uuid4())
        self.journal(operation=name,status='IN_FLIGHT',command_id=command_id,attempt_id=self.attempt_id)
        result=fn()
        self.journal(operation=name,status='OBSERVED',command_id=command_id,attempt_id=self.attempt_id)
        return result

    def capture(self, stage, after=None):
        frame=self.observations.garment_frame(after=after)
        if any(tuple(f['source_id'])==tuple(frame['source_id']) for f in self.frames):
            raise LookupError('duplicate acquisition cannot verify a new stage')
        frame['frame_id']=str(uuid.uuid4()); frame['attempt_id']=self.attempt_id; frame['stage']=stage
        path=self.executor.directory/f'{self.attempt_id}-{stage}-{frame["frame_id"]}.jpg'
        if not cv2.imwrite(str(path),frame['image']): raise LookupError('evidence write failed')
        self.frames.append(frame)
        self.journal(operation='frame',status='OBSERVED',frame_id=frame['frame_id'],
                     source_id=list(frame['source_id']),attempt_id=self.attempt_id)
        return frame

    def _cloth_mask(self, track, image):
        pre=next((f for f in self.frames if f['stage']=='1-pregrip'),None)
        if pre is None or not hasattr(track,'bbox_px'): return None
        x,y,w,h=map(int,track.bbox_px)
        if min(w,h)<5 or x<0 or y<0 or x+w>image.shape[1] or y+h>image.shape[0]: return None
        lab=cv2.cvtColor(pre['image'],cv2.COLOR_BGR2LAB).astype(float)
        roi=lab[y:y+h,x:x+w]
        core=roi[h//4:max(h//4+1,3*h//4),w//4:max(w//4+1,3*w//4)]
        cloth=np.median(core.reshape(-1,3),axis=0)
        border=np.concatenate((roi[0],roi[-1],roi[:,0],roi[:,-1]))
        ring=lab[max(0,y-5):min(lab.shape[0],y+h+5),max(0,x-5):min(lab.shape[1],x+w+5)]
        if ring.size:
            border=np.concatenate((ring[0],ring[-1],ring[:,0],ring[:,-1]))
        if np.linalg.norm(cloth-np.median(border,axis=0))<25.: return None
        current=cv2.cvtColor(image,cv2.COLOR_BGR2LAB).astype(float)
        return (np.linalg.norm(current-cloth,axis=2)<25.).astype(np.uint8)

    def _elevated_cloth(self,track,frame):
        mask=self._cloth_mask(track,frame['image'])
        if mask is None: return None
        try:
            camera=perception.front_camera()
            if frame['image'].shape[:2]!=(camera.height,camera.width): return None
            low_lift=int(getattr(self,'last_lift_steps',2))==1 and frame.get('stage')!='post-withdrawal'
            try:
                jaw=perception.front_jaw(frame['image'])
                point=np.asarray(jaw['position'])
            except LookupError:
                pre=next((f for f in self.frames if f.get('stage')=='1-pregrip'),None)
                if pre is None or pre['image'] is frame['image']: return None
                steps=1 if low_lift else max(2,int(getattr(self,'last_lift_steps',2)))
                point=np.asarray(perception.front_jaw(pre['image'])['position'])+np.array([0.,0.,steps*.04])
            min_z=.055 if low_lift else .08
            if point[2]<min_z: return None
            min_cloth_z=.045 if low_lift else .06
            offset=perception.calibration()['front_camera_x_m']
            C=np.array([offset,0.,camera.h/1000])
            count,labels,stats,centers=cv2.connectedComponentsWithStats(mask)
            for index in range(1,count):
                if stats[index,cv2.CC_STAT_AREA]<20: continue
                ray=np.asarray(camera.pixel_to_ray(*centers[index]),float)
                if ray[0]<=0: continue
                at_depth=C+ray*((point[0]-C[0])/ray[0])
                if at_depth[2]>min_cloth_z and np.linalg.norm(at_depth-point)<.09: return True
            return False
        except (LookupError,ValueError,KeyError): return None

    def _held_evidence(self,track,frames):
        if len(frames)<3 or not all(s in [f.get('stage') for f in frames] for s in ('1-pregrip','2-lifted','3-verify')): return False
        pre=next(f for f in frames if f['stage']=='1-pregrip')
        last=frames[-1]
        mask=self._cloth_mask(track,last['image'])
        before=self._cloth_mask(track,pre['image'])
        if mask is None or before is None: return False
        x,y,w,h=map(int,track.bbox_px)
        floor_verified=getattr(self,'floor_verified',False)
        if not self.holding:
            original=int(before[y:y+h,x:x+w].sum())
            floor_verified=original>=20 and int(mask[y:y+h,x:x+w].sum())<.4*original
        elevated=self._elevated_cloth(track,last)
        self.floor_verified=bool(floor_verified)
        last.update(floor_spot_reduced=bool(floor_verified),elevated_cloth_near_jaw_depth=elevated is True)
        return bool(floor_verified and elevated is True)

    def _support_evidence(self,track,frame):
        mask=self._cloth_mask(track,frame['image'])
        if mask is None or int(np.count_nonzero(mask))<15 or self._elevated_cloth(track,frame) is not False: return False
        frame.update(detached=True,floor_supported=True)
        return True

    def _yaw_gain(self, angle):
        return yaw_gain(getattr(self.executor, 'gains', None), angle)

    def _forward_gain(self):
        return forward_gain(getattr(self.executor, 'gains', None))

    def monitor(self, baseline, boundary, loaded=True):
        self.guard()
        s=self.snapshot()
        if not same_map_frame(map_identity(baseline), map_identity(s)):
            raise DeliveryRejected('MAP_CONTEXT_CHANGED')
        live=compute_safe_interior(s.map,s.labels,self.config,self.current_pose,dynamic_obstacles=dynamic_obstacles(self.observations,s,self.position))
        if not inside(boundary,self.position) or not inside(live,self.position):
            raise DeliveryRejected('BOUNDARY_ABORT')
        if loaded:
            frame=self.capture('retention')
            if not self.held_check(self.track,[*self.frames[:-1],frame]):
                raise DeliveryRejected('GARMENT_SLIPPED_DURING_TRANSPORT')
            if self.reserve_check(): raise DeliveryRejected('SAFE_RELEASE_BEFORE_DOCK')
        return s,live

    def transit(self, target, baseline, boundary, loaded=True):
        max_pulse=max(80.,float(getattr(self.executor,'max_transit_pulse_ms',500.)))
        for _ in range(600):
            s,b=self.monitor(baseline,boundary,loaded)
            route=loaded_route(b,self.position,target)
            if route is None: raise DeliveryRejected('LOADED_ROUTE_BLOCKED')
            if math.dist(self.position,target)<=.09: return
            waypoint=next((p for p in route if math.dist(self.position,p)>.035),target)
            x,y,yaw=self.current_pose[:3]
            angle=math.atan2(waypoint[1]-y,waypoint[0]-x)-yaw
            angle=math.atan2(math.sin(angle),math.cos(angle))
            if abs(angle)>.08:
                gain=self._yaw_gain(angle)
                omega=math.copysign(.2,angle); duration=min(max_pulse,max(80,abs(angle)/(abs(omega)*gain)*1000))
                self.operation('bounded_turn',lambda:self.executor.pulse(omega=omega,duration=duration))
            else:
                gain=self._forward_gain()
                straight_wp=waypoint
                if max_pulse>500.:
                    for p in route:
                        if abs(math.atan2(math.sin(math.atan2(p[1]-y,p[0]-x)-yaw),math.cos(math.atan2(p[1]-y,p[0]-x)-yaw)))<=.08 and check_corridor_containment(b,(x,y),p,0.):
                            straight_wp=p
                        else:
                            break
                duration=min(max_pulse,max(80,math.dist(self.position,straight_wp)/(.05*gain)*1000))
                step=.05*gain*duration/1000
                end=(x+step*math.cos(yaw),y+step*math.sin(yaw))
                if not check_corridor_containment(b,(x,y),end,0.): raise DeliveryRejected('LOADED_ROUTE_BLOCKED')
                self.operation('bounded_forward',lambda:self.executor.pulse(velocity=.05,duration=duration))
        raise DeliveryRejected('TRANSIT_DID_NOT_CONVERGE')

    def approach(self, category):
        barrier=self.observations.garment_barrier()
        for _ in range(40):
            self.guard()
            s,b=self.context()
            self.track,_=self.acquire(category,after=barrier,key=track_key(self.track))
            if self.track is None: raise DeliveryRejected('TARGET_LOST_AFTER_DEPLOY')
            p=self.track.grasp_point_m
            plan=planner.approach([p.x,p.y,p.z])
            if abs(plan['yaw'])<.035 and abs(plan['forward'])<.015: return
            angle=plan['yaw']; v=0. if abs(angle)>.035 else math.copysign(.05,plan['forward'])
            omega=math.copysign(.2,angle) if not v else 0.
            f_gain=self._forward_gain()
            if omega:
                duration=min(500,max(80,abs(angle)/(abs(omega)*self._yaw_gain(angle))*1000))
            else:
                duration=min(500,max(80,abs(plan['forward'])/(abs(v)*f_gain)*1000))
            x,y,yaw=self.current_pose[:3]
            step=v*f_gain*duration/1000
            end=(x+step*math.cos(yaw),y+step*math.sin(yaw))
            if not check_corridor_containment(b,(x,y),end,0.): raise DeliveryRejected('LOADED_ROUTE_BLOCKED')
            self.operation('leading_edge_approach',lambda:self.executor.pulse(velocity=v,omega=omega,duration=duration))
            barrier=self.observations.garment_barrier()
        raise DeliveryRejected('APPROACH_DID_NOT_CONVERGE')

    def release_withdraw(self):
        self.state('RELEASE_WITHDRAW_VERIFY')
        self.operation('manual_in_place_release',self.executor.open_jaws)
        self.operation('withdraw_hand',lambda:self.executor.command('app_arm_direction_move',{'direction':1}))
        barrier=self.observations.garment_barrier()
        s,b=self.context(); x,y,yaw=self.current_pose[:3]
        target=(x-.15*math.cos(yaw),y-.15*math.sin(yaw))
        if not check_corridor_containment(b,(x,y),target,0.): raise DeliveryRejected('WITHDRAWAL_ROUTE_BLOCKED')
        for _ in range(16):
            self.guard(); self.monitor(s,b,loaded=False)
            if math.dist(self.position,(x,y))>=.14: break
            p=self.position; q=(p[0]-.025*math.cos(yaw),p[1]-.025*math.sin(yaw))
            if not check_corridor_containment(b,p,q,0.): raise DeliveryRejected('WITHDRAWAL_ROUTE_BLOCKED')
            self.operation('withdraw_base',lambda:self.executor.pulse(velocity=-.05,duration=500))
        else: raise DeliveryRejected('WITHDRAWAL_UNVERIFIED')
        barrier=self.observations.garment_barrier()
        frame=self.capture('post-withdrawal',after=barrier)
        if not self.support_check(self.track,frame): raise DeliveryRejected('RELEASE_UNVERIFIED')
        self.holding=False; self.payload_uncertain=False; self.executor.gripped=False
        self.state('STOW_AND_VERIFY')
        result=self.operation('stow',self.executor.stow)
        self.stowed=bool(result.get('stowed'))
        if not self.stowed: raise DeliveryRejected('STOW_UNVERIFIED')

    def execute(self, goal):
        if (not all(math.isfinite(v) and -100<=v<=100 for v in (goal.drop_x_m,goal.drop_y_m))
                or not math.isfinite(goal.max_duration_s) or not 1<=goal.max_duration_s<=3600
                or type(goal.dry_run) is not bool or type(goal.allow_towing) is not bool
                or not isinstance(goal.category,str) or len(goal.category)>32): return self.result('INVALID_INPUT')
        self.deadline=self.clock()+goal.max_duration_s
        try:
            self.state('LOCALIZE_AND_VERIFY_CONTEXT')
            baseline,b=self.context(refresh=not goal.dry_run)
            self.baseline_tf=self.map_odom() if self.map_odom else None
            self.initial_identity=map_identity(baseline)
            target=(goal.drop_x_m,goal.drop_y_m)
            if loaded_route(b,self.position,target) is None: return self.result('LOADED_ROUTE_BLOCKED')
            self.track,batch=self.acquire(goal.category,key=self.target_track_id)
            if self.track is None: return self.result('NO_CONFIRMED_GARMENT')
            rejection=transport_policy(self.track,goal.allow_towing,self.towing)
            if rejection: return self.result(rejection)
            if goal.dry_run: return self.result('DRY_RUN_VALID')
            self.guard()
            cx,cy=cell(b,self.position)
            if b.clearance_m[cy,cx]<.8: return self.result('NO_OPEN_DEPLOYMENT_SPACE')
            self.state('DEPLOY_AND_REACQUIRE')
            self.operation('deploy',lambda:self.executor.deploy(settle=True))
            self.executor.last_motion=None
            barrier=self.observations.garment_barrier()
            key=track_key(self.track)
            self.track,_=self.acquire(goal.category,after=barrier,key=key)
            if self.track is None: raise DeliveryRejected('TARGET_LOST_AFTER_DEPLOY')
            self.approach(goal.category)
            self.state('GRASP_AND_VERIFY')
            self.operation('open_jaws',self.executor.open_jaws)
            p=self.track.grasp_point_m
            estimate=read_arm(self.executor.client)
            j=estimate.get('joints',{})
            joints=[j['m3'],j['m4']] if all(k in j for k in ('m3','m4')) else []
            if len(joints)<2 and hasattr(self.executor,'current_arm_joints'):
                try: joints=list(self.executor.current_arm_joints())[:2]
                except LookupError: pass
            if len(joints)<2: raise DeliveryRejected('ARM_POSE_UNVERIFIED')
            plan=planner.grasp_goal([p.x,p.y],joints[:2],size_m=.03)
            self.operation('calibrated_descent',lambda:self.executor.move_arm(plan['state']))
            self.capture('1-pregrip',after=self.observations.garment_barrier())
            self.payload_uncertain=True
            self.operation('close_jaws',lambda:self.executor.command('app_grip',{'flag':0}))
            lift_steps=1 if getattr(self.track,'transport_suitability',1)==2 else 2
            self.last_lift_steps=lift_steps
            for _ in range(lift_steps):
                self.operation('lift',lambda:self.executor.command('app_arm_direction_move',{'direction':1}))
            self.capture('2-lifted',after=self.observations.garment_barrier())
            self.capture('3-verify',after=self.observations.garment_barrier())
            if not self.held_check(self.track,self.frames):
                self.operation('miss_open_jaws',self.executor.open_jaws)
                self.payload_uncertain=False
                raise DeliveryRejected('GRASP_MISS')
            self.holding=True; self.payload_uncertain=False; self.executor.gripped=True
            self.state('QUALIFIED_LOADED_TRANSPORT')
            self.transit(target,baseline,b)
            self.release_withdraw()
            return self.result('DELIVERED')
        except DeliveryRejected as exc:
            code=str(exc)
            if self.executor:
                self.executor.end_rc()
                if code=='SAFE_RELEASE_BEFORE_DOCK':
                    try: self.release_withdraw()
                    except (DeliveryRejected,LookupError,TimeoutError): return self.result('SAFE_RELEASE_UNVERIFIED')
                elif code in ('GRASP_MISS','TARGET_LOST_AFTER_DEPLOY','GARMENT_SLIPPED_DURING_TRANSPORT'):
                    self.holding=False; self.executor.gripped=False
                    self.stowed=bool(self.executor.stow().get('stowed'))
            return self.result(code)
        except (LookupError,TimeoutError,ValueError,InterruptedError) as exc:
            return self.result('OBSERVATION_GAP',str(exc))
        finally:
            if self.executor:
                try: self.executor.end_rc()
                except (LookupError,TimeoutError,InterruptedError): pass


class PatrolLedger:
    MAX_ATTEMPTS_PER_ITEM = 2
    WAYPOINT_STATES = {'UNVISITED','VISITED','ADEQUATELY_OBSERVED','OCCLUDED_DEFERRED','CLEARED','UNREACHABLE'}
    ITEM_STATES = {'PENDING','IN_FLIGHT','IN_FLIGHT_UNKNOWN','DELIVERED','SKIPPED_UNSUPPORTED','FAILED'}

    def __init__(self, directory):
        self.path=Path(directory)/'patrol_ledger.json'
        self.data: dict=json.loads(self.path.read_text()) if self.path.exists() else dict(
            version=1,map_identity=None,allowed_segments=[],waypoints={},items={},actions=[],lost_rooms=[],state='LOCALIZE_AND_VERIFY_CONTEXT')
        if self.data.get('version')!=1: raise ValueError('unsupported patrol ledger version')
        for collection in ('items','actions'):
            values=self.data[collection].values() if collection=='items' else self.data[collection]
            for entry in values:
                if entry['status']=='IN_FLIGHT': entry['status']='IN_FLIGHT_UNKNOWN'
        self.save()

    def save(self): atomic_json(self.path,self.data)

    @property
    def requires_reconciliation(self):
        return any(item['status']=='IN_FLIGHT_UNKNOWN' for item in self.data['items'].values()) or any(
            a['status']=='IN_FLIGHT_UNKNOWN' for a in self.data['actions'])

    def reconcile(self, observations):
        for key,item in self.data['items'].items():
            if item['status']=='IN_FLIGHT_UNKNOWN' and observations.get(key)=='PRESENT_ON_FLOOR':
                item['status']='PENDING' if item['attempts']<self.MAX_ATTEMPTS_PER_ITEM else 'FAILED'
        for action in self.data['actions']:
            key=action.get('item_key')
            item_reconciled=key in observations and observations[key]=='PRESENT_ON_FLOOR'
            robot_reconciled=observations.get('__robot__')=='STOWED_LOCALIZED_STATIONARY'
            if action['status']=='IN_FLIGHT_UNKNOWN' and (item_reconciled or (not key and robot_reconciled)):
                action['status']='RECONCILED_NO_REPLAY'
        self.save()

    def bind_map(self, identity, allowed_segments, tf=None):
        previous=self.data['map_identity']
        changed=previous is not None and not same_map_frame(previous, identity)
        previous_tf=self.data.get('map_odom')
        jumped=tf is not None and previous_tf is not None and tf_jumped(tf, previous_tf)
        if not self.data['allowed_segments']: self.data['allowed_segments']=list(allowed_segments)
        else: self.data['allowed_segments']=[s for s in self.data['allowed_segments'] if s in allowed_segments]
        if changed or jumped:
            for w in self.data['waypoints'].values(): w.update(status='UNVISITED',views=[])
            self.data['route']=[]; self.data['state']='LOCALIZE_AND_VERIFY_CONTEXT'
        self.data['map_identity']=identity
        if tf is not None: self.data['map_odom']=list(tf)
        self.save()
        return changed or jumped

    def waypoints(self, points):
        active=set()
        for point in points:
            key=f'{point["x"]:.3f},{point["y"]:.3f}'; active.add(key)
            self.data['waypoints'].setdefault(key,dict(**point,status='UNVISITED',views=[]))
            if self.data['waypoints'][key]['status']=='UNREACHABLE': self.data['waypoints'][key]['status']='UNVISITED'
        for key,point in self.data['waypoints'].items():
            if key not in active: point['status']='UNREACHABLE'
        self.save()

    def observe(self,key,*,heading,frame_id,valid,empty,occluded=False):
        w=self.data['waypoints'][key]
        if not valid or occluded:
            w['status']='OCCLUDED_DEFERRED'; self.save(); return
        if frame_id not in {v['frame_id'] for v in w['views']}:
            w['views'].append(dict(heading=heading,frame_id=frame_id,empty=bool(empty)))
        views=w['views']
        diverse=any(abs(math.atan2(math.sin(a['heading']-b['heading']),math.cos(a['heading']-b['heading'])))>=.7
                    for a in views for b in views if a['frame_id']!=b['frame_id'] and a['empty'] and b['empty'])
        w['status']='CLEARED' if diverse else 'ADEQUATELY_OBSERVED' if len(views)>1 else 'VISITED'
        self.save()

    def may_attempt(self,key):
        i=self.data['items'].get(key)
        return i is None or (i['status'] in ('PENDING','FAILED') and i['attempts']<self.MAX_ATTEMPTS_PER_ITEM)

    def begin_item(self,key,position):
        if not self.may_attempt(key): raise ValueError('item replay or attempt budget exhausted')
        item=self.data['items'].setdefault(key,dict(position=list(position),attempts=0,status='PENDING',command_ids=[],frame_ids=[]))
        item['attempts']+=1; item['status']='IN_FLIGHT'; self.data['active_item']=key
        self.save()

    def skip_item(self,key,position,detail):
        self.data['items'][key]=dict(position=list(position),attempts=0,status='SKIPPED_UNSUPPORTED',
                                    command_ids=[],frame_ids=[],detail=detail)
        self.save()

    def finish_item(self,key,status,detail=''):
        if status not in self.ITEM_STATES: raise ValueError('invalid item state')
        self.data['items'][key].update(status=status,detail=detail)
        self.data.pop('active_item',None); self.save()

    def journal(self,**event):
        if self.data.get('active_item'): event['item_key']=self.data['active_item']
        existing=next((a for a in reversed(self.data['actions']) if event.get('command_id') and a.get('command_id')==event['command_id']),None)
        if existing is not None: existing.update(event)
        else: self.data['actions'].append(dict(event))
        key=self.data.get('active_item')
        if key:
            item=self.data['items'][key]
            for field,destination in (('command_id','command_ids'),('frame_id','frame_ids')):
                if event.get(field) and event[field] not in item[destination]: item[destination].append(event[field])
        self.save()

    def summary(self):
        w=list(self.data['waypoints'].values()); items=list(self.data['items'].values())
        return dict(waypoints_visited=sum(x['status'] not in ('UNVISITED','UNREACHABLE') for x in w),
            waypoints_cleared=sum(x['status']=='CLEARED' for x in w),
            waypoints_deferred=sum(x['status']=='OCCLUDED_DEFERRED' for x in w),
            items_attempted=sum(x['attempts'] for x in items),items_delivered=sum(x['status']=='DELIVERED' for x in items),
            items_skipped=sum(x['status']=='SKIPPED_UNSUPPORTED' for x in items),
            items_failed=sum(x['status'] in ('FAILED','IN_FLIGHT_UNKNOWN') for x in items))

    def completion(self):
        totals=self.summary()
        cleared=bool(self.data['waypoints']) and all(w['status']=='CLEARED' for w in self.data['waypoints'].values())
        complete=cleared and not self.data['lost_rooms'] and not self.requires_reconciliation and all(i['status']=='DELIVERED' for i in self.data['items'].values())
        return dict(**totals,completed=complete,partial_completion=not complete,dry_run_valid=False,
            status_code='PATROL_CLEARED' if complete else 'PARTIAL_COMPLETION_SMALL_ITEMS_ONLY',
            ledger_uri=self.path.resolve().as_uri(),detail='All reachable coverage cleared' if complete else 'Unresolved rooms, views, or garments remain')


class PatrolCollector:
    STATES=('LOCALIZE_AND_VERIFY_CONTEXT','SELECT_REACHABLE_WAYPOINT','UNLOADED_TRANSIT','OBSERVE',
        'SELECT_SUPPORTED_GARMENT','DEPLOY_AND_REACQUIRE','GRASP_AND_VERIFY','QUALIFIED_LOADED_TRANSPORT',
        'RELEASE_WITHDRAW_VERIFY','STOW_AND_VERIFY','CONTINUE')

    def __init__(self,observations,executor,ledger,*,boundary_config=None,towing_config=None,
                 pose=None,map_odom=None,feedback=lambda **_:None,delivery_factory=GarmentDeliverer,
                 navigator_factory=None,reserve_for_loaded_transit_and_dock=25.,resume_battery_pct=80.,
                 sleep=time.sleep,clock=time.monotonic,world=None):
        self.observations=observations; self.executor=executor; self.ledger=ledger
        self.config=boundary_config or BoundaryConfig(); self.towing=towing_config or TowingQualificationConfig()
        self.pose=pose; self.map_odom=map_odom; self.feedback=feedback
        self.delivery_factory=delivery_factory; self.navigator_factory=navigator_factory
        self.reserve=reserve_for_loaded_transit_and_dock; self.resume=resume_battery_pct
        self.sleep=sleep; self.clock=clock; self.deadline=math.inf; self.latest_observation=None; self.position=[0.,0.]
        self.world = world or (observations.world_map() if hasattr(observations, 'world_map') else SessionWorldMap())

    def state(self,state):
        self.ledger.data['state']=state; self.ledger.save()
        totals=self.ledger.summary()
        self.feedback(phase=3,state=state,reason='NONE',waypoints_visited=totals['waypoints_visited'],
            items_delivered=totals['items_delivered'],items_skipped=totals['items_skipped'],current_position_m=list(self.position))

    def guard(self):
        if self.clock()>=self.deadline: raise DeliveryRejected('TIMED_OUT')
        if self.executor and self.executor.hazard(): raise DeliveryRejected('BOUNDARY_ABORT')

    def _status(self):
        return read_status(self.observations)

    def _arm(self):
        return read_arm(self.observations)

    def battery(self):
        return self._status()['battery']

    def _pose(self, snapshot=None, max_age_s=None):
        if self.pose:
            return self.pose()
        snap = snapshot or self.observations.navigation_snapshot(refresh=False, max_age_s=max_age_s)
        return snap.map.robot

    def _record_world(self, action: str, **fields):
        fn = getattr(self.observations, 'record_world_event', None)
        if callable(fn):
            fn(action, **fields)
        elif self.world is not None:
            self.world.apply_event(dict(action=action, **fields))

    def verify_context(self,refresh=True):
        self.guard(); self.state('LOCALIZE_AND_VERIFY_CONTEXT')
        s=self.observations.navigation_snapshot(refresh=refresh,max_age_s=5.)
        if not s.active_floor_verified or not s.current_pose_verified or not 0<=s.cache_age_s<=5.: raise DeliveryRejected('UNLOCALIZED_MAP')
        pose=self._pose(s)
        if pose is None: raise DeliveryRejected('UNLOCALIZED_MAP')
        self.position=list(pose[:2]); self.latest_observation=s.evidence
        tf=self.map_odom() if self.map_odom else None
        self.ledger.bind_map(map_identity(s),self.config.allowed_segments,tf)
        if self.world is not None:
            self.world.bind_session(
                getattr(s, 'session_id', b'\x01' * 16),
                s.map_index, s.map.origin, s.map.width, s.map.height, s.map.resolution,
            )
        self.config=replace(self.config,allowed_segments=tuple(self.ledger.data['allowed_segments']))
        b=compute_safe_interior(s.map,s.labels,self.config,pose,dynamic_obstacles=dynamic_obstacles(self.observations,s,pose[:2],self.world))
        if not inside(b,pose[:2]): raise DeliveryRejected('UNLOCALIZED_MAP')
        self.ledger.data['lost_rooms']=list(b.lost_rooms)
        self.ledger.waypoints(b.waypoints)
        return s,b

    def recharge(self,deliverer=None):
        self.guard()
        if deliverer is not None and deliverer.holding:
            self.state('SAFE_RELEASE_BEFORE_DOCK')
            deliverer.release_withdraw()
            deliverer.holding=False
            self.executor.gripped=False
        if self.executor.gripped: raise DeliveryRejected('PAYLOAD_STATE_UNKNOWN')
        self.executor.end_rc()
        self.ledger.journal(operation='dock',status='IN_FLIGHT',command_id=str(uuid.uuid4()))
        self.executor.command('app_charge',[])
        self.state('CHARGING')
        for _ in range(3600):
            self.guard()
            if self.battery()>=self.resume:
                self.verify_context(refresh=True)
                for a in self.ledger.data['actions']:
                    if a['operation']=='dock': a['status']='OBSERVED'
                self.ledger.save(); return
            self.sleep(5.)
        raise DeliveryRejected('RECHARGE_TIMEOUT')

    def observe_waypoint(self,key):
        for heading in range(2):
            self.guard()
            if heading:
                start=self._pose()[2]
                command_id=str(uuid.uuid4())
                self.ledger.journal(operation='observe_turn',status='IN_FLIGHT',command_id=command_id)
                for _ in range(24):
                    self.guard()
                    self.verify_context(refresh=True)
                    yaw=self._pose()[2]
                    if abs(math.atan2(math.sin(yaw-start),math.cos(yaw-start)))>=.8: break
                    self.executor.pulse(omega=.35,duration=600)
                else:
                    self.ledger.data['waypoints'].setdefault(key,dict(status='OCCLUDED_DEFERRED',views=[]))['status']='OCCLUDED_DEFERRED'
                    self.ledger.save(); return batch
                self.executor.end_rc()
                self.ledger.journal(operation='observe_turn',status='OBSERVED',command_id=command_id)
            barrier=self.observations.garment_barrier()
            batch=self.observations.garment_snapshot(after=barrier)
            pose=self._pose()
            if self.world is not None and pose is not None:
                self.world.ingest_observations(batch, pose)
            valid=batch.model_available and batch.geofence_active
            source=bytes(batch.evidence.session_id.uuid).hex()+':'+str(batch.evidence.sequence)
            frame=self.observations.garment_frame(after=barrier)
            empty=not batch.garments
            self.ledger.data['waypoints'].setdefault(key,dict(status='UNVISITED',views=[]))
            self.ledger.observe(key,heading=pose[2],frame_id=source,valid=valid,
                                empty=empty,occluded=not frame.get('coverage_valid',False))
            if batch.garments: return batch
        return batch

    def _waypoint_priority(self, key: str, allow_towing: bool = True) -> tuple[int, float]:
        wp = self.ledger.data['waypoints'][key]
        wx, wy = float(wp['x']), float(wp['y'])
        dist = math.dist(self.position, (wx, wy))
        if self.world is not None:
            allowed_mob = ('pickupable', 'draggable') if allow_towing else ('pickupable',)
            for obj in self.world.pending_objectives(tuple(self.position), allowed_mobility=allowed_mob):
                nearest_key = min(
                    self.ledger.data['waypoints'].keys(),
                    key=lambda k: math.hypot(float(self.ledger.data['waypoints'][k]['x']) - obj.x,
                                             float(self.ledger.data['waypoints'][k]['y']) - obj.y),
                )
                if key == nearest_key or math.hypot(obj.x - wx, obj.y - wy) <= 0.65:
                    return (0, math.hypot(obj.x - self.position[0], obj.y - self.position[1]) + 0.1 * math.hypot(obj.x - wx, obj.y - wy))
        return (1, dist)

    def execute(self,goal):
        if (not math.isfinite(goal.max_duration_s) or not 1<=goal.max_duration_s<=3600
                or not all(math.isfinite(v) and -100<=v<=100 for v in (goal.drop_x_m,goal.drop_y_m))
                or not 1<=goal.max_items<=1000):
            return dict(self.ledger.completion(),status_code='INVALID_INPUT')
        self.deadline=self.clock()+goal.max_duration_s
        attempted=0
        try:
            snapshot,b=self.verify_context(refresh=not goal.dry_run)
            pose=self._pose(snapshot)
            loaded_boundary=compute_safe_interior(snapshot.map,snapshot.labels,
                replace(self.config,deployed_arm_extension_m=.10),pose,dynamic_obstacles=dynamic_obstacles(self.observations,snapshot,pose[:2],self.world))
            if loaded_route(loaded_boundary,pose[:2],(goal.drop_x_m,goal.drop_y_m)) is None:
                return dict(self.ledger.completion(),status_code='LOADED_ROUTE_BLOCKED',detail='No safe deployed-arm route to staging drop')
            if goal.dry_run:
                return dict(self.ledger.completion(),status_code='DRY_RUN_VALID',dry_run_valid=True,completed=False,partial_completion=False)
            if self.ledger.requires_reconciliation:
                batch=self.observations.garment_snapshot(after=self.observations.garment_barrier())
                reconciliation={track_key(g):'PRESENT_ON_FLOOR' for g in batch.garments if g.contact_valid and g.geofence_verified}
                status=self._status()
                arm=self._arm()
                if (arm.get('state')=='IN_ROBOT' and status.get('in_returning') is False
                        and status.get('in_cleaning') is False and status.get('state') in (3,8)):
                    reconciliation['__robot__']='STOWED_LOCALIZED_STATIONARY'
                self.ledger.reconcile(reconciliation)
                if self.ledger.requires_reconciliation: return dict(self.ledger.completion(),detail='Fresh observation required; no in-flight command replay')
            visited_keys: set[str] = set()
            while True:
                self.guard()
                snapshot,b=self.verify_context()
                candidates = [k for k, w in self.ledger.data['waypoints'].items()
                              if k not in visited_keys and w['status'] not in ('CLEARED', 'UNREACHABLE')]
                if not candidates:
                    break
                key = min(candidates, key=lambda k: self._waypoint_priority(k, allow_towing=bool(goal.allow_towing)))
                visited_keys.add(key)
                waypoint=self.ledger.data['waypoints'].get(key)
                if waypoint is None or waypoint['status'] in ('CLEARED','UNREACHABLE'): continue
                self.state('SELECT_REACHABLE_WAYPOINT')
                if self.battery()<self.reserve:
                    self.recharge()
                    snapshot,b=self.verify_context()
                self.state('UNLOADED_TRANSIT')
                commands=getattr(getattr(self.executor,'client',None),'commands',None)
                if self.navigator_factory is not None or commands is not None:
                    nav_cls=self.navigator_factory or PinAndGoNavigator
                    nav=nav_cls(self.observations,commands,boundary_config=self.config,
                        hazard=self.executor.hazard if self.executor else lambda:None,
                        odom=getattr(self.executor,'odom',lambda:None),
                        clock=self.clock,sleep=self.sleep)
                    command_id=str(uuid.uuid4())
                    self.ledger.journal(operation='pin_and_go',status='IN_FLIGHT',command_id=command_id)
                    nav_res=nav.execute(SimpleNamespace(target_x_m=float(waypoint['x']),target_y_m=float(waypoint['y']),
                        max_duration_s=min(600.,max(1.,self.deadline-self.clock())),dry_run=False))
                    self.ledger.journal(operation='pin_and_go',status='OBSERVED',command_id=command_id,status_code=nav_res.get('status_code'))
                    if not nav_res.get('arrived'):
                        if nav_res.get('status_code') in ('BOUNDARY_ABORT','TIMED_OUT','CANCELLED'):
                            raise DeliveryRejected(nav_res['status_code'])
                        waypoint['status']='UNREACHABLE'; self.ledger.save(); continue
                    self.position=list(nav_res.get('final_position_m',getattr(nav,'position',self.position)))
                else:
                    mover=self.delivery_factory(self.observations,self.executor,boundary_config=self.config,
                        pose=self.pose,map_odom=self.map_odom,journal=self.ledger.journal)
                    mover.deadline=self.deadline
                    mover.config=self.config
                    mover.baseline_tf=self.map_odom() if self.map_odom else None
                    try: mover.transit((waypoint['x'],waypoint['y']),snapshot,b,loaded=False)
                    except DeliveryRejected:
                        waypoint['status']='UNREACHABLE'; self.ledger.save(); continue
                    self.position=list(mover.position)
                self.state('OBSERVE')
                batch=self.observe_waypoint(key)
                self.state('SELECT_SUPPORTED_GARMENT')
                for g in sorted(batch.garments,key=track_key):
                    self.guard()
                    if not (g.contact_valid and g.inside_geofence and g.geofence_verified): continue
                    item_key=track_key(g)
                    if not self.ledger.may_attempt(item_key): continue
                    if attempted>=goal.max_items: return self.ledger.completion()
                    p=g.floor_contact_m
                    pose=self._pose(max_age_s=5.)
                    location=transform_xy((p.x,p.y),pose).tolist()
                    if any(v['status']=='DELIVERED' for v in self.ledger.data['items'].values()) and math.dist(location,(goal.drop_x_m,goal.drop_y_m))<=.45:
                        continue
                    aliases=[k for k,v in self.ledger.data['items'].items() if math.dist(v['position'],location)<.15]
                    if any(not self.ledger.may_attempt(k) for k in aliases): continue
                    rejection=transport_policy(g,goal.allow_towing,self.towing)
                    mob=mobility_from_suitability(int(getattr(g,'transport_suitability',1)))
                    if rejection:
                        self.ledger.skip_item(item_key,location,rejection)
                        rad = float(max(0.10, 0.5 * max(getattr(g, 'footprint_width_m', 0.16), getattr(g, 'footprint_depth_m', 0.16))))
                        self._record_world('upsert', x=location[0], y=location[1], radius_m=rad, category=g.category, mobility=mob, relevance='deferred', detail=rejection, track_id=item_key)
                        continue
                    if not (g.grasp_valid and g.reachable_for_grasp): continue
                    attempted+=1
                    self.ledger.begin_item(item_key,location)
                    delivery=self.delivery_factory(self.observations,self.executor,boundary_config=self.config,towing_config=self.towing,
                        pose=self.pose,map_odom=self.map_odom,target_track_id=item_key,journal=self.ledger.journal,reserve_check=lambda:self.battery()<self.reserve,
                        feedback=lambda **f:self.state(f['stage']))
                    result=delivery.execute(SimpleNamespace(category=g.category,drop_x_m=goal.drop_x_m,drop_y_m=goal.drop_y_m,
                        max_duration_s=max(1.,self.deadline-self.clock()),allow_towing=goal.allow_towing,dry_run=False))
                    uncertain=delivery.holding or getattr(delivery,'payload_uncertain',False)
                    status='DELIVERED' if result['delivered'] else 'IN_FLIGHT_UNKNOWN' if uncertain else 'FAILED'
                    self.ledger.finish_item(item_key,status,result['status_code'])
                    if status == 'DELIVERED':
                        self._record_world('completed', x=goal.drop_x_m, y=goal.drop_y_m, track_id=item_key, category=g.category, mobility=mob, detail='DELIVERED')
                    elif status == 'FAILED':
                        self._record_world('failed', x=location[0], y=location[1], track_id=item_key, detail=result['status_code'])
                        if not self.ledger.may_attempt(item_key):
                            self._record_world('upsert', x=location[0], y=location[1], radius_m=0.12, category=g.category, mobility=mob, relevance='deferred', detail=result['status_code'], track_id=item_key)
                    if result['status_code']=='SAFE_RELEASE_BEFORE_DOCK':
                        self.recharge(delivery)
                        break
                    elif uncertain: return self.ledger.completion()
                self.state('CONTINUE')
            return self.ledger.completion()
        except (DeliveryRejected,LookupError,TimeoutError,InterruptedError) as exc:
            return dict(self.ledger.completion(),detail=str(exc)[:128])
        finally:
            if self.executor:
                try: self.executor.end_rc()
                except (LookupError,TimeoutError,InterruptedError): pass
