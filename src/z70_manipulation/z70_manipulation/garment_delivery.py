"""Small-cloth delivery and deliberately separate supervised towing qualification.

No native navigation while loaded. Capability artifacts are evidence, not an
implicit enabling switch; the deployed policy defaults to unqualified towing.
"""
from dataclasses import dataclass, replace
import heapq
import json
import math
import os
from pathlib import Path
import time
import uuid

import cv2
import numpy as np
from z70_mapping.boundary import BoundaryConfig, compute_safe_interior, check_corridor_containment
from .navigation import inside, cell


@dataclass(frozen=True)
class TowingQualificationConfig:
    towing_qualified: bool = False
    qualified_categories: tuple[str, ...] = ()
    artifact_uri: str = ''


def transport_policy(garment, allow_towing, config):
    suitability = garment.transport_suitability
    if suitability == 1:
        return None
    if allow_towing and (not config.towing_qualified or suitability != 1):
        # Towing requires both supervised qualification AND a class-specific
        # artifact. The production small-item executor never guesses an envelope.
        if (config.towing_qualified and suitability == 2 and config.artifact_uri
                and garment.category in config.qualified_categories):
            return 'TOWING_REQUIRES_SUPERVISED_TRIAL'
        return 'TOWING_UNQUALIFIED_SKIP'
    return 'UNSUPPORTED_GARMENT_SKIP'


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        with temporary.open('x') as stream:
            json.dump(value, stream, allow_nan=False, sort_keys=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        fd = os.open(str(path.parent), os.O_RDONLY)
        try: os.fsync(fd)
        finally: os.close(fd)
    finally:
        temporary.unlink(missing_ok=True)


class TowingTrialHarness:
    """Record supervised straight/turn/doorway trials; never moves hardware itself."""
    PHASES = ('straight_pull', 'observed_turn', 'doorway_traversal')

    def __init__(self, directory):
        self.path = Path(directory) / 'towing_capability.json'
        self.trials = []

    def record(self, phase, *, passed, swept_envelope, rear_observable, evidence_uri,
               entanglement=False, vo_observable=True):
        if phase not in self.PHASES or type(passed) is not bool:
            raise ValueError('invalid supervised towing trial')
        points = np.asarray(swept_envelope, float)
        if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all() or len(points) < 2:
            raise ValueError('record chassis + deployed arm + trailing garment swept envelope')
        if not evidence_uri: raise ValueError('trial evidence required')
        self.trials.append(dict(trial_id=str(uuid.uuid4()), phase=phase, passed=passed,
            swept_envelope=points.tolist(), envelope_includes='chassis+deployed_arm+trailing_garment',
            rear_observable=bool(rear_observable), vo_observable=bool(vo_observable),
            entanglement=bool(entanglement), evidence_uri=evidence_uri))
        value = self.artifact()
        atomic_json(self.path, value)
        return value

    def run(self, observer):
        """Observer owns supervised motion and returns measured phase evidence."""
        for phase in self.PHASES:
            self.record(phase, **observer(phase))
            if self.artifact()['capability'] == 'UNSUPPORTED': break
        return self.artifact()

    def artifact(self):
        unsupported = any(not t['passed'] or t['entanglement'] for t in self.trials)
        observed = {t['phase'] for t in self.trials if t['passed'] and t['rear_observable'] and t['vo_observable']}
        capability = 'UNSUPPORTED' if unsupported else 'SUPPORTED' if observed == set(self.PHASES) else 'INCONCLUSIVE'
        return dict(version=1, capability=capability, trials=self.trials,
                    warnings=['Rear travel path is not continuously observable'] if any(not t['rear_observable'] for t in self.trials) else [],
                    production_towing_enabled=False)


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


def track_key(garment):
    return bytes(garment.track_id.uuid).hex()


def map_identity(snapshot):
    m = snapshot.map
    return [snapshot.map_index, snapshot.map_sequence, list(m.origin), m.width, m.height, m.resolution]


class GarmentDeliverer:
    """Bounded ROS-only sequence; injected verification must be observation-backed.

    The default verifier fails closed when elevated-cloth/depth or support evidence
    is absent. A floor-only detector cannot certify a held payload.
    """
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
        if self.initial_identity is not None and self.initial_identity[:1]+self.initial_identity[2:]!=identity[:1]+identity[2:]:
            raise DeliveryRejected('MAP_CONTEXT_CHANGED')
        if self.map_odom and self.baseline_tf is not None:
            tf=self.map_odom(); previous=self.baseline_tf
            if math.dist(tf[:2],previous[:2])>.15 or abs(math.atan2(math.sin(tf[2]-previous[2]),math.cos(tf[2]-previous[2])))>.15:
                raise DeliveryRejected('MAP_CONTEXT_CHANGED')
        p=self.pose() if self.pose else s.map.robot
        if p is None or len(p)<3 or not np.isfinite(p[:3]).all(): raise DeliveryRejected('UNLOCALIZED_MAP')
        self.position=list(p[:2]); self.current_pose=p
        return s

    def context(self, refresh=True):
        s=self.snapshot(refresh)
        b=compute_safe_interior(s.map,s.labels,self.config,self.current_pose)
        if not inside(b,self.position): raise DeliveryRejected('LOADED_ROUTE_BLOCKED')
        return s,b

    def acquire(self, category, after=None, key=None):
        batch=self.observations.garment_snapshot(after=after)
        self.latest_observation=batch.evidence
        candidates=[g for g in batch.garments if (category in ('','any') or g.category==category)
            and all(getattr(g,field) for field in ('reachable_for_grasp','inside_geofence','geofence_verified','grasp_valid','contact_valid'))
            and (key is None or track_key(g)==key)]
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
        """Appearance association, never a mass or semantic qualification.

        Refuse floor-coloured/ambiguous cloth rather than treating detector loss
        as acquisition. The model's box and calibrated jaw depth supply the two
        independent spatial constraints.
        """
        pre=next((f for f in self.frames if f['stage']=='1-pregrip'),None)
        if pre is None or not hasattr(track,'bbox_px'): return None
        x,y,w,h=map(int,track.bbox_px)
        if min(w,h)<5 or x<0 or y<0 or x+w>image.shape[1] or y+h>image.shape[0]: return None
        lab=cv2.cvtColor(pre['image'],cv2.COLOR_BGR2LAB).astype(float)
        roi=lab[y:y+h,x:x+w]
        core=roi[h//4:max(h//4+1,3*h//4),w//4:max(w//4+1,3*w//4)]
        cloth=np.median(core.reshape(-1,3),axis=0)
        border=np.concatenate((roi[0],roi[-1],roi[:,0],roi[:,-1]))
        # A bounding box may be entirely cloth; use a ring outside it as floor.
        ring=lab[max(0,y-5):min(lab.shape[0],y+h+5),max(0,x-5):min(lab.shape[1],x+w+5)]
        if ring.size:
            border=np.concatenate((ring[0],ring[-1],ring[:,0],ring[:,-1]))
        if np.linalg.norm(cloth-np.median(border,axis=0))<25.: return None
        current=cv2.cvtColor(image,cv2.COLOR_BGR2LAB).astype(float)
        return (np.linalg.norm(current-cloth,axis=2)<25.).astype(np.uint8)

    def _elevated_cloth(self,track,frame):
        from . import perception
        mask=self._cloth_mask(track,frame['image'])
        if mask is None: return None
        try:
            jaw=perception.front_jaw(frame['image'])
            point=np.asarray(jaw['position'])
            if point[2]<.08: return None
            camera=perception.front_camera()
            if frame['image'].shape[:2]!=(camera.height,camera.width): return None
            offset=perception.calibration()['front_camera_x_m']
            C=np.array([offset,0.,camera.h/1000])
            count,labels,stats,centers=cv2.connectedComponentsWithStats(mask)
            for index in range(1,count):
                if stats[index,cv2.CC_STAT_AREA]<20: continue
                ray=np.asarray(camera.pixel_to_ray(*centers[index]),float)
                if ray[0]<=0: continue
                at_depth=C+ray*((point[0]-C[0])/ray[0])
                if at_depth[2]>.06 and np.linalg.norm(at_depth-point)<.09: return True
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
        # Floor-spot reduction is established before any loaded base motion;
        # retention subsequently checks the same cloth near current jaw depth.
        floor_verified=getattr(self,'floor_verified',False)
        if not self.holding:
            original=int(before[y:y+h,x:x+w].sum())
            floor_verified=original>=20 and int(mask[y:y+h,x:x+w].sum())<.4*original
        elevated=self._elevated_cloth(track,last)
        self.floor_verified=bool(floor_verified)
        last.update(floor_spot_reduced=bool(floor_verified),elevated_cloth_near_jaw_depth=elevated is True)
        return bool(floor_verified and elevated is True)

    def _support_evidence(self,track,frame):
        from . import perception
        mask=self._cloth_mask(track,frame['image'])
        if mask is None or self._elevated_cloth(track,frame) is not False: return False
        # Fresh floor-contact hypotheses must agree with the observed appearance,
        # not merely the absence of an elevated blob or a release acknowledgement.
        batch=self.observations.garment_snapshot()
        camera=perception.front_camera()
        if frame['image'].shape[:2]!=(camera.height,camera.width): return False
        for g in batch.garments:
            if g.category!=track.category or not (g.contact_valid and g.geofence_verified and g.inside_geofence): continue
            u,v=map(int,g.contact_px)
            if not (2<=u<mask.shape[1]-2 and 2<=v<mask.shape[0]-2): continue
            contact=camera.pixel_to_floor(u,v)
            if contact is None or not 0<contact['forward_mm']<1000: continue
            if np.count_nonzero(mask[v-2:v+3,u-2:u+3])<5: continue
            frame.update(detached=True,floor_supported=True)
            return True
        return False

    def monitor(self, baseline, boundary, loaded=True):
        self.guard()
        s=self.snapshot()
        if self.map_odom and self.baseline_tf is not None:
            tf=self.map_odom(); old_tf=self.baseline_tf
            if (math.dist(tf[:2],old_tf[:2])>.15 or abs(math.atan2(math.sin(tf[2]-old_tf[2]),math.cos(tf[2]-old_tf[2])))>.15):
                raise DeliveryRejected('MAP_CONTEXT_CHANGED')
        # A changing pixel revision is replanned, but frame/geometry changes are
        # never silently accepted while holding a payload.
        old,new=map_identity(baseline),map_identity(s)
        if old[:1]+old[2:] != new[:1]+new[2:]: raise DeliveryRejected('MAP_CONTEXT_CHANGED')
        live=compute_safe_interior(s.map,s.labels,self.config,self.current_pose)
        if not inside(boundary,self.position) or not inside(live,self.position):
            raise DeliveryRejected('BOUNDARY_ABORT')
        if loaded:
            frame=self.capture('retention')
            if not self.held_check(self.track,[*self.frames[:-1],frame]):
                raise DeliveryRejected('GARMENT_SLIPPED_DURING_TRANSPORT')
            if self.reserve_check(): raise DeliveryRejected('SAFE_RELEASE_BEFORE_DOCK')
        return s,live

    def transit(self, target, baseline, boundary, loaded=True):
        # Every pulse is bounded and observed before the next; no predicted
        # position is credited as arrival, including when VO is unavailable.
        for _ in range(600):
            s,b=self.monitor(baseline,boundary,loaded)
            route=loaded_route(b,self.position,target)
            if route is None: raise DeliveryRejected('LOADED_ROUTE_BLOCKED')
            if math.dist(self.position,target)<=.035: return
            waypoint=next((p for p in route if math.dist(self.position,p)>.035),target)
            x,y,yaw=self.current_pose[:3]
            angle=math.atan2(waypoint[1]-y,waypoint[0]-x)-yaw
            angle=math.atan2(math.sin(angle),math.cos(angle))
            if abs(angle)>.08:
                omega=math.copysign(.2,angle); duration=min(500,max(80,abs(angle)/.2*1000))
                self.operation('bounded_turn',lambda:self.executor.pulse(omega=omega,duration=duration))
            else:
                duration=min(500,max(80,math.dist(self.position,waypoint)/.05*1000))
                end=(x+.05*duration/1000*math.cos(yaw),y+.05*duration/1000*math.sin(yaw))
                if not check_corridor_containment(b,(x,y),end,0.): raise DeliveryRejected('LOADED_ROUTE_BLOCKED')
                self.operation('bounded_forward',lambda:self.executor.pulse(velocity=.05,duration=duration))
        raise DeliveryRejected('TRANSIT_DID_NOT_CONVERGE')

    def approach(self, category):
        from . import planner
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
            duration=min(500,max(80,abs(angle)/.2*1000 if omega else abs(plan['forward'])/.05*1000))
            x,y,yaw=self.current_pose[:3]
            end=(x+v*duration/1000*math.cos(yaw),y+v*duration/1000*math.sin(yaw))
            if not check_corridor_containment(b,(x,y),end,0.): raise DeliveryRejected('LOADED_ROUTE_BLOCKED')
            self.operation('leading_edge_approach',lambda:self.executor.pulse(velocity=v,omega=omega,duration=duration))
            barrier=self.observations.garment_barrier()
        raise DeliveryRejected('APPROACH_DID_NOT_CONVERGE')

    def release_withdraw(self):
        self.state('RELEASE_WITHDRAW_VERIFY')
        self.operation('manual_in_place_release',self.executor.open_jaws)
        self.operation('withdraw_hand',lambda:self.executor.command('app_arm_direction_move',{'direction':1}))
        barrier=self.observations.garment_barrier()
        # Back up 0.15m using observed map-frame displacement and safe corridors.
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
            if goal.dry_run: return self.result('DRY_RUN_VALID','TEMPORARY_STAGING_DROP; no commands dispatched')
            self.guard()
            # Firmware deployment scans the full perimeter; do not deploy at the
            # cloth's close-approach position or knowingly trigger vendor backoff.
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
            from . import planner
            p=self.track.grasp_point_m
            estimate=self.executor.client.get('/api/arm/estimate')
            j=estimate.get('joints',{})
            joints=[j['m3'],j['m4']] if all(k in j for k in ('m3','m4')) else []
            if len(joints)<2: raise DeliveryRejected('ARM_POSE_UNVERIFIED')
            plan=planner.grasp_goal([p.x,p.y],joints[:2],size_m=.03)
            self.operation('calibrated_descent',lambda:self.executor.move_arm(plan['state']))
            self.capture('1-pregrip',after=self.observations.garment_barrier())
            self.payload_uncertain=True  # Persisted in-flight grip must not be replayed after a read gap.
            self.operation('close_jaws',lambda:self.executor.command('app_grip',{}))
            for _ in range(2):
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
            return self.result('DELIVERED','TEMPORARY_STAGING_DROP; not washing-machine confirmation')
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
            if self.executor: self.executor.end_rc()
            return self.result('OBSERVATION_GAP',str(exc))
        finally:
            if self.executor: self.executor.end_rc()
