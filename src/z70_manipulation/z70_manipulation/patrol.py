"""Deterministic patrol executive with a durable, conservative mission ledger."""
import math
import json
from pathlib import Path
import time
from types import SimpleNamespace
import uuid
from dataclasses import replace
from .garment_delivery import (GarmentDeliverer, PinAndGoNavigator, TowingQualificationConfig, atomic_json,
    inside, map_identity, same_map_frame, tf_jumped, transport_policy, track_key, loaded_route, DeliveryRejected)
from z70_mapping.boundary import BoundaryConfig, compute_safe_interior
from z70_mapping.dynamic_layer import SessionWorldMap, mobility_from_suitability
from .perception import transform_xy
from .observations import read_arm, read_status


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

    def _dynamic_obstacles(self, snapshot, pose_xy=None):
        dyn = getattr(snapshot, 'dynamic_obstacles', None)
        if dyn is not None:
            return dyn
        if hasattr(self.observations, 'dynamic_obstacles'):
            return self.observations.dynamic_obstacles(pose_xy)
        if self.world is not None and self.world.width > 0 and self.world.height > 0:
            return self.world.obstacle_mask(protect_xy=pose_xy)
        return None

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
                s.map_index,
                s.map.origin,
                s.map.width,
                s.map.height,
                s.map.resolution,
            )
        self.config=replace(self.config,allowed_segments=tuple(self.ledger.data['allowed_segments']))
        b=compute_safe_interior(s.map,s.labels,self.config,pose,dynamic_obstacles=self._dynamic_obstacles(s,pose[:2]))
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
                    key=lambda k: math.hypot(
                        float(self.ledger.data['waypoints'][k]['x']) - obj.x,
                        float(self.ledger.data['waypoints'][k]['y']) - obj.y,
                    ),
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
                replace(self.config,deployed_arm_extension_m=.10),pose,dynamic_obstacles=self._dynamic_obstacles(snapshot,pose[:2]))
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
                candidates = [
                    k for k, w in self.ledger.data['waypoints'].items()
                    if k not in visited_keys and w['status'] not in ('CLEARED', 'UNREACHABLE')
                ]
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
