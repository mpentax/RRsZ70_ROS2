"""Unit and DDS tests for garment delivery, patrol collection, and Pin-and-Go navigation."""
import hashlib
import struct
import threading
import time
from types import SimpleNamespace as NS
import numpy as np
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from unique_identifier_msgs.msg import UUID
from z70_interfaces.action import DeliverGarment, PatrolCollect, PinAndGo
from z70_interfaces.msg import ArmEstimate, EvidenceStamp, GarmentObservation, GarmentObservations, MapSnapshot, RobotState
from z70_interfaces.srv import RawRpc, RefreshMap, Stop, TaskSession
from z70_manipulation import planner
from z70_manipulation.garment_delivery import (
    DeliveryRejected,
    GarmentDeliverer,
    PatrolCollector,
    PatrolLedger,
    PinAndGoNavigator,
    TowingQualificationConfig,
    loaded_route,
    transport_policy,
)
from z70_manipulation.node import LATCHED, PickServer
from z70_mapping.boundary import BoundaryConfig, compute_safe_interior
from z70_mapping.node import VendorMap


def _grid_map(size=30, pose=(1., 1., 0.)):
    return NS(
        width=size,
        height=size,
        resolution=.1,
        origin=(0., 0.),
        pixels=np.full((size, size), 15, np.uint8),
        occupancy=np.zeros((size, size), np.int8),
        rooms=[1],
        zones=[],
        virtual_walls=[],
        robot=pose,
    )


def test_towing_is_not_implicitly_qualified():
    garment = NS(transport_suitability=2)
    assert transport_policy(garment, True, TowingQualificationConfig()) == 'TOWING_UNQUALIFIED_SKIP'
    assert transport_policy(NS(transport_suitability=1), True, TowingQualificationConfig()) is None


def test_loaded_route_is_four_connected_and_respects_deployed_envelope():
    m = _grid_map()
    b = compute_safe_interior(m, {1: 'room'}, BoundaryConfig(allowed_segments=(1,), deployed_arm_extension_m=.1), m.robot)
    route = loaded_route(b, (1., 1.), (2., 2.))
    assert route and route[-1] == (2., 2.)
    assert loaded_route(b, (1., 1.), (.15, .15)) is None


def test_delivery_dry_run_is_read_only_and_loaded_route_blocked():
    m = _grid_map()
    garment=NS(category='sock',transport_suitability=1,reachable_for_grasp=True,
        inside_geofence=True,geofence_verified=True,grasp_valid=True,contact_valid=True,track_id=NS(uuid=list(range(16))))
    obs=NS(navigation_snapshot=lambda **_:NS(map=m,labels={1:'room'},map_index=1,map_sequence=1,
        current_pose_verified=True,active_floor_verified=True,cache_age_s=0.,evidence=None),
        garment_snapshot=lambda **_:NS(garments=[garment],evidence=None))
    delivery=GarmentDeliverer(obs,None,boundary_config=BoundaryConfig(allowed_segments=(1,)))
    goal=NS(category='sock',drop_x_m=2.,drop_y_m=2.,max_duration_s=60.,allow_towing=False,dry_run=True)
    assert delivery.execute(goal)['dry_run_valid']
    goal.drop_x_m=.1
    assert delivery.execute(goal)['status_code']=='LOADED_ROUTE_BLOCKED'


class DeliveryFixture:
    def __init__(self,tmp_path):
        self.pose=[1.,1.,0.]; self.calls=[]; self.sequence=0; self.lost=False
        self.garment=NS(category='sock',transport_suitability=1,reachable_for_grasp=True,
            inside_geofence=True,geofence_verified=True,grasp_valid=True,contact_valid=True,
            grasp_point_m=NS(x=.4,y=0.,z=0.),track_id=NS(uuid=list(range(16))))
        self.map=_grid_map(50, self.pose)
        self.obs=NS(navigation_snapshot=lambda **_:NS(map=self.map,labels={},map_index=1,map_sequence=1,
            current_pose_verified=True,active_floor_verified=True,cache_age_s=0.,evidence=None),
            garment_snapshot=self.batch,garment_barrier=lambda: self.sequence,garment_frame=self.frame)
        self.ex=NS(directory=tmp_path,hazard=lambda:None,deployed=False,gripped=False,
            end_rc=lambda:self.calls.append('stop'),open_jaws=lambda:self.calls.append('open'),
            deploy=self.deploy,stow=self.stow,move_arm=lambda _:self.calls.append('descent'),
            command=lambda m,p:self.calls.append(m),pulse=self.pulse,
            client=NS(get=lambda _:dict(joints=dict(m3=150,m4=45))))
        self.goal=NS(category='sock',drop_x_m=1.,drop_y_m=1.,max_duration_s=60.,allow_towing=False,dry_run=False)
    def batch(self,**kwargs): return NS(garments=[] if self.lost else [self.garment],evidence=None)
    def frame(self,**kwargs):
        self.sequence+=1
        return dict(image=np.zeros((10,10,3),np.uint8),source_id=('s',self.sequence))
    def deploy(self,**kwargs): self.calls.append('deploy'); self.ex.deployed=True
    def stow(self): self.calls.append('stow'); return dict(stowed=True)
    def pulse(self,velocity=0.,omega=0.,duration=500):
        self.pose[0]+=velocity*duration/1000; self.calls.append('pulse')
    def runner(self,monkeypatch,held=True,support=True):
        monkeypatch.setattr(planner,'grasp_goal',lambda *_,**__:dict(state=[150,45]))
        runner=GarmentDeliverer(self.obs,self.ex,boundary_config=BoundaryConfig(allowed_segments=(1,)),
            held_check=lambda *_:held,support_check=lambda *_:support)
        runner.approach=lambda _:None
        return runner


@pytest.mark.parametrize('held,expected',[(False,'GRASP_MISS'),(True,'DELIVERED')])
def test_delivery_verifies_lift_and_release(tmp_path,monkeypatch,held,expected):
    fixture=DeliveryFixture(tmp_path); runner=fixture.runner(monkeypatch,held=held)
    result=runner.execute(fixture.goal)
    assert result['status_code']==expected
    assert result['delivered']==held and result['stowed']
    assert fixture.calls.count('app_grip')==1
    assert fixture.calls.count('app_arm_direction_move')==(3 if held else 2)
    assert [f['stage'] for f in runner.frames[:3]]==['1-pregrip','2-lifted','3-verify']
    assert len({f['frame_id'] for f in runner.frames})==len(runner.frames)


def test_lost_post_deploy_track_stows_without_gripping(tmp_path,monkeypatch):
    fixture=DeliveryFixture(tmp_path); runner=fixture.runner(monkeypatch)
    def deploy(**_): fixture.lost=True
    fixture.ex.deploy=deploy
    assert runner.execute(fixture.goal)['status_code']=='TARGET_LOST_AFTER_DEPLOY'
    assert 'app_grip' not in fixture.calls and 'stow' in fixture.calls


def test_slip_during_transport_stows_never_credits_delivery(tmp_path,monkeypatch):
    fixture=DeliveryFixture(tmp_path); runner=fixture.runner(monkeypatch)
    checks=iter([True,False]); runner.held_check=lambda *_:next(checks)
    result=runner.execute(fixture.goal)
    assert result['status_code']=='GARMENT_SLIPPED_DURING_TRANSPORT'
    assert not result['delivered'] and result['stowed']


def test_post_withdrawal_support_failure_is_not_delivery(tmp_path,monkeypatch):
    fixture=DeliveryFixture(tmp_path); runner=fixture.runner(monkeypatch,support=False)
    result=runner.execute(fixture.goal)
    assert result['status_code']=='RELEASE_UNVERIFIED' and not result['delivered']
    assert 'stow' not in fixture.calls


def test_identical_acquisition_cannot_be_relabelled_as_three_evidence_frames(tmp_path,monkeypatch):
    fixture=DeliveryFixture(tmp_path); runner=fixture.runner(monkeypatch)
    fixture.obs.garment_frame=lambda **_:dict(image=np.zeros((10,10,3),np.uint8),source_id=('s',1))
    assert runner.execute(fixture.goal)['status_code']=='OBSERVATION_GAP'


def test_unqualified_visual_measurement_never_confirms_hold(tmp_path,monkeypatch):
    fixture=DeliveryFixture(tmp_path); runner=fixture.runner(monkeypatch)
    assert not runner._held_evidence(fixture.garment,[dict(image=np.zeros((10,10,3),np.uint8))])


def test_appearance_verification_requires_floor_reduction_and_elevated_cloth(tmp_path,monkeypatch):
    fixture=DeliveryFixture(tmp_path); runner=fixture.runner(monkeypatch)
    fixture.garment.bbox_px=[10,60,20,20]
    before=np.full((100,100,3),150,np.uint8); before[60:80,10:30]=[0,0,240]
    after=np.full_like(before,150); after[20:40,60:80]=[0,0,240]
    runner.frames=[dict(stage='1-pregrip',image=before),dict(stage='2-lifted',image=after),dict(stage='3-verify',image=after)]
    monkeypatch.setattr(runner,'_elevated_cloth',lambda *_:True)
    assert runner._held_evidence(fixture.garment,runner.frames)
    runner.frames[-1]['image']=before
    assert not runner._held_evidence(fixture.garment,runner.frames)


def run_dds_dry_action(tmp_path, action_type, topic, goal, monkeypatch):
    def wait(predicate):
        deadline = time.monotonic() + 8.
        while not predicate():
            assert time.monotonic() < deadline, 'offline DDS deadline'
            time.sleep(.01)
    rclpy.init(domain_id=94)
    server=peer=executor=thread=action=None
    try:
        server=PickServer(parameter_overrides=[Parameter('patrol_ledger_directory',value=str(tmp_path))])
        peer=rclpy.create_node('garment_action_fixture')
        fixture=DeliveryFixture(tmp_path)
        snapshot=fixture.obs.navigation_snapshot()
        snapshot.evidence=EvidenceStamp(valid=True)
        monkeypatch.setattr(server.observations,'navigation_snapshot',lambda **_:snapshot)
        monkeypatch.setattr(server,'map_pose',lambda source='base_footprint':(1.,1.,0.))
        executor=MultiThreadedExecutor(num_threads=4); executor.add_node(server); executor.add_node(peer)
        thread=threading.Thread(target=executor.spin); thread.start()
        pub=peer.create_publisher(GarmentObservations,'/z70/garments/observations',LATCHED)
        batch=GarmentObservations(model_available=True,geofence_active=True)
        batch.evidence.valid=True; batch.evidence.received_at=peer.get_clock().now().to_msg()
        g=GarmentObservation(category='sock',transport_suitability=1,reachable_for_grasp=True,
            inside_geofence=True,geofence_verified=True,grasp_valid=True,contact_valid=True)
        g.track_id.uuid=list(range(16)); batch.garments=[g]
        pub.publish(batch)
        wait(lambda:server.observations.garments is not None)
        action=ActionClient(peer,action_type,topic)
        assert action.wait_for_server(timeout_sec=5.)
        future=action.send_goal_async(goal); wait(future.done)
        handle=future.result(); assert handle.accepted
        future=handle.get_result_async(); wait(future.done)
        result=future.result()
        assert result.status==4 and result.result.status_code=='DRY_RUN_VALID'
        assert result.result.dry_run_valid and not server.lock.locked()
        assert server.client is None
    finally:
        if action: action.destroy()
        if executor: executor.shutdown(timeout_sec=5.)
        if thread: thread.join(5.)
        if server:
            server.observations.close(); server.destroy_node()
        if peer: peer.destroy_node()
        rclpy.try_shutdown()


def test_loaded_monitor_rejects_tf_jump_before_any_pulse(tmp_path,monkeypatch):
    fixture=DeliveryFixture(tmp_path); runner=fixture.runner(monkeypatch)
    baseline,b=runner.context(); runner.baseline_tf=(0.,0.,0.); runner.map_odom=lambda:(.3,0.,0.)
    with pytest.raises(DeliveryRejected,match='MAP_CONTEXT_CHANGED'): runner.monitor(baseline,b,loaded=False)
    assert 'pulse' not in fixture.calls


def test_unknown_grip_timeout_never_replays_grasp_or_credits_payload(tmp_path,monkeypatch):
    fixture=DeliveryFixture(tmp_path); runner=fixture.runner(monkeypatch)
    def command(method,params):
        fixture.calls.append(method)
        if method=='app_grip': raise TimeoutError('dispatch unknown')
    fixture.ex.command=command
    result=runner.execute(fixture.goal)
    assert result['status_code']=='OBSERVATION_GAP' and not result['delivered']
    assert runner.payload_uncertain and fixture.calls.count('app_grip')==1
    assert 'app_arm_direction_move' not in fixture.calls


def test_deliver_garment_dds_action_uses_typed_observations_without_commands(tmp_path,monkeypatch):
    run_dds_dry_action(tmp_path,DeliverGarment,'/z70/deliver_garment',
        DeliverGarment.Goal(category='sock',drop_x_m=2.,drop_y_m=2.,max_duration_s=10.,allow_towing=False,dry_run=True),monkeypatch)


def test_transit_and_approach_use_calibrated_gains(tmp_path,monkeypatch):
    fixture=DeliveryFixture(tmp_path)
    fixture.ex.gains={'forward':{'gain':.5},'yaw_positive':{'gain':.5},'yaw_negative':{'gain':.5}}
    durations=[]
    def pulse(velocity=0.,omega=0.,duration=500):
        durations.append((velocity,omega,duration))
        fixture.pose[0]+=.2
    fixture.ex.pulse=pulse
    runner=GarmentDeliverer(fixture.obs,fixture.ex,boundary_config=BoundaryConfig(allowed_segments=(1,)),
        held_check=lambda *_:True,support_check=lambda *_:True)
    baseline,b=runner.context()
    runner.transit((1.15,1.05),baseline,b,loaded=False)
    assert durations and durations[0][2]==pytest.approx(500.)
    durations.clear()
    calls=iter([dict(yaw=0.,forward=.015),dict(yaw=0.,forward=0.)])
    monkeypatch.setattr(planner,'approach',lambda *_:next(calls))
    runner.track=fixture.garment
    runner.approach('sock')
    assert durations and durations[0][2]==pytest.approx(500.)


def _patrol_fixture(tmp_path, suitability=1, empty=False):
    fixture = DeliveryFixture(tmp_path)
    fixture.garment.transport_suitability = suitability
    fixture.garment.floor_contact_m = NS(x=.4, y=0.)
    evidence = NS(session_id=NS(uuid=[1] * 16), sequence=1)
    fixture.garment.evidence = evidence
    fixture.obs.get = lambda _: dict(telemetry=dict(status=dict(value=dict(battery=90))))
    fixture.obs.garment_frame = lambda **_: dict(coverage_valid=True, valid_empty_view=empty)
    fixture.obs.garment_snapshot = lambda **_: NS(
        garments=[] if empty else [fixture.garment], evidence=evidence, model_available=True, geofence_active=True
    )
    return fixture


def test_restart_never_replays_or_credits_in_flight_delivery(tmp_path):
    ledger=PatrolLedger(tmp_path)
    ledger.begin_item('sock',[1.,2.])
    ledger.journal(operation='release',status='IN_FLIGHT',command_id='cmd')
    restarted=PatrolLedger(tmp_path)
    assert restarted.data['items']['sock']['status']=='IN_FLIGHT_UNKNOWN'
    assert restarted.data['actions'][-1]['status']=='IN_FLIGHT_UNKNOWN'
    assert restarted.requires_reconciliation
    assert restarted.summary()['items_delivered']==0
    assert not restarted.may_attempt('sock')
    restarted.reconcile({'sock':'PRESENT_ON_FLOOR'})
    assert restarted.may_attempt('sock')


def test_attempts_are_bounded_and_single_heading_never_clears(tmp_path):
    ledger=PatrolLedger(tmp_path)
    ledger.waypoints([{'x':1.,'y':2.}])
    key=next(iter(ledger.data['waypoints']))
    ledger.observe(key,heading=0.,frame_id='f1',valid=True,empty=True)
    assert ledger.data['waypoints'][key]['status']!='CLEARED'
    ledger.observe(key,heading=1.57,frame_id='f1',valid=True,empty=True)
    assert ledger.data['waypoints'][key]['status']!='CLEARED'
    ledger.observe(key,heading=1.57,frame_id='f2',valid=True,empty=True)
    assert ledger.data['waypoints'][key]['status']=='CLEARED'
    for _ in range(2):
        ledger.begin_item('sock',[1.,2.]); ledger.finish_item('sock','FAILED')
    assert not ledger.may_attempt('sock')


def test_partial_completion_reports_skipped_deferred_and_lost_rooms(tmp_path):
    ledger=PatrolLedger(tmp_path)
    ledger.waypoints([{'x':1.,'y':2.}])
    key=next(iter(ledger.data['waypoints']))
    ledger.data['waypoints'][key]['status']='CLEARED'
    assert ledger.completion()['completed']
    ledger.begin_item('jeans',[1.,2.]); ledger.finish_item('jeans','SKIPPED_UNSUPPORTED')
    result=ledger.completion()
    assert not result['completed'] and result['partial_completion']
    assert result['status_code']=='PARTIAL_COMPLETION_SMALL_ITEMS_ONLY'
    assert result['items_skipped']==1


def test_map_revision_invalidates_coverage_without_expanding_approval(tmp_path):
    ledger=PatrolLedger(tmp_path)
    ledger.bind_map([1,1,[0.,0.],20,20,.1], [1])
    ledger.waypoints([{'x':1.,'y':2.}])
    key=next(iter(ledger.data['waypoints']))
    ledger.data['waypoints'][key]['status']='CLEARED'
    assert ledger.bind_map([2,2,[0.,0.],20,20,.1], [1,2])
    assert ledger.data['allowed_segments']==[1]
    assert ledger.data['waypoints'][key]['status']=='UNVISITED'


def test_low_battery_releases_payload_before_docking_and_relocalizes(tmp_path):
    events=[]; batteries=iter([20,40,80])
    obs=NS(get=lambda _: {'telemetry':{'status':{'value':{'battery':next(batteries),'in_returning':False}}}})
    ex=NS(gripped=True,hazard=lambda:None,end_rc=lambda:events.append('stop'),command=lambda *args:events.append(args[0]))
    deliverer=NS(holding=True,release_withdraw=lambda:events.append('safe_release'))
    patrol=PatrolCollector(obs,ex,PatrolLedger(tmp_path),sleep=lambda _:None)
    patrol.verify_context=lambda **_:events.append('relocalized')
    patrol.recharge(deliverer)
    assert events.index('safe_release')<events.index('app_charge')<events.index('relocalized')


def test_release_failure_never_docks(tmp_path):
    events=[]
    ex=NS(gripped=True,hazard=lambda:None,end_rc=lambda:None,command=lambda *args:events.append(args[0]))
    patrol=PatrolCollector(None,ex,PatrolLedger(tmp_path))
    def fail(): raise LookupError('support not observed')
    with pytest.raises(LookupError): patrol.recharge(NS(holding=True,release_withdraw=fail))
    assert not events


def test_tf_jump_invalidates_route_and_coverage(tmp_path):
    ledger=PatrolLedger(tmp_path)
    identity=[1,1,[0.,0.],20,20,.1]
    ledger.bind_map(identity,[1],tf=[0.,0.,0.]); ledger.waypoints([{'x':1.,'y':2.}])
    key=next(iter(ledger.data['waypoints']))
    ledger.data['waypoints'][key]['status']='CLEARED'; ledger.data['route']=[[1.,2.]]
    assert ledger.bind_map(identity,[1],tf=[.3,0.,0.])
    assert ledger.data['route']==[] and ledger.data['waypoints'][key]['status']=='UNVISITED'


def test_patrol_collect_dds_dry_action(tmp_path,monkeypatch):
    run_dds_dry_action(tmp_path,PatrolCollect,'/z70/patrol_collect',
        PatrolCollect.Goal(drop_x_m=2.,drop_y_m=2.,max_duration_s=10.,max_items=10,allow_towing=False,dry_run=True),monkeypatch)


@pytest.mark.parametrize('delivery_status,expected', [('GRASP_MISS','FAILED'),('GARMENT_SLIPPED_DURING_TRANSPORT','FAILED'),('DELIVERED','DELIVERED')])
def test_patrol_records_verified_delivery_not_attempt_as_success(tmp_path,delivery_status,expected):
    fixture=_patrol_fixture(tmp_path)
    class Delivery:
        holding=False
        def __init__(self,*_,**__): self.position=[1.,1.]
        def transit(self,*_,**__): pass
        def execute(self,goal): return dict(delivered=delivery_status=='DELIVERED',status_code=delivery_status)
    ledger=PatrolLedger(tmp_path)
    patrol=PatrolCollector(fixture.obs,fixture.ex,ledger,boundary_config=BoundaryConfig(allowed_segments=(1,),waypoint_spacing_m=10),delivery_factory=Delivery)
    goal=NS(drop_x_m=1.,drop_y_m=1.,max_duration_s=60.,max_items=10,allow_towing=False,dry_run=False)
    result=patrol.execute(goal)
    assert ledger.data['items'][bytes(fixture.garment.track_id.uuid).hex()]['status']==expected
    assert result['items_delivered']==(delivery_status=='DELIVERED')
    assert result['partial_completion']


def test_unsupported_towing_is_skipped_without_delivery_dispatch(tmp_path):
    fixture=_patrol_fixture(tmp_path,suitability=2)
    class Transit:
        position=[1.,1.]
        def __init__(self,*_,**__): pass
        def transit(self,*_,**__): pass
        def execute(self,_): raise AssertionError('unqualified towing dispatched')
    ledger=PatrolLedger(tmp_path)
    patrol=PatrolCollector(fixture.obs,fixture.ex,ledger,boundary_config=BoundaryConfig(allowed_segments=(1,),waypoint_spacing_m=10),delivery_factory=Transit)
    result=patrol.execute(NS(drop_x_m=1.,drop_y_m=1.,max_duration_s=60.,max_items=10,allow_towing=True,dry_run=False))
    assert result['items_skipped']==1 and result['items_delivered']==0 and result['partial_completion']
    assert result['items_attempted']==0


def test_unknown_action_without_item_requires_explicit_reconciliation(tmp_path):
    ledger=PatrolLedger(tmp_path)
    ledger.journal(operation='unloaded_move',status='IN_FLIGHT',command_id='move')
    restart=PatrolLedger(tmp_path)
    restart.reconcile({})
    assert restart.requires_reconciliation


def test_same_map_pose_refresh_does_not_erase_cleared_coverage(tmp_path):
    ledger=PatrolLedger(tmp_path)
    ledger.bind_map([1,1,[0.,0.],20,20,.1],[1]); ledger.waypoints([{'x':1.,'y':2.}])
    key=next(iter(ledger.data['waypoints'])); ledger.data['waypoints'][key]['status']='CLEARED'
    ledger.bind_map([1,2,[0.,0.],20,20,.1],[1])
    assert ledger.data['waypoints'][key]['status']=='CLEARED'


def test_patrol_dry_run_validates_loaded_drop_route_without_commands(tmp_path):
    fixture=DeliveryFixture(tmp_path)
    patrol=PatrolCollector(fixture.obs,None,PatrolLedger(tmp_path),boundary_config=BoundaryConfig(allowed_segments=(1,)))
    result=patrol.execute(NS(drop_x_m=.05,drop_y_m=.05,max_duration_s=10.,max_items=10,allow_towing=False,dry_run=True))
    assert result['status_code']=='LOADED_ROUTE_BLOCKED' and not result['dry_run_valid']


def test_unloaded_transit_uses_pin_and_go_when_commands_available(tmp_path):
    fixture=_patrol_fixture(tmp_path,empty=True)
    fixture.ex.client.commands=NS()
    nav_calls=[]
    class Nav:
        def __init__(self,obs,commands,**_):
            self.commands=commands
            self.position=[2.,2.]
        def execute(self,goal):
            nav_calls.append((goal.target_x_m,goal.target_y_m))
            return dict(arrived=True,status_code='ARRIVED',final_position_m=[goal.target_x_m,goal.target_y_m])
    class FailMover:
        def __init__(self,*_,**__): raise AssertionError('mover fallback should not be used when PinAndGo is available')
    ledger=PatrolLedger(tmp_path)
    patrol=PatrolCollector(fixture.obs,fixture.ex,ledger,boundary_config=BoundaryConfig(allowed_segments=(1,),waypoint_spacing_m=10),
                           delivery_factory=FailMover,navigator_factory=Nav)
    patrol.observe_waypoint=lambda key: NS(garments=[])
    result=patrol.execute(NS(drop_x_m=1.,drop_y_m=1.,max_duration_s=60.,max_items=10,allow_towing=False,dry_run=False))
    assert nav_calls
    assert any(a['operation']=='pin_and_go' and a['status']=='OBSERVED' for a in ledger.data['actions'])
    assert result['partial_completion']


def room(pose=(1., 1., 0.)):
    pixels = np.full((80, 80), 15, np.uint8)
    pixels[[0, -1], :] = 1; pixels[:, [0, -1]] = 1
    occupancy = np.where(pixels == 1, 100, 0).astype(np.int8)
    return VendorMap(80, 80, (0., 0.), pixels, occupancy, pose, None, {1: {}}, {}, {})


class NavObservations:
    def __init__(self, poses=None):
        self.poses = iter(poses or [(1., 1., 0.)] * 100)
        self.sequence = 0
        self.states = iter([16, 3] * 100)
        self.arm_state = 'IN_ROBOT'
        self.status = dict(common_status=0, error_code=0, battery=80, in_cleaning=False, in_returning=False)
        self.verified = True
        self.stale = False

    def get(self, path):
        if path == '/api/arm/estimate':
            return {'state': self.arm_state}
        return {'telemetry': {'status': {'value': dict(self.status, state=next(self.states, 3))}}}

    def navigation_snapshot(self, refresh=False, max_age_s=5.):
        self.sequence += 1
        return NS(map=room(next(self.poses)), labels={}, sequence=self.sequence,
                  active_floor_verified=self.verified, current_pose_verified=self.verified,
                  cache_age_s=10. if self.stale else 0., evidence=None)


class NavCommands:
    latest_outcome = None

    def __init__(self):
        self.calls = []
        self.reply = dict(dispatch='sent', response='received')
        self.stops = 0

    def rpc(self, method, params, timeout_ms=5000):
        self.calls.append((method, params))
        return self.reply if method == 'app_goto_target' else dict(dispatch='sent', response='received')

    def stop(self):
        self.stops += 1


def nav_run(obs=None, commands=None, dry_run=True, **kwargs):
    obs = obs or NavObservations()
    commands = commands or NavCommands()
    result = PinAndGoNavigator(obs, commands, sleep=lambda _: None, **kwargs).execute(
        NS(target_x_m=2., target_y_m=1., max_duration_s=2., dry_run=dry_run))
    return result, commands


def test_dry_run_valid_does_not_dispatch():
    result, commands = nav_run()
    assert result['status_code'] == 'DRY_RUN_VALID' and result['dry_run_valid']
    assert not result['arrived'] and result['final_position_m'] == [1., 1.]
    assert result['position_error_m'] == 1.
    assert commands.calls == []


@pytest.mark.parametrize('attribute,value,code', [
    ('arm_state', 'UNKNOWN', 'ARM_NOT_STOWED'),
    ('verified', False, 'UNLOCALIZED_MAP'),
    ('stale', True, 'UNLOCALIZED_MAP'),
])
def test_fail_closed_preconditions(attribute, value, code):
    obs = NavObservations()
    setattr(obs, attribute, value)
    result, commands = nav_run(obs)
    assert result['status_code'] == code and commands.calls == []


@pytest.mark.parametrize('key,value', [
    ('common_status', 64), ('error_code', 1), ('battery', 15),
    ('in_cleaning', True), ('in_returning', True),
])
def test_state_preconditions(key, value):
    obs = NavObservations()
    obs.status[key] = value
    result, commands = nav_run(obs)
    assert result['status_code'] == 'ARM_NOT_STOWED' and not commands.calls


def test_unknown_dispatch_is_stopped_once_never_replayed():
    commands = NavCommands()
    commands.reply = dict(dispatch='unknown', response='transport_error')
    result, _ = nav_run(commands=commands, dry_run=False)
    assert result['status_code'] == 'UNKNOWN_DISPATCH'
    assert commands.calls == [('app_goto_target', [2000, 1000]), ('stop_goto_target', [])]
    assert commands.stops == 1


def test_vendor_rejection_not_arrival():
    commands = NavCommands()
    commands.reply = dict(dispatch='sent', response='vendor_error')
    result, _ = nav_run(commands=commands, dry_run=False)
    assert result['status_code'] == 'VENDOR_REJECTED' and not result['arrived']
    assert commands.calls == [('app_goto_target', [2000, 1000])]


@pytest.mark.parametrize('final,expected', [(1., 'STOPPED_SHORT'), (2., 'ARRIVED')])
def test_state_transition_requires_post_stop_position(final, expected):
    obs = NavObservations([(1., 1., 0.), (1.3, 1., 0.), (final, 1., 0.), (final, 1., 0.)])
    obs.states = iter([3, 3, 16, 3])
    result, commands = nav_run(obs, dry_run=False)
    assert result['status_code'] == expected
    assert result['arrived'] == (expected == 'ARRIVED')
    assert result['final_position_m'] == [final, 1.]
    assert commands.calls == [('app_goto_target', [2000, 1000])]


def test_live_boundary_exit_stops_native_and_rc():
    obs = NavObservations([(1., 1., 0.), (.1, 1., 0.)])
    obs.states = iter([3, 3, 16])
    result, commands = nav_run(obs, dry_run=False)
    assert result['status_code'] == 'BOUNDARY_ABORT'
    assert commands.calls[-1] == ('stop_goto_target', []) and commands.stops == 1


def test_cancel_before_dispatch_never_moves():
    result, commands = nav_run(dry_run=False, cancelled=lambda: True)
    assert result['status_code'] == 'CANCELLED' and commands.calls == []


def test_live_cancellation_stops():
    checks = iter([False, True])
    result, commands = nav_run(dry_run=False, cancelled=lambda: next(checks))
    assert result['status_code'] == 'CANCELLED' and commands.calls[-1] == ('stop_goto_target', [])


def test_timeout_stops_even_if_always_goto():
    obs = NavObservations()
    obs.states = iter([3, 3] + [16] * 100)
    ticks = iter([0., 0., .1, 1., 3.])
    result, commands = nav_run(obs, dry_run=False, clock=lambda: next(ticks))
    assert result['status_code'] == 'TIMED_OUT' and commands.stops == 1


def test_changed_map_index_is_not_arrival():
    obs = NavObservations([(1., 1., 0.), (2., 1., 0.), (2., 1., 0.), (2., 1., 0.)])
    obs.states = iter([3, 3, 16, 3])
    original = obs.navigation_snapshot

    def snapshot(**kwargs):
        s = original(**kwargs)
        s.map_index = 1 if s.sequence < 4 else 2
        return s

    obs.navigation_snapshot = snapshot
    result, commands = nav_run(obs, dry_run=False)
    assert result['status_code'] == 'BOUNDARY_ABORT' and commands.stops == 1


def test_real_dds_pin_and_go_dry_run_and_unverified_map():
    """Verify PinAndGo action over DDS with PickServer and latched ROS observations."""
    def raw_map(x=1000):
        m = room()
        pixels = m.pixels.tobytes()
        image = struct.pack('<HHIiiII', 2, 24, len(pixels), 0, 0, 80, 80) + pixels
        robot = struct.pack('<HHIiii', 8, 8, 12, x, 1000, 0)
        body = image + robot + struct.pack('<HHI', 1024, 8, 20)
        raw = b'rr' + struct.pack('<H', 20) + struct.pack('<I', len(body) + 20) + bytes(12) + body
        return raw + hashlib.sha1(raw).digest()

    def wait(predicate):
        until = time.monotonic() + 8.
        while not predicate():
            assert time.monotonic() < until, 'offline DDS deadline'
            time.sleep(.02)

    rclpy.init(domain_id=96)
    server = peer = action = executor = thread = None
    try:
        server = PickServer()
        peer = rclpy.create_node('pin_and_go_offline_fixture')
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(server); executor.add_node(peer)
        pubs = {cls: peer.create_publisher(cls, topic, LATCHED) for cls, topic in (
            (ArmEstimate, '/z70/arm_estimate'), (RobotState, '/z70/state'),
            (MapSnapshot, '/z70/driver/map_snapshot'))}
        state = RobotState(connected=True, robot_state=3, robot_state_known=True, error_code_known=True,
                           common_status_known=True, battery_known=True, battery_fraction=.8,
                           cleaning_known=True, returning_known=True)
        state.evidence.valid = True; state.evidence.received_at = peer.get_clock().now().to_msg()
        arm = ArmEstimate(); arm.evidence.valid = True; arm.validity.value = 3; arm.validity.reason = 'COMMAND_DERIVED_STOW'
        snapshot = MapSnapshot(raw=list(raw_map()), map_index=1, map_sequence=1)
        snapshot.evidence.valid = True; snapshot.evidence.sequence = 1
        snapshot.evidence.received_at = peer.get_clock().now().to_msg()
        pubs[RobotState].publish(state); pubs[ArmEstimate].publish(arm); pubs[MapSnapshot].publish(snapshot)
        calls = []; native = {'active': False, 'refreshes': 0}

        def session(request, response):
            response.accepted = True; response.task_id = UUID(uuid=[1] * 16); return response

        def rpc(request, response):
            calls.append((request.method, request.params_json))
            response.outcome.dispatch = 1; response.outcome.response = 1; response.outcome.error_code = 'NONE'
            response.result_json = '["ok"]'
            if request.method == 'app_goto_target':
                native['active'] = True; native['refreshes'] = 0; state.robot_state = 16
            return response

        def refresh(request, response):
            if native['active']:
                native['refreshes'] += 1
                x = 1300 if native['refreshes'] == 1 else 2000
                state.robot_state = 16 if native['refreshes'] == 1 else 3
            else:
                x = 1000
            state.evidence.received_at = peer.get_clock().now().to_msg(); pubs[RobotState].publish(state)
            snapshot.raw = list(raw_map(x)); snapshot.evidence.sequence += 1
            snapshot.evidence.received_at = peer.get_clock().now().to_msg(); pubs[MapSnapshot].publish(snapshot)
            response.outcome.error_code = 'NONE'; response.outcome.dispatch = 1; response.outcome.response = 1
            response.snapshot = snapshot.evidence; return response

        for srv_cls, srv_name, cb in [
            (TaskSession, '/z70/tasks/session', session),
            (RawRpc, '/z70/driver/raw_rpc', rpc),
            (Stop, '/z70/stop', lambda req, res: res),
            (RefreshMap, '/z70/map/refresh', refresh),
        ]:
            peer.create_service(srv_cls, srv_name, cb)
        action = ActionClient(peer, PinAndGo, '/z70/nav/pin_and_go')
        thread = threading.Thread(target=executor.spin); thread.start()
        assert action.wait_for_server(timeout_sec=5.)
        wait(lambda: server.observations.map_snapshot is not None and server.observations.arm is not None
             and server.observations.state is not None)

        def goal(dry_run=True):
            future = action.send_goal_async(PinAndGo.Goal(target_x_m=2., target_y_m=1., max_duration_s=5., dry_run=dry_run))
            wait(future.done); handle = future.result(); assert handle.accepted
            future = handle.get_result_async(); wait(future.done); return future.result()

        result = goal()
        assert result.status == 6 and result.result.status_code == 'UNLOCALIZED_MAP'
        snapshot.active_floor_verified = True; snapshot.current_pose_verified = True; snapshot.evidence.sequence = 2
        snapshot.evidence.received_at = peer.get_clock().now().to_msg(); pubs[MapSnapshot].publish(snapshot)
        wait(lambda: server.observations.map_snapshot.evidence.sequence == 2)
        result = goal()
        assert result.status == 4 and result.result.status_code == 'DRY_RUN_VALID'
        assert result.result.dry_run_valid and not result.result.arrived
        assert list(result.result.final_position_m) == [1., 1.]
        assert not server.lock.locked()
        assert calls == []
        result = goal(dry_run=False)
        assert result.status == 4 and result.result.status_code == 'ARRIVED'
        assert result.result.arrived and list(result.result.final_position_m) == [2., 1.]
        assert calls == [('app_goto_target', '[2000, 1000]')]
        assert not server.lock.locked()
    finally:
        if action is not None:
            action.destroy()
        if executor is not None:
            executor.shutdown(timeout_sec=5.)
        if thread is not None:
            thread.join(5.)
        if server is not None:
            server.observations.close(); server.destroy_node()
        if peer is not None:
            peer.destroy_node()
        rclpy.try_shutdown()
