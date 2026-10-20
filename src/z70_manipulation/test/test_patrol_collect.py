"""Patrol ledger persistence, waypoint coverage, and battery return logic."""
from types import SimpleNamespace as NS
import json
import pytest
from z70_manipulation.patrol import PatrolLedger, PatrolCollector


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
    from z70_interfaces.action import PatrolCollect
    from test_garment_delivery import run_dds_dry_action
    run_dds_dry_action(tmp_path,PatrolCollect,'/z70/patrol_collect',
        PatrolCollect.Goal(drop_x_m=2.,drop_y_m=2.,max_duration_s=10.,max_items=10,allow_towing=False,dry_run=True),monkeypatch)


@pytest.mark.parametrize('delivery_status,expected', [('GRASP_MISS','FAILED'),('GARMENT_SLIPPED_DURING_TRANSPORT','FAILED'),('DELIVERED','DELIVERED')])
def test_patrol_records_verified_delivery_not_attempt_as_success(tmp_path,delivery_status,expected):
    from test_garment_delivery import DeliveryFixture
    from z70_mapping.boundary import BoundaryConfig
    fixture=DeliveryFixture(tmp_path)
    fixture.garment.floor_contact_m=NS(x=.4,y=0.)
    fixture.garment.evidence=NS(session_id=NS(uuid=[1]*16),sequence=1)
    fixture.obs.get=lambda _:dict(telemetry=dict(status=dict(value=dict(battery=90))))
    fixture.obs.garment_frame=lambda **_:dict(coverage_valid=True,valid_empty_view=False)
    fixture.obs.garment_snapshot=lambda **_:NS(garments=[fixture.garment],evidence=fixture.garment.evidence,model_available=True,geofence_active=True)
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
    assert result['partial_completion']  # occupied single view is not room clearance


def test_unsupported_towing_is_skipped_without_delivery_dispatch(tmp_path):
    from test_garment_delivery import DeliveryFixture
    from z70_mapping.boundary import BoundaryConfig
    fixture=DeliveryFixture(tmp_path); fixture.garment.transport_suitability=2
    fixture.garment.floor_contact_m=NS(x=.4,y=0.)
    evidence=NS(session_id=NS(uuid=[1]*16),sequence=1)
    fixture.obs.get=lambda _:dict(telemetry=dict(status=dict(value=dict(battery=90))))
    fixture.obs.garment_frame=lambda **_:dict(coverage_valid=True,valid_empty_view=False)
    fixture.obs.garment_snapshot=lambda **_:NS(garments=[fixture.garment],evidence=evidence,model_available=True,geofence_active=True)
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
    from test_garment_delivery import DeliveryFixture
    from z70_mapping.boundary import BoundaryConfig
    fixture=DeliveryFixture(tmp_path)
    patrol=PatrolCollector(fixture.obs,None,PatrolLedger(tmp_path),boundary_config=BoundaryConfig(allowed_segments=(1,)))
    result=patrol.execute(NS(drop_x_m=.05,drop_y_m=.05,max_duration_s=10.,max_items=10,allow_towing=False,dry_run=True))
    assert result['status_code']=='LOADED_ROUTE_BLOCKED' and not result['dry_run_valid']


def test_unloaded_transit_uses_pin_and_go_when_commands_available(tmp_path):
    from test_garment_delivery import DeliveryFixture
    from z70_mapping.boundary import BoundaryConfig
    fixture=DeliveryFixture(tmp_path)
    fixture.obs.get=lambda _:dict(telemetry=dict(status=dict(value=dict(battery=90))))
    fixture.obs.garment_frame=lambda **_:dict(coverage_valid=True,valid_empty_view=True)
    fixture.obs.garment_snapshot=lambda **_:NS(garments=[],evidence=NS(session_id=NS(uuid=[1]*16),sequence=1),model_available=True,geofence_active=True)
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
