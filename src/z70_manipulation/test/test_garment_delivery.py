"""Unit and DDS tests for garment delivery."""
import threading
import time
from types import SimpleNamespace as NS
import numpy as np
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from z70_interfaces.action import DeliverGarment
from z70_interfaces.msg import EvidenceStamp, GarmentObservation, GarmentObservations
from z70_manipulation import planner
from z70_manipulation.garment_delivery import (
    DeliveryRejected,
    GarmentDeliverer,
    TowingQualificationConfig,
    loaded_route,
    transport_policy,
)
from z70_manipulation.node import PickServer
from z70_manipulation.observations import LATCHED
from z70_mapping.boundary import BoundaryConfig, compute_safe_interior


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
            for a in server.action_servers: a.destroy()
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
