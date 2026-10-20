"""Unit tests for ArmEstimator command and status tracking."""
import math
import pytest
from z70_driver.estimator import ArmEstimator
from z70_description.arm_model import vendor_forward, vendor_inverse


def deployed(now=lambda:100.):
    e=ArmEstimator(now=now);e.set_anchor('stowed');e.observe('app_arm_out_compartment',{},'received');return e


def test_unknown_start_and_deploy_requires_unrestored_stow():
    e=ArmEstimator();assert e.snapshot()['state']=='UNKNOWN'
    assert not e.observe('app_arm_out_compartment',{},'received')
    e.set_anchor('stowed');e.observe('app_arm_out_compartment',{},'received')
    assert e.snapshot()['joints']==dict(m3=150,m4=45,m5=0,m6=90)
    e.observe('app_stop_grasp',[],'received');assert e.snapshot()['wrist'] is None
    assert not e.observe('app_arm_out_compartment',{},'received')


def test_nominal_vendor_endpoint_is_not_the_cad_wrist():
    a1, a2 = map(math.radians, (120, -15))
    x = .0475 + .1775*math.cos(a1) + .205*math.cos(a2)
    y = .14 + .1775*math.sin(a1) + .205*math.sin(a2)
    e = deployed()
    assert e.snapshot()['wrist'] == pytest.approx(dict(x=x, y=y))
    assert e.observe('app_arm_direction_move', {'direction': 3}, 'received')
    d = e.snapshot()
    assert d['wrist'] == pytest.approx(dict(x=x+.04, y=y))
    j = d['joints']
    u, v = map(math.radians, (270-j['m3'], j['m4']-j['m3']+90))
    assert .0475+.1775*math.cos(u)+.205*math.cos(v) == pytest.approx(x+.04)
    assert .14+.1775*math.sin(u)+.205*math.sin(v) == pytest.approx(y)


def test_nominal_fk_ik_and_hand_not_physical_transform():
    for m3,m4 in ((150,45),(200,90),(240,130)):
        assert vendor_inverse(*vendor_forward(m3,m4)[1],(m3,m4))==pytest.approx((m3,m4))
    e=deployed();before=e.snapshot()['wrist']
    for direction in (3,3,2): e.observe('app_arm_direction_move',dict(direction=direction),'received')
    assert e.snapshot()['wrist']==pytest.approx(dict(x=before['x']+.08,y=before['y']-.04))


def test_jog_direction_sigma_clamp_and_firmware_predictive_limit():
    e=deployed()
    for m in ([0,0,1],[0,0,1],[0,0,1],[0,0,2]):e.observe('app_arm_move',dict(m=m),'received')
    d=e.snapshot();assert d['joints']['m5']==20 and d['stepsSinceAnchor']==4
    assert d['sigmaDeg']['m5']==pytest.approx(5.)
    assert e.observe('app_arm_move',dict(m=[0,0,2,0,0]),'received')
    assert e.snapshot()['joints']['m5']==10
    assert not e.observe('app_pick_up_enter_exit',dict(flag=1),'received')
    assert e.observe('app_put_down',dict(flag=0),'received')
    assert e.snapshot()['state']=='IN_PROCESS_POS' and e.snapshot()['joints']['m6']==90
    for _ in range(20):e.observe('app_arm_move',dict(m=[0,1,0]),'received')
    assert e.snapshot()['joints']['m4']==170
    e=deployed()
    for m in ([1,1,0],[1,1,0],[1,0,0],[1,1,0],[1,1,0],[1,1,0],[1,1,0],[1,0,0],[0,1,0]):
        e.observe('app_arm_move',dict(m=m),'received')
        assert e.snapshot()['wrist']['x']<=.367


@pytest.mark.parametrize('outcome',['not_sent','vendor_error'])
def test_rejected_unsent_and_logical_duplicate_never_integrate(outcome):
    e=deployed();before=e.snapshot()
    assert not e.observe('app_arm_move',dict(m=[0,0,1]),outcome)
    assert e.snapshot()==before
    e.observe('app_arm_move',dict(m=[0,0,1]),'received',command_id='one')
    assert not e.observe('app_arm_move',dict(m=[0,0,1]),'received',command_id='one')
    assert e.snapshot()['joints']['m5']==10


def test_unknown_jog_cannot_recover_without_stow_and_grip_never_lifts():
    e=deployed();e.observe('app_arm_move',dict(m=[0,0,1]),'unknown')
    assert e.snapshot()['state']=='UNKNOWN'
    assert not e.observe('app_arm_move',dict(m=[0,0,1]),'received')
    e.observe('app_arm_in_compartment',{},'received');assert e.snapshot()['state']=='IN_ROBOT'
    e=deployed();wrist=e.snapshot()['wrist']
    e.observe('app_pick_up_enter_exit',dict(flag=1),'received')
    assert not e.observe('app_grip',dict(flag=0),'unknown')
    assert e.snapshot()['state']=='WORKING_POS' and e.snapshot()['joints']['m6']==90
    assert e.observe('app_grip',dict(flag=0),'received')
    assert e.snapshot()['state']=='WORKING_POS' and e.snapshot()['joints']['m6']==0 and e.snapshot()['wrist']==wrist
    for outcome in ('received','unknown'):
        e=deployed();j=e.snapshot()['joints']
        e.observe('app_open_gripper',{},'unknown');assert e.snapshot()['state']=='WORKING_POS'
        e.observe('app_grip',{},outcome)
        assert e.snapshot()['state']=='UNKNOWN' and e.snapshot()['joints']==j and e.snapshot()['wrist'] is None
        assert e.snapshot()['unknownReason']=='GRIP_POSE_UNVERIFIED'


def test_program_priority_steps_and_autonomous_unknown():
    e=deployed();e.observe('app_start_program',dict(cmd_ids=[
        dict(cmd_id='arm_joints',value=[0,1,1,2],step=2),dict(cmd_id='delay'),
        dict(cmd_id='arm_move',direction=1,step=1)]),'received')
    d=e.snapshot();assert d['stepsSinceAnchor']==3 and d['joints']['m5']==20 and d['joints']['m6']==0
    assert d['wrist']['y']==pytest.approx(vendor_forward(150,65)[1][1]+.04)
    e.observe('app_start_program',dict(cmd_ids=[dict(cmd_id='arm_grasp')]),'received')
    assert e.snapshot()['state']=='UNKNOWN'
    e.observe('app_start_program',dict(cmd_ids=[dict(cmd_id='arm_in')]),'received')
    assert e.snapshot()['state']=='IN_ROBOT'


def test_status_old_requests_mode_exit_and_official_app_visibility_limits():
    clock=[100.];e=deployed(lambda:clock[0]);clock[0]+=1
    e.observe_status(dict(state=39,common_status=1<<10),101000)
    before=e.snapshot()
    e.observe_status(dict(state=39,common_status=1<<10),102000);assert e.snapshot()==before
    e.observe_status(dict(common_status=0),101500);assert e.snapshot()==before
    e.observe_status(dict(common_status=0),103000);assert e.snapshot()['state']=='UNKNOWN'
    e=deployed(lambda:100.);e.observe_status(dict(state=3),99999);assert e.snapshot()['state']=='WORKING_POS'
    e.observe_status(dict(state=42,common_status=0),102000);assert e.snapshot()['state']=='WORKING_POS'
    e.observe_status(dict(state=41),103000);assert e.snapshot()['state']=='UNKNOWN'


def test_restart_is_unreconciled_even_if_saved_stowed(tmp_path):
    path=tmp_path/'durable/arm.json';e=ArmEstimator(path=path);e.set_anchor('stowed')
    e.observe('app_arm_out_compartment',{},'received');e.observe('app_arm_move',dict(m=[0,0,1]),'received')
    loaded=ArmEstimator(path=path);d=loaded.snapshot()
    assert d['restored'] and d['joints']['m5']==10 and d['wrist'] is None
    assert not loaded.observe('app_arm_move',dict(m=[0,0,1]),'received')
    loaded.set_anchor('stowed');assert not loaded.snapshot()['restored']
    assert ArmEstimator(path=path).snapshot()['restored']
    path.write_text('{broken');assert ArmEstimator(path=path).snapshot()['state']=='UNKNOWN'

