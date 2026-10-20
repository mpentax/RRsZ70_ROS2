"""CommandEvent attempt joining for online chassis calibration."""
from copy import deepcopy
from z70_interfaces.msg import CommandEvent
from z70_odometry.chassis_calibrator import CommandStream, OnlineCalibration


def event(phase,seq,attempt=2,dispatch=1,response=1):
    msg=CommandEvent(phase=phase,operation='app_rc_move')
    msg.evidence.session_id.uuid=[1]*16;msg.evidence.sequence=seq
    msg.evidence.source_time_known=True;msg.evidence.source_clock=1
    msg.evidence.source_at.sec=100
    msg.outcome.command_id.uuid=[9]*16
    msg.outcome.attempt_id.uuid=[attempt]*16
    msg.outcome.dispatch=dispatch;msg.outcome.response=response
    msg.requested_twist.angular.z=.2;msg.requested_duration.nanosec=500_000_000
    return msg


def test_unknown_retry_and_finished_event_never_double_count(tmp_path):
    estimator=OnlineCalibration(tmp_path/'calibration.json');stream=CommandStream(estimator)
    stream.add(event(0,1))
    stream.add(event(1,2));stream.add(event(2,3,dispatch=2,response=3))
    assert not estimator.pending and estimator.skipped==1
    stream.add(event(1,4,attempt=3));stream.add(event(2,5,attempt=3))
    stream.add(event(4,6,attempt=3));stream.add(event(4,6,attempt=3))
    assert len(estimator.pending)==1 and len(estimator.seen)==2
    assert estimator.pending[0]['start']==100
    assert estimator.pending[0]['id'].endswith(bytes([3]*16).hex())


def test_gap_restart_or_unmapped_clock_does_not_invent_attempts(tmp_path):
    estimator=OnlineCalibration(tmp_path/'calibration.json');stream=CommandStream(estimator)
    stream.add(event(1,1));stream.add(event(2,3))
    assert not estimator.pending and estimator.skip_reasons['command event gap']==1
    msg=event(1,4);msg.evidence.source_clock=2
    stream.add(msg);stream.add(event(2,5))
    assert not estimator.pending
    msg=event(1,1);msg.evidence.session_id.uuid=[5]*16;stream.add(msg)
    reply=event(2,2);reply.evidence.session_id.uuid=[5]*16;stream.add(reply)
    assert len(estimator.pending)==1
    assert estimator.skip_reasons['command event gap']==2
