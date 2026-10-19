"""Calibration resource paths, persistence, and state mapping tests."""
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest
from z70_odometry.chassis_calibrator import OnlineCalibration
from z70_odometry.calibration_state import CalibrationPublication, calibrated_camera_x, gains_from_state
from z70_odometry import resources


def message():
    return NS(header=NS(stamp=NS()), evidence=NS(stale=False,session_id=NS()), validity=NS())


def test_runtime_calibration_path(monkeypatch,tmp_path):
    monkeypatch.setenv('XDG_DATA_HOME',str(tmp_path/'data'))
    first=resources.runtime_calibration_path()
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv('Z70_REPO','/unusable/archive')
    assert first==resources.runtime_calibration_path()==tmp_path/'data/RRsZ70_ROS2/calibration/chassis/online.json'
    assert not first.exists()
    monkeypatch.setenv('XDG_DATA_HOME','relative-invalid')
    assert resources.runtime_calibration_path()==Path.home()/'.local/share/RRsZ70_ROS2/calibration/chassis/online.json'
    monkeypatch.delenv('XDG_DATA_HOME')
    assert resources.runtime_calibration_path()==Path.home()/'.local/share/RRsZ70_ROS2/calibration/chassis/online.json'


def test_initial_camera_x_m():
    assert resources.initial_camera_x_m()==.16


def test_atomic_save_retains_identity(tmp_path):
    path=tmp_path/'data/online.json'; estimator=OnlineCalibration(path)
    estimator.seed(resources.seed_directory())
    assert estimator.summary()['n']>0
    original=path.read_bytes(); ident=hashlib.sha256(original).hexdigest()
    assert estimator.calibration_id==ident
    restored=OnlineCalibration(path); restored.seed(resources.seed_directory()); restored.save()
    assert path.read_bytes()==original
    assert restored.updated_at==estimator.updated_at
    assert restored.calibration_id==ident
    assert restored.summary()==estimator.summary()
    restored.skip('test observation gap'); restored.save()
    assert restored.calibration_id!=ident
    assert not list(path.parent.glob('*.tmp'))
    assert path.stat().st_mode & 0o077==0


def test_empty_state_snapshot(tmp_path):
    estimator=OnlineCalibration(tmp_path/'online.json'); estimator.save()
    publication=CalibrationPublication()
    state=publication.snapshot(estimator,'initial receipt',message)
    assert not state.evidence.valid and state.validity.value==0
    assert not state.forward_known and not state.forward_sigma_known
    assert state.forward_gain==state.forward_sigma==0
    assert not state.camera_x_known and not state.camera_x_ready
    assert calibrated_camera_x(state) is None and gains_from_state(state)=={}
    assert publication.snapshot(estimator,'later timer',message) is state
    assert state.evidence.received_at=='initial receipt'
    assert state.evidence.sequence==1
    assert state.calibration_id==hashlib.sha256(estimator.path.read_bytes()).hexdigest()


def test_seeded_state_snapshot(tmp_path):
    estimator=OnlineCalibration(tmp_path/'online.json'); estimator.seed(resources.seed_directory())
    publication=CalibrationPublication(); state=publication.snapshot(estimator,'receipt',message)
    assert state.validity.value==1 and state.evidence.valid
    assert state.sample_count==estimator.summary()['n']
    assert state.evidence.source_time_known and state.evidence.source_clock==1
    assert state.evidence.timestamp_kind==3
    assert state.camera_x_m==estimator.summary()['camera_x']['value_m']
    for name,value in gains_from_state(state).items():
        assert value['gain']==estimator.summary()[name]['gain']
    state.evidence.stale=True
    assert gains_from_state(state)=={} and calibrated_camera_x(state) is None


def test_single_sample_sigma_and_ready_gate(tmp_path):
    estimator=OnlineCalibration(tmp_path/'online.json')
    estimator.add_measurement(dict(id='synthetic',command_yaw=.3,command_forward=0.,yaw=.2,
                                  camera_dx=.28*(np.cos(.2)-1),camera_dy=.28*np.sin(.2)))
    estimator.save(); state=CalibrationPublication().snapshot(estimator,'receipt',message)
    assert state.camera_x_known and not state.camera_x_sigma_known
    assert not state.camera_x_ready and calibrated_camera_x(state) is None
    assert state.yaw_positive_known and not state.yaw_positive_sigma_known

