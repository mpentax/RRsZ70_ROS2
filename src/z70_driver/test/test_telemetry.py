import math
import pytest
from z70_driver import telemetry


def test_decodes_real_service_envelope_without_confusing_connection_state():
    state = {'connection': {'state': 'connected'}, 'telemetry': {'status': {
        'value': {'battery': 97, 'state': 8, 'error_code': 0, 'in_cleaning': 0, 'in_returning': 0},
        'observedAt': 100000, 'error': None}}}
    decoded = telemetry.decode_status(state, now=101)
    assert decoded['battery'] == 97
    assert decoded['state_name'] == 'CHARGING'
    assert decoded['error_name'] == 'NONE'
    assert decoded['charging'] is True
    assert decoded['cleaning'] is False
    assert decoded['connected'] is True
    assert decoded['status_age_s'] == 1
    assert decoded['status_valid'] is True


def test_command_age_uses_latest_attempt_and_never_claims_physical_completion():
    decoded = telemetry.decode_command({'commands': [
        {'requestedAt': 1000, 'sentAt': 1000, 'replyAt': 1200, 'method': 'app_rc_move', 'dispatch': 'sent', 'response': 'received'},
        {'requestedAt': 2000, 'sentAt': None, 'replyAt': 2100, 'method': 'app_rc_end', 'dispatch': 'not_sent', 'response': 'transport_error'}]}, now=4)
    assert decoded['age_s'] == 2
    assert decoded['outcome'] == 'not_sent/transport_error'
    assert decoded['method'] == 'app_rc_end'
    assert math.isnan(telemetry.decode_command({'commands': []}, now=4)['age_s'])


def test_unknown_arm_is_not_a_measured_zero_joint():
    values = telemetry.decode_arm({'state': 'UNKNOWN', 'unknownReason': 'mode exited', 'joints': {'m3': 0}, 'sigmaDeg': {'m3': 2.5}})
    assert values['state'] == 'UNKNOWN'
    assert values['unknown_reason'] == 'mode exited'
    assert math.isnan(values['m3_deg'])
    assert values['sigma_m3_deg'] == 2.5
    deployed = telemetry.decode_arm({'state': 'WORKING_POS', 'joints': {'m3': 150}, 'sigmaDeg': {'m3': 1}})
    assert deployed['m3_deg'] == 150


def test_calibration_nulls_remain_unknown_and_vo_counts_are_typed():
    values = telemetry.decode_calibration({'yaw_positive': {'gain': 1.2}, 'camera_x': {'value_m': None, 'ready': False}}, {'ok': 3, 'rejected': 2, 'camera_x_m': .16})
    assert values['yaw_positive_gain'] == 1.2
    assert math.isnan(values['camera_x_m'])
    assert values['camera_x_ready'] is False
    assert values['vo_accepted'] == 3
    assert values['vo_rejected'] == 2
    assert values['active_camera_x_m'] == .16


@pytest.mark.parametrize('code,name', [(2,'CHARGER_DISCONNECTED'),(3,'IDLE'),(39,'REMOTE_PICKUP'),(42,'PROGRAM_MODE'),(999,'UNKNOWN(999)')])
def test_state_codes_are_not_inferred_from_modes(code,name):
    decoded = telemetry.decode_status({'telemetry': {'status': {'value': {'state': code}}}},now=10)
    assert decoded['state_name'] == name


def test_absent_status_does_not_claim_no_error_or_empty_battery():
    decoded = telemetry.decode_status({},now=10)
    assert decoded['status_valid'] is False
    assert decoded['error_code'] == -1
    assert decoded['error_name'] == 'UNKNOWN(-1)'
    assert decoded['charging'] is None
    assert math.isnan(decoded['battery'])


def test_cached_status_error_or_age_invalidates_live_battery_and_flags():
    for entry in [
        {'value': {'state': 8, 'battery': 80, 'in_cleaning': 0}, 'observedAt': 1000, 'error': None},
        {'value': {'state': 8, 'battery': 80, 'in_cleaning': 0}, 'observedAt': 10000, 'error': 'timeout'}]:
        decoded = telemetry.decode_status({'telemetry': {'status': entry}},now=11)
        assert decoded['status_valid'] is False
        assert math.isnan(decoded['battery'])
        assert decoded['charging'] is None
        assert decoded['cleaning'] is None
