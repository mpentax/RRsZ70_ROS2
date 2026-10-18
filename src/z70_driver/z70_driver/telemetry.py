"""Decode RoboClaw's public telemetry, without turning absent data into observations.

Common enums: python-roborock roborock/data/v1/v1_code_mappings.py.
Z70 additions 39/41/42: recovered vendor arm model and arm-feedback.ts.
Unknown numeric codes are deliberately retained, not mapped to a healthy default.
"""
import math

STATE_NAMES = {
    0: 'UNKNOWN', 1: 'STARTING', 2: 'CHARGER_DISCONNECTED', 3: 'IDLE',
    4: 'REMOTE_CONTROL_ACTIVE', 5: 'CLEANING', 6: 'RETURNING_HOME',
    7: 'MANUAL_MODE', 8: 'CHARGING', 9: 'CHARGING_PROBLEM', 10: 'PAUSED',
    11: 'SPOT_CLEANING', 12: 'ERROR', 13: 'SHUTTING_DOWN', 14: 'UPDATING',
    15: 'DOCKING', 16: 'GOING_TO_TARGET', 17: 'ZONED_CLEANING',
    18: 'SEGMENT_CLEANING', 22: 'EMPTYING_THE_BIN', 23: 'WASHING_THE_MOP',
    25: 'WASHING_THE_MOP', 26: 'GOING_TO_WASH_THE_MOP', 28: 'IN_CALL',
    29: 'MAPPING', 30: 'EGG_ATTACK', 32: 'PATROL', 33: 'ATTACHING_THE_MOP',
    34: 'DETACHING_THE_MOP', 39: 'REMOTE_PICKUP', 41: 'ARM_RESETTING',
    42: 'PROGRAM_MODE', 100: 'CHARGING_COMPLETE', 101: 'DEVICE_OFFLINE', 103: 'LOCKED',
}
ERROR_NAMES = dict(enumerate([
    'NONE', 'LIDAR_BLOCKED', 'BUMPER_STUCK', 'WHEELS_SUSPENDED', 'CLIFF_SENSOR_ERROR',
    'MAIN_BRUSH_JAMMED', 'SIDE_BRUSH_JAMMED', 'WHEELS_JAMMED', 'ROBOT_TRAPPED',
    'NO_DUSTBIN', 'STRAINER_ERROR', 'COMPASS_ERROR', 'LOW_BATTERY', 'CHARGING_ERROR',
    'BATTERY_ERROR', 'WALL_SENSOR_DIRTY', 'ROBOT_TILTED', 'SIDE_BRUSH_ERROR', 'FAN_ERROR',
    'DOCK_POWER_ERROR', 'OPTICAL_FLOW_SENSOR_DIRTY', 'VERTICAL_BUMPER_PRESSED',
    'DOCK_LOCATOR_ERROR', 'RETURN_TO_DOCK_FAIL', 'NOGO_ZONE_DETECTED', 'VISUAL_SENSOR_ERROR',
    'LIGHT_TOUCH_ERROR', 'VIBRARISE_JAMMED', 'ROBOT_ON_CARPET', 'FILTER_BLOCKED',
    'INVISIBLE_WALL_DETECTED', 'CANNOT_CROSS_CARPET', 'INTERNAL_ERROR',
]))
ERROR_NAMES.update({34: 'COLLECT_DUST_ERROR', 35: 'AUTO_EMPTY_DOCK_VOLTAGE_ERROR',
                    38: 'CHECK_CLEAN_WATER_TANK', 39: 'CHECK_DIRTY_WATER_TANK',
                    41: 'CLEAN_WATER_EMPTY', 42: 'DOCK_WATER_FILTER_ERROR',
                    44: 'DOCK_WATER_FILTER_BLOCKED', 48: 'UP_WATER_EXCEPTION',
                    49: 'DRAIN_WATER_EXCEPTION', 51: 'TEMPERATURE_PROTECTION',
                    53: 'CLEANING_TANK_FULL_OR_BLOCKED', 56: 'AUDIO_ERROR'})


def decode_command(envelope, now):
    commands = [c for c in envelope.get('commands', []) if isinstance(c, dict)]
    latest = max(commands, key=lambda c: number(c.get('requestedAt')) if math.isfinite(number(c.get('requestedAt'))) else -1, default={})
    stamp = number(latest.get('sentAt'))
    if not math.isfinite(stamp):
        stamp = number(latest.get('requestedAt'))
    return dict(age_s=max(0., now - stamp / 1000.) if math.isfinite(stamp) else math.nan,
                method=latest.get('method', 'none'),
                outcome=f"{latest.get('dispatch', 'none')}/{latest.get('response', 'none')}")


def decode_arm(estimate):
    state = estimate.get('state', 'UNKNOWN')
    values = dict(state=state, unknown_reason=estimate.get('unknownReason') or '',
                  steps_since_anchor=number(estimate.get('stepsSinceAnchor')))
    for joint in ('m3', 'm4', 'm5', 'm6'):
        values[joint + '_deg'] = number((estimate.get('joints') or {}).get(joint)) if state not in ('UNKNOWN', 'IN_ROBOT') else math.nan
        values['sigma_' + joint + '_deg'] = number((estimate.get('sigmaDeg') or {}).get(joint))
    return values


def decode_calibration(calibration, odom):
    camera = calibration.get('camera_x') or {}
    values = dict(camera_x_m=number(camera.get('value_m')), camera_x_sigma_m=number(camera.get('sigma_m')),
                  camera_x_ready=bool(camera.get('ready')), samples=number(calibration.get('n')),
                  active_camera_x_m=number(odom.get('camera_x_m')),
                  vo_accepted=number(odom.get('ok')), vo_rejected=number(odom.get('rejected')),
                  vo_state=odom.get('state', 'unknown'))
    for key in ('yaw_positive', 'yaw_negative', 'forward'):
        values[key + '_gain'] = number((calibration.get(key) or {}).get('gain'))
    return values


def number(value):
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) else math.nan


def decode_status(envelope, now):
    entry = envelope.get('telemetry', {}).get('status', {})
    status = entry.get('value') or {}
    if isinstance(status, list):
        status = status[0] if status and isinstance(status[0], dict) else {}
    if not isinstance(status, dict):
        status = {}
    state = status.get('state')
    error = status.get('error_code')
    age = max(0., now - entry['observedAt'] / 1000.) if math.isfinite(number(entry.get('observedAt'))) else math.nan
    valid = bool(status) and entry.get('error') is None and math.isfinite(age) and age < 10
    code = state if isinstance(state, int) and not isinstance(state, bool) else -1
    error_code = error if isinstance(error, int) and not isinstance(error, bool) else -1
    return dict(battery=number(status.get('battery')) if valid else math.nan, state_code=code,
                state_name=STATE_NAMES.get(code, f'UNKNOWN({code})'), error_code=error_code,
                error_name=ERROR_NAMES.get(error_code, f'UNKNOWN({error_code})'),
                cleaning=bool(status.get('in_cleaning')) if valid and 'in_cleaning' in status else None,
                returning=bool(status.get('in_returning')) if valid and 'in_returning' in status else None,
                charging=code == 8 if valid and code >= 0 else None,
                connection_state=envelope.get('connection', {}).get('state', 'unknown'),
                connected=envelope.get('connection', {}).get('state') == 'connected',
                status_valid=valid, status_age_s=age)
