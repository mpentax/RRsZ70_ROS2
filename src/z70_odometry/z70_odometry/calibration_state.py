"""Typed public fit state."""
import math
import uuid
from z70_interfaces.msg import CalibrationState


def _number(value, nonnegative=False):
    return (isinstance(value, (float, int)) and not isinstance(value, bool)
            and math.isfinite(value) and (not nonnegative or value >= 0))


def calibrated_camera_x(state):
    """Only an explicitly usable fit may change the active floor camera offset."""
    if (state.evidence.valid and not state.evidence.stale and state.validity.value == 1
            and state.camera_x_ready and state.camera_x_known and state.camera_x_sigma_known
            and _number(state.camera_x_m) and _number(state.camera_x_sigma_m, True)):
        return float(state.camera_x_m)
    return None


def gains_from_state(state):
    """Executor-compatible gains; absence is unknown, not a guessed replacement."""
    if not state.evidence.valid or state.evidence.stale or state.validity.value != 1:
        return {}
    result = {}
    for name in ('yaw_positive', 'yaw_negative', 'forward'):
        value = getattr(state, name + '_gain')
        if getattr(state, name + '_known') and _number(value):
            sigma = getattr(state, name + '_sigma')
            result[name] = dict(gain=float(value), n=getattr(state, name + '_count'),
                                sigma=float(sigma) if getattr(state, name + '_sigma_known') and _number(sigma, True) else None)
    return result


def fill_state(msg, summary, calibration_id, updated_at):
    """Populate flat fields; missing values stay zero with explicit unknown flags."""
    msg.calibration_id = calibration_id
    msg.sample_count = int(summary['n'])
    msg.skipped_count = int(summary['skipped'])
    msg.model = summary['model']
    msg.header.frame_id = 'base_footprint'
    source_ns = round(updated_at * 10**9) if updated_at is not None else 0
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(source_ns, 10**9)
    msg.evidence.source_at = msg.header.stamp
    msg.evidence.source_time_known = updated_at is not None
    msg.evidence.source_clock = 1 if updated_at is not None else 0
    msg.evidence.timestamp_kind = 3
    msg.evidence.valid = summary['n'] > 0
    msg.evidence.reason = 'FIT_ESTIMATE' if summary['n'] else 'NO_SAMPLES'
    msg.validity.value = 1 if summary['n'] else 0
    msg.validity.reason = msg.evidence.reason
    for name in ('yaw_positive', 'yaw_negative', 'forward'):
        value = summary[name]
        setattr(msg, name + '_known', _number(value['gain']))
        setattr(msg, name + '_gain', float(value['gain']) if _number(value['gain']) else 0.)
        setattr(msg, name + '_sigma_known', _number(value['sigma'], True))
        setattr(msg, name + '_sigma', float(value['sigma']) if _number(value['sigma'], True) else 0.)
        setattr(msg, name + '_count', int(value['n']))
    camera = summary['camera_x']
    for source, field, known in [('value_m', 'camera_x_m', 'camera_x_known'),
                                  ('sigma_m', 'camera_x_sigma_m', 'camera_x_sigma_known'),
                                  ('residual_rms_m', 'camera_x_residual_rms_m', 'camera_x_residual_known')]:
        valid = _number(camera[source], source != 'value_m')
        setattr(msg, known, valid)
        setattr(msg, field, float(camera[source]) if valid else 0.)
    msg.camera_x_count = int(camera['n'])
    msg.camera_x_ready = bool(camera['ready'])
    return msg


class CalibrationPublication:
    """Cache original update evidence across periodic/late-subscriber publication."""
    def __init__(self):
        self.session = uuid.uuid4()
        self.sequence = 0
        self.message = None
        self.identity = None

    def snapshot(self, estimator, received_at, message_type=None):
        identity = estimator.calibration_id
        if self.message is None or self.identity != identity:
            if message_type is None:
                message_type = CalibrationState
            self.sequence += 1
            msg = fill_state(message_type(), estimator.summary(), identity, estimator.updated_at)
            msg.evidence.received_at = received_at
            msg.evidence.session_id.uuid = list(self.session.bytes)
            msg.evidence.sequence = self.sequence
            self.message, self.identity = msg, identity
        return self.message
