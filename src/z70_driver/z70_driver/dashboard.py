"""Read-only, typed dashboard topics and standard ROS diagnostics."""
import math
import time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Bool, Float32, Int32, String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from z70_interfaces.msg import CalibrationState, OdometryStatus, CommandEvent
from .commands import LATCHED

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


def number(value):
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) else math.nan


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
    joints, sigma = estimate.get('joints') or {}, estimate.get('sigmaDeg') or {}
    values = dict(state=state, unknown_reason=estimate.get('unknownReason') or '',
                  steps_since_anchor=number(estimate.get('stepsSinceAnchor')))
    for joint in ('m3', 'm4', 'm5', 'm6'):
        values[joint + '_deg'] = number(joints.get(joint)) if state not in ('UNKNOWN', 'IN_ROBOT') else math.nan
        values['sigma_' + joint + '_deg'] = number(sigma.get(joint))
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


STATUS_TYPES = {'battery': Float32, 'state_code': Int32, 'state_name': String,
                'error_code': Int32, 'error_name': String, 'cleaning': Bool,
                'returning': Bool, 'charging': Bool, 'connected': Bool,
                'connection_state': String, 'status_valid': Bool, 'status_age_s': Float32}
ARM_TYPES = {'state': String, 'unknown_reason': String, 'steps_since_anchor': Float32,
             **{j + '_deg': Float32 for j in ('m3', 'm4', 'm5', 'm6')},
             **{'sigma_' + j + '_deg': Float32 for j in ('m3', 'm4', 'm5', 'm6')}}
CAL_TYPES = {key: Float32 for key in ('camera_x_m', 'camera_x_sigma_m', 'samples',
             'active_camera_x_m', 'vo_accepted', 'vo_rejected', 'yaw_positive_gain',
             'yaw_negative_gain', 'forward_gain')}
CAL_TYPES.update(camera_x_ready=Bool, vo_state=String)


class Dashboard:
    def __init__(self, node):
        self.node = node
        self.state = {}; self.arm = {}; self.commands = {}; self.calibration = {}; self.odom = {}
        self.state_received = self.arm_received = self.commands_received = self.cal_received = self.odom_received = 0.
        self.publishers = {}
        self.types = {}
        for prefix, types in [('/z70/', STATUS_TYPES), ('/z70/arm/', ARM_TYPES),
                              ('/z70/calibration/', CAL_TYPES),
                              ('/z70/last_command/', {'age_s': Float32, 'outcome': String, 'method': String})]:
            for key, kind in types.items():
                topic = prefix + key
                self.publishers[topic] = node.create_publisher(kind, topic, 10)
                self.types[topic] = kind
        self.diagnostics = node.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.path = Path()
        self.pub_path = node.create_publisher(Path, '/z70/odom_path', 10)
        node.create_subscription(Odometry, '/odom', self.on_odometry, 10)
        node.create_subscription(CalibrationState, '/z70/chassis_calibration', self.on_calibration, LATCHED)
        node.create_subscription(OdometryStatus, '/z70/odom_status', self.on_odom, 10)
        node.create_subscription(CommandEvent, '/z70/commands', self.on_command, 256)
        node.create_timer(1., self.publish)

    def on_odometry(self, msg):
        if self.path.header.frame_id != msg.header.frame_id:
            self.path.poses = []
        self.path.header = msg.header
        pose = PoseStamped(header=msg.header, pose=msg.pose.pose)
        if self.path.poses:
            prev = self.path.poses[-1].pose.position
            if math.hypot(pose.pose.position.x - prev.x, pose.pose.position.y - prev.y) < 0.002:
                self.path.poses[-1] = pose
                self.pub_path.publish(self.path)
                return
        self.path.poses = (self.path.poses + [pose])[-2000:]
        self.pub_path.publish(self.path)

    def on_calibration(self, msg):
        if not msg.evidence.valid or msg.evidence.stale:
            self.calibration = {}; self.cal_received = 0.; return
        self.calibration = {'n': msg.sample_count, 'camera_x': {
            'value_m': msg.camera_x_m if msg.camera_x_known else None,
            'sigma_m': msg.camera_x_sigma_m if msg.camera_x_sigma_known else None,
            'ready': msg.camera_x_ready}}
        for name in ('yaw_positive', 'yaw_negative', 'forward'):
            self.calibration[name] = {'gain': getattr(msg, name+'_gain') if getattr(msg, name+'_known') else None}
        self.cal_received = time.time()

    def on_odom(self, msg):
        if not msg.evidence.valid or msg.evidence.stale:
            self.odom_received=0.
            return
        self.odom['camera_x_m']=msg.active_camera_x_m if msg.active_camera_x_known and math.isfinite(msg.active_camera_x_m) else None
        if msg.validity.value==0:
            self.odom['state']='init'
            self.odom_received=time.time()
            return
        self.odom = {'ok': 0, 'rejected': 0, **self.odom, 'state': 'accepted' if msg.pair_accepted else 'rejected'}
        key = 'ok' if msg.pair_accepted else 'rejected'
        self.odom[key] = self.odom.get(key, 0)+1
        self.odom_received = time.time()

    def on_command(self, msg):
        stamp = msg.outcome.requested_at
        self.commands = {'commands': [{'method': msg.operation,
            'requestedAt': stamp.sec*1000+stamp.nanosec/1e6,
            'dispatch': 'enum:'+str(msg.outcome.dispatch), 'response': 'enum:'+str(msg.outcome.response)}]}
        self.commands_received = time.time()

    def values(self, now):
        return (decode_status(self.state, now), decode_arm(self.arm),
                decode_command(self.commands, now), decode_calibration(self.calibration, self.odom))

    def message(self, now):
        state, arm, command, calibration = self.values(now)
        fresh = bool(self.state_received) and now - self.state_received < 5
        robot_fresh = fresh and state['status_valid'] and math.isfinite(state['status_age_s']) and state['status_age_s'] < 10
        statuses = [
            ('Connection', 0 if fresh and state['connected'] else 3, state['connection_state'],
             {'http_age_s': now-self.state_received if self.state_received else 'unknown', 'connected': state['connected']}),
            ('Robot', 3 if not robot_fresh else 2 if state['error_code'] != 0 else 1 if state['battery'] < 20 else 0,
             state['state_name'] + ' / ' + state['error_name'], state),
            ('Arm estimate', 3 if not self.arm_received or now-self.arm_received >= 5 else 1 if arm['state'] == 'UNKNOWN' else 0,
             arm['state'] + (' — ' + arm['unknown_reason'] if arm['unknown_reason'] else ''), arm),
            ('Last RC command', 3 if not self.commands_received else 1 if command['outcome'] not in ('sent/received', 'none/none', 'enum:1/enum:1', 'enum:0/enum:0') else 0,
             command['method'] + ' / ' + command['outcome'], command),
            ('Calibration', 3 if not self.cal_received else 0 if calibration['camera_x_ready'] else 1,
             'camera offset fit ready' if calibration['camera_x_ready'] else 'camera offset fit not ready', calibration),
            ('Visual odometry', 3 if not self.odom_received or now-self.odom_received >= 5 else 1 if calibration['vo_state'] == 'rejected' else 0,
             calibration['vo_state'], {k: v for k, v in calibration.items() if k.startswith('vo_') or k == 'active_camera_x_m'}),
        ]
        result = DiagnosticArray()
        result.header.stamp = self.node.get_clock().now().to_msg()
        result.header.frame_id = 'base_footprint'
        result.status = [DiagnosticStatus(name='Z70/' + name, hardware_id='Saros Z70', level=bytes([level]),
                         message=message, values=[KeyValue(key=k, value=str(v)) for k,v in values.items()])
                         for name, level, message, values in statuses]
        return result

    def publish(self, now=None):
        now = time.time() if now is None else now
        state, arm, command, calibration = self.values(now)
        for prefix, values, received, ttl in [('/z70/', state, self.state_received, 5),
                        ('/z70/arm/', arm, self.arm_received, 5),
                        ('/z70/last_command/', command, self.commands_received, math.inf),
                        ('/z70/calibration/', calibration, self.cal_received or self.odom_received, math.inf)]:
            stale = not received or now - received >= ttl
            for key, value in values.items():
                topic = prefix + key
                kind = self.types[topic]
                if kind == Float32 and stale: value = math.nan
                if prefix == '/z70/' and stale:
                    if key in ('connected', 'status_valid'): value = False
                    elif key == 'connection_state': value = 'unavailable'
                    elif key in ('cleaning','returning','charging'): continue
                if kind == Bool and value is None: continue
                self.publishers[topic].publish(kind(data=value))
        self.diagnostics.publish(self.message(now))
