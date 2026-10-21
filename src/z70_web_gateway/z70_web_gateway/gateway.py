"""Thin shared ROS client. No transport, acquisition, calibration files or recovery loops."""
from collections import OrderedDict
from copy import deepcopy
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hmac
import json
import os
import re
import socket
import stat
import threading
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict as data
from rosidl_runtime_py.set_message import set_message_fields
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import OccupancyGrid
from unique_identifier_msgs.msg import UUID
from z70_driver.driver import frame_key, urdf_to_app
from z70_interfaces.msg import RobotState, CameraState, ArmEstimate, CalibrationState, FrameMetadata
from z70_interfaces.srv import RawRpc, Stop, SelectCamera, SetArmAnchor, SetControlSession
from z70_driver.commands import LATCHED
from z70_manipulation.goals import ALL_GOALS, GOAL_CONTEXT_PRIMITIVES, simulate_goal
from std_msgs.msg import Bool, String

ACTIONS = {name: (spec.action_cls, spec.topic) for name, spec in ALL_GOALS.items()}
SERVICES = (
    ('rpc', RawRpc, '/z70/driver/raw_rpc'),
    ('stop', Stop, '/z70/stop'),
    ('control/session', SetControlSession, '/z70/control/session'),
    ('camera/select', SelectCamera, '/z70/camera/select'),
    ('arm/anchor', SetArmAnchor, '/z70/arm/anchor'),
)
TOPICS = (
    ('session', Bool, '/z70/control/enabled'),
    ('state', RobotState, '/z70/state'),
    ('camera', CameraState, '/z70/camera/state'),
    ('arm', ArmEstimate, '/z70/arm_estimate'),
    ('calibration', CalibrationState, '/z70/chassis_calibration'),
    ('map', OccupancyGrid, '/map'),
    ('dynamic_obstacles', OccupancyGrid, '/z70/map/dynamic_obstacles'),
    ('world_objects', String, '/z70/world/objects'),
    ('spatial_context', String, '/z70/map/spatial_context'),
)
TERMINAL = {'succeeded', 'failed', 'cancelled', 'rejected'}
ID = r'[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}'


class ApiError(Exception):
    def __init__(self, status, code, dispatch='not_sent', task_id=None):
        self.status = status
        self.body = {'error': code, 'dispatch': dispatch}
        if task_id: self.body['id'] = task_id


def checked(body, allowed, required=()):
    if not isinstance(body, dict) or set(body) - set(allowed) or set(required) - set(body):
        raise ApiError(400, 'INVALID_INPUT')


def integer(value, low, high):
    return type(value) is int and low <= value <= high


def goal_message(kind, payload):
    spec = ALL_GOALS.get(kind)
    if spec is None or not isinstance(payload, dict): raise ApiError(400, 'INVALID_INPUT')
    checked(payload, spec.fields, spec.fields)
    if not spec.validate_payload(payload): raise ApiError(400, 'INVALID_INPUT')
    try:
        message = spec.action_cls.Goal()
        set_message_fields(message, payload)
        return message
    except (TypeError, ValueError, AssertionError, AttributeError, OverflowError):
        raise ApiError(400, 'INVALID_INPUT') from None


def await_future(future, timeout, task_id=None):
    event = threading.Event(); future.add_done_callback(lambda _: event.set())
    if not event.wait(timeout): raise ApiError(504, 'RESPONSE_TIMEOUT', 'unknown', task_id)
    try: return future.result()
    except Exception: raise ApiError(503, 'ROS_UNAVAILABLE', 'unknown', task_id) from None


class Gateway(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('z70_web_gateway', parameter_overrides=parameter_overrides or [])
        self.lock = threading.RLock()
        self.samples = {}; self.frames = {}; self.images = {}; self.metadata = {}
        self.tasks = OrderedDict(); self.handles = {}; self.disappeared = {}
        self.group = ReentrantCallbackGroup()
        self.actions = {k: ActionClient(self, cls, path, callback_group=self.group) for k, (cls, path) in ACTIONS.items()}
        self.ros_clients = {k: self.create_client(cls, path, callback_group=self.group) for k, cls, path in SERVICES}
        self.subscriptions_owned = [
            self.create_subscription(cls, topic, partial(self.sample, key), LATCHED, callback_group=self.group)
            for key, cls, topic in TOPICS
        ]
        for camera in ('front', 'gripper'):
            self.subscriptions_owned += [
                self.create_subscription(CompressedImage, f'/camera/{camera}/image_raw/compressed', partial(self.image, camera), 2, callback_group=self.group),
                self.create_subscription(FrameMetadata, f'/camera/{camera}/metadata', partial(self.meta, camera), 2, callback_group=self.group),
            ]
        self.create_timer(1., self.check_tasks, callback_group=self.group)

    def sample(self, key, message):
        with self.lock: self.samples[key] = data(message)

    def destroy_node(self):
        # Action clients are waitables, not ordinary Node clients. Destroy them
        # before the ROS context so result/feedback timers cannot outlive it.
        for client in self.actions.values(): client.destroy()
        self.actions.clear()
        return super().destroy_node()

    frame_key = staticmethod(frame_key)

    def image(self, camera, message):
        if len(message.data) > 5*1024*1024: return
        with self.lock:
            self.images[camera] = message; self.match(camera)

    def meta(self, camera, message):
        with self.lock:
            self.metadata[camera] = message; self.match(camera)

    def match(self, camera):
        image, meta = self.images.get(camera), self.metadata.get(camera)
        if image is None or meta is None or self.frame_key(image) != self.frame_key(meta): return
        if meta.camera != {'front': 1, 'gripper': 2}[camera]: return
        e = meta.evidence
        if not e.valid or e.stale: return
        # Uncalibrated images are still useful for viewing. Never manufacture metric validity.
        self.frames[camera] = (bytes(image.data), deepcopy(meta))

    def frame(self, body):
        checked(body, ('camera',))
        with self.lock:
            if self.samples.get('session', {}).get('data') is False: raise ApiError(409, 'SESSION_STOPPED')
            current = self.samples.get('camera') or {}
            camera = body.get('camera', {1:'front', 2:'gripper'}.get(current.get('selected_camera')))
            if 'camera' in body and camera not in ('front', 'gripper'): raise ApiError(400, 'INVALID_INPUT')
            frame = self.frames.get(camera)
            if not frame: raise ApiError(503, 'OBSERVATION_GAP')
            image, meta = frame; e = meta.evidence
            age = time.time() - e.received_at.sec - e.received_at.nanosec/1e9
            if not 0 <= age <= 5 or not image.startswith(b'\xff\xd8') or not image.endswith(b'\xff\xd9'):
                raise ApiError(503, 'OBSERVATION_GAP')
            return image, {'X-Roboclaw-Camera': camera, 'X-Roboclaw-Frame-Id': str(e.sequence),
                'X-Roboclaw-Camera-Session': str(uuid.UUID(bytes=bytes(e.session_id.uuid))),
                'X-Roboclaw-Received-At': str(e.received_at.sec*1000+e.received_at.nanosec//1000000)}

    def call(self, key, request, timeout):
        client = self.ros_clients[key]
        if not client.service_is_ready(): raise ApiError(503, 'ROS_UNAVAILABLE')
        return data(await_future(client.call_async(request), timeout))

    def prune(self):
        terminal = [ident for ident, row in self.tasks.items() if row['status'] in TERMINAL]
        for ident in terminal[:-256]:
            self.tasks.pop(ident, None); self.handles.pop(ident, None); self.disappeared.pop(ident, None)

    def task(self, ident):
        with self.lock:
            if ident not in self.tasks: raise ApiError(404, 'TASK_NOT_FOUND')
            return deepcopy(self.tasks[ident])

    def feedback(self, ident, message):
        with self.lock:
            row = self.tasks.get(ident)
            if row and row['status'] not in TERMINAL:
                row.update(status='executing', phase=message.feedback.phase, feedback=data(message.feedback))

    def finished(self, ident, future):
        with self.lock:
            row = self.tasks[ident]
            try:
                reply = future.result()
                status = {4:'succeeded', 5:'cancelled', 6:'failed'}.get(reply.status, 'unknown')
                row.update(status=status, result=data(reply.result))
                if status in TERMINAL: row['phase'] = 5
            except Exception: row['status'] = 'unknown'
            self.prune()

    def accepted(self, ident, future):
        with self.lock:
            row = self.tasks[ident]
            try:
                handle = future.result()
                if not handle.accepted:
                    row['status'] = 'rejected'; self.prune(); return
                self.handles[ident] = handle
                if row['status'] in ('pending', 'unknown'): row['status'] = 'accepted'
                handle.get_result_async().add_done_callback(partial(self.finished, ident))
            except Exception: row['status'] = 'unknown'

    def goal_message(self, body):
        checked(body, ('kind', 'goal'), ('kind', 'goal'))
        kind = body['kind']
        if not isinstance(kind, str) or kind not in ACTIONS: raise ApiError(400, 'INVALID_INPUT')
        return kind, ACTIONS[kind][1], ACTIONS[kind][0], goal_message(kind, body['goal'])

    def validate(self, body):
        try:
            if not isinstance(body, dict): raise ApiError(400, 'INVALID_INPUT')
            simulate = body.get('simulate', False)
            context = body.get('context', {})
            if type(simulate) is not bool or not isinstance(context, dict): raise ApiError(400, 'INVALID_INPUT')
            allowed_ctx = {p['name'] for p in GOAL_CONTEXT_PRIMITIVES}
            if set(context) - allowed_ctx: raise ApiError(400, 'INVALID_INPUT')
            kind, topic, cls, message = self.goal_message({k: v for k, v in body.items() if k not in ('simulate', 'context')})
            res = {'valid': True, 'kind': kind, 'topic': topic, 'action': cls.__name__}
            if simulate:
                live_ctx = {}
                with self.lock:
                    raw_spatial = self.samples.get('spatial_context', {}).get('data')
                    raw_world = self.samples.get('world_objects', {}).get('data')
                    arm_sample = deepcopy(self.samples.get('arm'))
                if isinstance(arm_sample, dict):
                    val = arm_sample.get('validity', {}).get('value')
                    if val in (1, 2):
                        j = urdf_to_app(arm_sample.get('joints', {}).get('name'), arm_sample.get('joints', {}).get('position'))
                        if j:
                            live_ctx['arm'] = {'state': 'IN_PROCESS_POS', 'joints': {**j, 'm6': 90.}}
                    elif val == 3:
                        live_ctx['arm'] = {'state': 'IN_ROBOT', 'joints': {'m3': 0., 'm4': 0., 'm5': 0., 'm6': 0.}}
                for key, raw_json in (('world_objects', raw_world), ('spatial_context', raw_spatial)):
                    if isinstance(raw_json, str):
                        try:
                            live_ctx[key] = json.loads(raw_json)
                        except (ValueError, TypeError):
                            pass
                trace = []
                sim_ok, sim_res = simulate_goal(ALL_GOALS[kind], message, _trace=trace, **{**live_ctx, **context})
                res['simulated'] = {'ok': bool(sim_ok), 'result': data(sim_res), 'trace': trace}
            return 200, res, {}
        except ApiError as e:
            return e.status, {'valid': False, 'error': e.body['error'], **e.body}, {}
        except Exception as e:
            return 400, {'valid': False, 'error': str(e) or 'INVALID_INPUT', 'dispatch': 'not_sent'}, {}

    def start(self, body):
        kind, _, _, message = self.goal_message(body); client = self.actions[kind]
        if not client.server_is_ready(): raise ApiError(503, 'ROS_UNAVAILABLE')
        ident = str(uuid.uuid4())
        with self.lock:
            self.prune()
            if sum(t['status'] not in TERMINAL for t in self.tasks.values()) >= 32: raise ApiError(409, 'CAPACITY')
            self.tasks[ident] = dict(id=ident, kind=kind, status='pending', phase=0, feedback=None, result=None)
        try:
            future = client.send_goal_async(message, goal_uuid=UUID(uuid=list(uuid.UUID(ident).bytes)),
                feedback_callback=partial(self.feedback, ident))
            future.add_done_callback(partial(self.accepted, ident))
            handle = await_future(future, 3, ident)
            if not handle.accepted: raise ApiError(409, 'REJECTED', task_id=ident)
            return self.task(ident)
        except ApiError:
            with self.lock:
                if self.tasks[ident]['status'] == 'pending': self.tasks[ident]['status'] = 'unknown'
            raise
        except Exception:
            with self.lock: self.tasks[ident]['status'] = 'unknown'
            raise ApiError(503, 'ROS_UNAVAILABLE', 'unknown', ident) from None

    def cancel(self, ident):
        with self.lock:
            row = self.task(ident); handle = self.handles.get(ident)
            if row['status'] in TERMINAL: raise ApiError(409, 'TASK_FINISHED')
            if handle is None: raise ApiError(503, 'ROS_UNAVAILABLE', 'unknown', ident)
        reply = await_future(handle.cancel_goal_async(), 3, ident)
        if not reply.goals_canceling: raise ApiError(409, 'CANCEL_REJECTED', 'unknown', ident)
        return {'id': ident, 'cancel_requested': True}

    def check_tasks(self):
        with self.lock:
            for ident, row in self.tasks.items():
                if row['status'] in TERMINAL: continue
                if self.actions[row['kind']].server_is_ready(): self.disappeared.pop(ident, None)
                elif time.monotonic() - self.disappeared.setdefault(ident, time.monotonic()) > 5:
                    row['status'] = 'unknown'  # No replay or false terminal completion on lost server.

    def route(self, method, path, body):
        if method == 'GET' and path == '/api/ros/state':
            with self.lock:
                return 200, dict(available=self.ros_clients['stop'].service_is_ready(),
                    session_enabled=self.samples.get('session', {}).get('data'),
                    **{k: deepcopy(self.samples.get(k)) for k in ('state', 'camera', 'arm', 'calibration')}), {}
        if method == 'GET' and path == '/api/ros/map':
            with self.lock:
                res = {'map': deepcopy(self.samples.get('map'))}
                if 'dynamic_obstacles' in self.samples:
                    res['dynamic_obstacles'] = deepcopy(self.samples['dynamic_obstacles'])
                for key in ('world_objects', 'spatial_context'):
                    raw_json = self.samples.get(key, {}).get('data')
                    if isinstance(raw_json, str):
                        try:
                            res[key] = json.loads(raw_json)
                        except (ValueError, TypeError):
                            pass
                return 200, res, {}
        if method == 'GET' and path == '/api/ros/goals':
            return 200, {
                'goals': [spec.describe() for spec in ALL_GOALS.values()],
                'context_primitives': list(GOAL_CONTEXT_PRIMITIVES),
            }, {}
        if method == 'POST' and path == '/api/ros/goals/validate':
            return self.validate(body)
        if path == '/api/ros/tasks':
            if method == 'GET':
                with self.lock: return 200, {'tasks':deepcopy(list(self.tasks.values()))}, {}
            if method == 'POST': return 202, self.start(body), {}
        match = re.fullmatch('/api/ros/tasks/('+ID+')(/cancel)?', path)
        if match:
            ident, cancel = match.groups()
            if method == 'GET' and not cancel: return 200, self.task(ident), {}
            if method == 'POST' and cancel:
                checked(body, ()); return 200, self.cancel(ident), {}
        if method != 'POST': raise ApiError(405, 'METHOD_NOT_ALLOWED')
        if path == '/api/ros/frame':
            image, headers = self.frame(body); return 200, image, headers
        if path == '/api/ros/control/session':
            checked(body, ('enabled',), ('enabled',))
            if type(body['enabled']) is not bool: raise ApiError(400, 'INVALID_INPUT')
            result = self.call('control/session', SetControlSession.Request(enabled=body['enabled']), 34)
            if not result['enabled']:
                with self.lock:
                    self.samples['session'] = {'data':False}
                    self.frames.clear(); self.images.clear(); self.metadata.clear()
            return 200, result, {}
        if path == '/api/ros/stop':
            checked(body, ('scope',)); scope = body.get('scope', 3)
            if not integer(scope, 1, 3): raise ApiError(400, 'INVALID_INPUT')
            return 200, self.call('stop', Stop.Request(scope=scope), 15.5), {}
        if path == '/api/ros/camera/select':
            checked(body, ('camera',), ('camera',))
            if body['camera'] not in ('front', 'gripper'): raise ApiError(400, 'INVALID_INPUT')
            return 200, self.call('camera/select', SelectCamera.Request(camera={'front':1,'gripper':2}[body['camera']]), 20.5), {}
        if path == '/api/ros/arm/anchor':
            checked(body, ('anchor', 'reason'), ('anchor',)); reason = body.get('reason', '')
            if body['anchor'] not in ('unknown', 'stowed', 'deployed') or not isinstance(reason, str) or len(reason) > 64:
                raise ApiError(400, 'INVALID_INPUT')
            # Reason is an enum-like observation label, not free text with accidental secrets.
            if reason and not re.fullmatch(r'[A-Za-z0-9 _.-]{1,64}', reason): raise ApiError(400, 'INVALID_INPUT')
            req = SetArmAnchor.Request(anchor={'unknown':0,'stowed':1,'deployed':2}[body['anchor']], observation_reason=reason)
            return 200, self.call('arm/anchor', req, 3), {}
        if path == '/api/ros/rpc':
            checked(body, ('method', 'params', 'timeoutMs', 'transport', 'takeover'), ('method', 'params'))
            method_name = body['method']; timeout = body.get('timeoutMs', 12000)
            if (not isinstance(method_name, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]{0,127}', method_name)
                    or not integer(timeout, 1, 30000) or body.get('transport', 'local') not in ('local', 'cloud')
                    or type(body.get('takeover', False)) is not bool): raise ApiError(400, 'INVALID_INPUT')
            params = json.dumps(body['params'], allow_nan=False, separators=(',', ':'))
            if len(params.encode()) > 16384: raise ApiError(400, 'INVALID_INPUT')
            req = RawRpc.Request(method=method_name, params_json=params, timeout_ms=timeout,
                transport=0 if body.get('transport', 'local') == 'local' else 1, takeover=body.get('takeover', False))
            return 200, self.call('rpc', req, timeout/1000+16), {}
        raise ApiError(404, 'NOT_FOUND')


class HttpServer(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False
    def __init__(self, address, gateway, token):
        self.gateway, self.token = gateway, token
        self.slots = threading.BoundedSemaphore(40)
        super().__init__(address, Handler)

    def process_request(self, request, client_address):
        if not self.slots.acquire(False):
            self.shutdown_request(request); return
        try: super().process_request(request, client_address)
        except Exception:
            self.slots.release(); raise

    def process_request_thread(self, request, client_address):
        try: super().process_request_thread(request, client_address)
        finally: self.slots.release()

    def handle_error(self, request, client_address): pass  # No URLs, payloads or credentials in logs.


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args): pass

    def do_GET(self): self.handle_api()
    def do_POST(self): self.handle_api()

    def handle_api(self):
        self.connection.settimeout(5)
        try:
            if self.headers.get('Host') != f'127.0.0.1:{self.server.server_port}': raise ApiError(403, 'INVALID_HOST')
            if self.headers.get('Origin') is not None: raise ApiError(403, 'BROWSER_DIRECT_ACCESS_DENIED')
            if not hmac.compare_digest(self.headers.get('Authorization', ''), 'Bearer '+self.server.token): raise ApiError(401, 'UNAUTHORIZED')
            if not self.path.startswith('/api/ros/') or '?' in self.path or '%' in self.path: raise ApiError(404, 'NOT_FOUND')
            body = {}
            if self.command == 'POST':
                if self.headers.get('Transfer-Encoding') or self.headers.get('Content-Encoding', 'identity') != 'identity': raise ApiError(400, 'INVALID_INPUT')
                if self.headers.get('Content-Type', '').split(';')[0] != 'application/json': raise ApiError(415, 'JSON_REQUIRED')
                length = self.headers.get('Content-Length', '')
                if not length.isdigit() or not 1 <= int(length) <= 192*1024: raise ApiError(413, 'BODY_LIMIT')
                body = json.loads(self.rfile.read(int(length)), parse_constant=lambda _: (_ for _ in ()).throw(ValueError()))
                if not isinstance(body, dict): raise ApiError(400, 'INVALID_INPUT')
            status_code, value, headers = self.server.gateway.route(self.command, self.path, body)
        except ApiError as error: status_code, value, headers = error.status, error.body, {}
        except (ValueError, TypeError, AssertionError): status_code, value, headers = 400, ApiError(400, 'INVALID_INPUT').body, {}
        except socket.timeout: status_code, value, headers = 408, ApiError(408, 'BODY_TIMEOUT').body, {}
        except Exception: status_code, value, headers = 503, ApiError(503, 'ROS_UNAVAILABLE', 'unknown').body, {}
        try:
            binary = isinstance(value, bytes)
            content = value if binary else json.dumps(value, allow_nan=False).encode()
            self.send_response(status_code)
            for key, val in headers.items(): self.send_header(key, val)
            self.send_header('Content-Type', 'image/jpeg' if binary else 'application/json')
            self.send_header('Content-Length', str(len(content))); self.send_header('Cache-Control', 'no-store')
            self.end_headers(); self.wfile.write(content)
        except (OSError, ValueError): pass


def read_token(path):
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_mode & 0o077 or info.st_size != 43:
            raise ValueError('Unsafe token file')
        token = os.read(fd, 44).decode('ascii')
        if not re.fullmatch(r'[A-Za-z0-9_-]{43}', token): raise ValueError('Invalid token file')
        return token
    finally: os.close(fd)


def main():
    rclpy.init()
    node = Gateway()
    token_file = node.declare_parameter('token_file', '').value
    port = node.declare_parameter('port', 57020).value
    server = None; thread = None; failed = False
    executor = MultiThreadedExecutor(num_threads=6)
    try:
        if not integer(port, 57000, 57100): raise ValueError('Invalid port')
        server = HttpServer(('127.0.0.1', port), node, read_token(token_file))
        thread = threading.Thread(target=server.serve_forever); thread.start()
        executor.add_node(node); executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    except Exception:
        failed = True
        node.get_logger().error('Gateway unavailable: check port and server-side token_file parameter.')
    finally:
        if server: server.shutdown(); server.server_close()
        if thread: thread.join(3)
        executor.shutdown(timeout_sec=3); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    if failed: raise SystemExit(1)
