"""SIGINT must revoke active work and drain publishers before the context is destroyed."""
from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from unique_identifier_msgs.msg import UUID

from z70_interfaces.action import BasePulse, Pick
from z70_interfaces.msg import FrameMetadata
from z70_interfaces.srv import SetControlSession, Stop, TaskSession

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
USER_ARM_ESTIMATE = Path.home() / '.local/share/RRsZ70_ROS2/arm-estimate.json'


def _env(tmp_path: Path, domain: int) -> dict[str, str]:
    env = dict(os.environ)
    env.pop('Z70_REPO', None)
    env.update(
        ROS_DOMAIN_ID=str(domain),
        ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
        ROS_LOCALHOST_ONLY='1',
        ROS_LOG_DIR=str(tmp_path / 'ros-log'),
        XDG_DATA_HOME=str(tmp_path / 'xdg'),
        Z70_REPO=str(tmp_path),
    )
    return env


def _jpeg() -> bytes:
    image = np.zeros((48, 64, 3), np.uint8)
    image[:, :, 1] = 180
    ok, encoded = cv2.imencode('.jpg', image)
    assert ok
    return encoded.tobytes()


def _user_arm_stamp():
    if not USER_ARM_ESTIMATE.exists():
        return None
    info = USER_ARM_ESTIMATE.stat()
    return (info.st_mtime_ns, info.st_size)


def _assert_user_arm_untouched(before) -> None:
    assert _user_arm_stamp() == before


def _wait_action(executor, client, timeout: float, log_path: Path) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not client.server_is_ready():
        executor.spin_once(timeout_sec=0.05)
    assert client.server_is_ready(), log_path.read_text()


def _assert_estimate_parameter(executor, node, expected: Path, output: str) -> None:
    client = AsyncParameterClient(node, 'z70_driver')
    assert client.wait_for_services(timeout_sec=10), output
    future = client.get_parameters(['arm_estimate_path'])
    executor.spin_until_future_complete(future, timeout_sec=5)
    assert future.done(), output
    configured = future.result().values[0].string_value
    assert configured == str(expected), configured
    assert Path(configured) != USER_ARM_ESTIMATE


def _assert_clean(process: subprocess.Popen, output: str) -> None:
    assert process.returncode == 0, output
    assert 'context is invalid' not in output
    assert 'RCLError' not in output
    assert 'process has died' not in output
    assert 'Fatal Python error' not in output
    assert 'TypeError' not in output


def _reap(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.kill()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass


class _RecordingHTTPServer(ThreadingHTTPServer):
    errors: list[str] = []

    def handle_error(self, request, client_address):
        import traceback
        type(self).errors.append(traceback.format_exc())


def _start_http(handler) -> tuple[_RecordingHTTPServer, threading.Thread, int]:
    _RecordingHTTPServer.errors = []
    server = _RecordingHTTPServer(('127.0.0.1', 0), handler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, name='shutdown-http', daemon=True)
    thread.start()
    return server, thread, server.server_address[1]


def _stop_http(server: ThreadingHTTPServer | None, thread: threading.Thread | None) -> None:
    if server is not None:
        server.shutdown()
        server.server_close()
    if thread is not None:
        thread.join(timeout=5)


class _JsonHandler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def _body(self) -> bytes:
        length = int(self.headers.get('Content-Length', '0') or 0)
        return self.rfile.read(length) if length else b''

    def _send(self, body: bytes, content_type: str, extra: dict[str, str] | None = None) -> None:
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        for key, value in (extra or {}).items():
            self.send_header(key, value)
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_args):
        return


def test_driver_sigint_drains_camera_publish_before_context_shutdown(tmp_path: Path):
    before = _user_arm_stamp()
    jpeg = _jpeg()
    seen = threading.Event()
    estimate_path = tmp_path / 'arm-estimate.json'

    class Handler(_JsonHandler):
        def do_GET(self):
            self._body()
            if self.path.startswith('/api/commands/events'):
                self.close_connection = True
                self._send(b'', 'text/plain')
                return
            self._send(b'{"telemetry":{},"connection":{"state":"disconnected"}}', 'application/json')

        def do_POST(self):
            self._body()
            if self.path.startswith('/api/camera/frame'):
                seen.set()
                self._send(jpeg, 'image/jpeg', {'X-Roboclaw-Camera': 'front'})
                return
            self._send(b'{}', 'application/json')

    server, thread, port = _start_http(Handler)
    log_path = tmp_path / 'driver-shutdown.log'
    log = log_path.open('w')
    process = subprocess.Popen(
        [sys.executable, str(Path(__file__).resolve()), 'driver', str(port), str(estimate_path)],
        cwd=tmp_path,
        env=_env(tmp_path, 95),
        stdout=log,
        stderr=subprocess.STDOUT,
    )
    context = Context()
    node = None
    executor = None
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and not seen.is_set():
            assert process.poll() is None, log_path.read_text()
            time.sleep(0.05)
        assert seen.is_set(), log_path.read_text()
        rclpy.init(context=context, domain_id=95)
        node = rclpy.create_node('shutdown_driver_parameters', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        _assert_estimate_parameter(executor, node, estimate_path, log_path.read_text())
        process.send_signal(signal.SIGINT)
        process.wait(timeout=15)
        _assert_clean(process, log_path.read_text())
        assert not _RecordingHTTPServer.errors, _RecordingHTTPServer.errors
    finally:
        _reap(process)
        _stop_http(server, thread)
        log.close()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
        _assert_user_arm_untouched(before)


def test_perception_sigint_exits_without_segfault_while_publishing(tmp_path: Path):
    log_path = tmp_path / 'perception-shutdown.log'
    log = log_path.open('w')
    process = subprocess.Popen(
        [sys.executable, str(Path(__file__).resolve()), 'perception'],
        cwd=tmp_path,
        env=_env(tmp_path, 96),
        stdout=log,
        stderr=subprocess.STDOUT,
    )
    context = Context()
    node = None
    executor = None
    try:
        rclpy.init(context=context, domain_id=96)
        node = rclpy.create_node('shutdown_frame_source', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        published = []
        node.create_subscription(String, '/z70/objects/json', published.append, LATCHED)
        image_pub = node.create_publisher(CompressedImage, '/camera/front/image_raw/compressed', 2)
        meta_pub = node.create_publisher(FrameMetadata, '/camera/front/metadata', 2)
        jpeg = _jpeg()
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline and not published:
            assert process.poll() is None, log_path.read_text()
            now = time.time()
            image = CompressedImage()
            image.header.stamp.sec = int(now)
            image.header.stamp.nanosec = int((now % 1) * 1e9)
            image.header.frame_id = 'camera_optical_frame'
            image.format = 'jpeg'
            image.data = jpeg
            meta = FrameMetadata()
            meta.header = image.header
            meta.camera = 1
            meta.width = 64
            meta.height = 48
            meta.calibration_id = 'shutdown-test'
            meta.evidence.valid = True
            meta.evidence.stale = False
            meta.evidence.received_at.sec = image.header.stamp.sec
            meta.evidence.received_at.nanosec = image.header.stamp.nanosec
            meta.evidence.sequence = int(now * 1000) % 100000
            image_pub.publish(image)
            meta_pub.publish(meta)
            executor.spin_once(timeout_sec=0.05)
        assert published, log_path.read_text()
        process.send_signal(signal.SIGINT)
        process.wait(timeout=15)
        _assert_clean(process, log_path.read_text())
    finally:
        _reap(process)
        log.close()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()


def test_sigint_revokes_active_pick_and_inflight_driver_command(tmp_path: Path):
    before = _user_arm_stamp()
    estimate_path = tmp_path / 'command-arm-estimate.json'
    held = threading.Event()
    release = threading.Event()
    methods = []

    class Handler(_JsonHandler):
        def do_GET(self):
            self._body()
            if self.path.startswith('/api/commands/events'):
                self.close_connection = True
                self._send(b'', 'text/plain')
                return
            self._send(b'{"telemetry":{},"connection":{"state":"disconnected"}}', 'application/json')

        def do_POST(self):
            raw = self._body()
            if self.path.startswith('/api/driver/rpc'):
                method = ''
                try:
                    method = json.loads(raw.decode() or '{}').get('method', '')
                except (UnicodeError, json.JSONDecodeError):
                    method = ''
                methods.append(method)
                if method == 'app_rc_move':
                    held.set()
                    release.wait(timeout=20)
                self._send(
                    b'{"commandId":"00000000-0000-0000-0000-000000000001",'
                    b'"attemptId":"00000000-0000-0000-0000-000000000002","attemptIndex":0,'
                    b'"dispatch":"sent","response":"received","requestedAt":1,"finishedAt":2}',
                    'application/json',
                )
                return
            self._send(b'{}', 'application/json')

    server, thread, port = _start_http(Handler)
    driver_log = tmp_path / 'driver-command.log'
    pick_log = tmp_path / 'pick-shutdown.log'
    driver_out = driver_log.open('w')
    pick_out = pick_log.open('w')
    driver = subprocess.Popen(
        [sys.executable, str(Path(__file__).resolve()), 'driver', str(port), str(estimate_path)],
        cwd=tmp_path,
        env=_env(tmp_path, 94),
        stdout=driver_out,
        stderr=subprocess.STDOUT,
    )
    sessions = []
    stops = []

    def on_session(request, response):
        sessions.append(bool(request.begin))
        response.accepted = True
        response.task_id = UUID(uuid=list(uuid.uuid4().bytes))
        response.reason = ''
        return response

    def on_stop(request, response):
        stops.append(int(request.scope))
        return response

    context = Context()
    node = None
    executor = None
    pick = None
    pick_context = Context()
    pick_node = None
    pick_executor = None
    try:
        rclpy.init(context=context, domain_id=94)
        node = rclpy.create_node('shutdown_command_probe', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        pulse = ActionClient(node, BasePulse, '/z70/base/pulse')
        _wait_action(executor, pulse, 15, driver_log)
        goal = BasePulse.Goal()
        goal.linear_mps = 0.05
        goal.angular_radps = 0.0
        goal.duration.sec = 0
        goal.duration.nanosec = 800_000_000
        send = pulse.send_goal_async(goal)
        executor.spin_until_future_complete(send, timeout_sec=10)
        assert send.done() and send.result().accepted, driver_log.read_text()
        assert held.wait(timeout=10), driver_log.read_text()
        _assert_estimate_parameter(executor, node, estimate_path, driver_log.read_text())
        driver.send_signal(signal.SIGINT)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and 'app_rc_stop' not in methods:
            time.sleep(0.05)
        assert 'app_rc_stop' in methods, methods
        assert estimate_path.is_file(), methods
        release.set()
        driver.wait(timeout=12)
        _assert_clean(driver, driver_log.read_text())
        assert not _RecordingHTTPServer.errors, _RecordingHTTPServer.errors
        if executor is not None:
            executor.shutdown()
            executor = None
        if node is not None:
            node.destroy_node()
            node = None
        if context.ok():
            context.shutdown()

        pick = subprocess.Popen(
            [sys.executable, str(Path(__file__).resolve()), 'active-pick'],
            cwd=tmp_path,
            env=_env(tmp_path, 97),
            stdout=pick_out,
            stderr=subprocess.STDOUT,
        )
        rclpy.init(context=pick_context, domain_id=97)
        pick_node = rclpy.create_node('shutdown_pick_probe', context=pick_context)
        pick_node.create_service(TaskSession, '/z70/tasks/session', on_session)
        pick_node.create_service(Stop, '/z70/stop', on_stop)
        pick_executor = SingleThreadedExecutor(context=pick_context)
        pick_executor.add_node(pick_node)
        pick_client = ActionClient(pick_node, Pick, '/z70/pick')
        _wait_action(pick_executor, pick_client, 15, pick_log)
        pick_goal = pick_client.send_goal_async(Pick.Goal(color='yellow', dry_run=False))
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not pick_goal.done():
            pick_executor.spin_once(timeout_sec=0.05)
        assert pick_goal.done() and pick_goal.result().accepted, pick_log.read_text()
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and True not in sessions:
            pick_executor.spin_once(timeout_sec=0.05)
        assert True in sessions, pick_log.read_text()
        time.sleep(0.3)
        started = time.monotonic()
        pick.send_signal(signal.SIGINT)
        while time.monotonic() < started + 10 and (not stops or False not in sessions):
            pick_executor.spin_once(timeout_sec=0.05)
        pick.wait(timeout=12)
        elapsed = time.monotonic() - started
        _assert_clean(pick, pick_log.read_text())
        assert elapsed < 12, elapsed
        assert stops, pick_log.read_text()
        assert False in sessions, sessions
    finally:
        release.set()
        _reap(driver)
        _reap(pick)
        _stop_http(server, thread)
        driver_out.close()
        pick_out.close()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
        if pick_executor is not None:
            pick_executor.shutdown()
        if pick_node is not None:
            pick_node.destroy_node()
        if pick_context.ok():
            pick_context.shutdown()
        _assert_user_arm_untouched(before)


def test_http_server_records_handler_errors_on_the_server():
    class Handler(_JsonHandler):
        def do_GET(self):
            raise RuntimeError('handler boom')

    server, thread, port = _start_http(Handler)
    try:
        import urllib.request
        try:
            urllib.request.urlopen(f'http://127.0.0.1:{port}/', timeout=2)
        except Exception:
            pass
        assert _RecordingHTTPServer.errors
        assert 'handler boom' in _RecordingHTTPServer.errors[-1]
    finally:
        _stop_http(server, thread)


def test_close_admission_rejects_new_work_and_keeps_stop_and_end():
    from z70_driver.commands import Dispatcher

    stopped = []
    dispatcher = Dispatcher(lambda scope: stopped.append(scope) or [scope])
    accepted, token, reason = dispatcher.begin()
    assert accepted and reason == 'NONE'
    assert dispatcher.reserve(token) is not None
    dispatcher.close_admission()
    assert dispatcher.reserve(token) is None
    refused, _, why = dispatcher.begin()
    assert refused is False and why == 'SESSION_STOPPED'
    assert dispatcher.end(token) is True
    assert stopped == [3]
    assert dispatcher.stop(1) == [1]


def _pulse_goal():
    goal = BasePulse.Goal()
    goal.linear_mps = 0.05
    goal.angular_radps = 0.0
    goal.duration.sec = 0
    goal.duration.nanosec = 800_000_000
    return goal


def _held_camera_handler(release: threading.Event, paths: list, methods: list):
    class Handler(_JsonHandler):
        def do_GET(self):
            self._body()
            paths.append(self.path.split('?', 1)[0])
            if self.path.startswith('/api/commands/events'):
                self.close_connection = True
                self._send(b'', 'text/plain')
                return
            self._send(b'{"telemetry":{},"connection":{"state":"disconnected"}}', 'application/json')

        def do_POST(self):
            raw = self._body()
            paths.append(self.path.split('?', 1)[0])
            if self.path.startswith('/api/camera/frame'):
                release.wait(timeout=30)
                self._send(b'', 'image/jpeg', {'X-Roboclaw-Camera': 'front'})
                return
            if self.path.startswith('/api/driver/rpc'):
                try:
                    methods.append(json.loads(raw.decode() or '{}').get('method', ''))
                except (UnicodeError, json.JSONDecodeError):
                    methods.append('')
            self._send(b'{}', 'application/json')

    return Handler


def _start_held_driver(tmp_path: Path, domain: int, release: threading.Event):
    before = _user_arm_stamp()
    estimate_path = tmp_path / f'arm-{domain}.json'
    paths: list[str] = []
    methods: list[str] = []
    server, thread, port = _start_http(_held_camera_handler(release, paths, methods))
    log_path = tmp_path / f'driver-{domain}.log'
    log = log_path.open('w')
    process = subprocess.Popen(
        [sys.executable, str(Path(__file__).resolve()), 'driver', str(port), str(estimate_path)],
        cwd=tmp_path,
        env=_env(tmp_path, domain),
        stdout=log,
        stderr=subprocess.STDOUT,
    )
    return before, estimate_path, paths, methods, server, thread, log_path, log, process


def _wait_service(executor, client, timeout: float, output: str = '') -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not client.service_is_ready():
        executor.spin_once(timeout_sec=0.05)
    assert client.service_is_ready(), output


def _wait_log(process: subprocess.Popen, log_path: Path, text: str, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and text not in log_path.read_text():
        assert process.poll() is None, log_path.read_text()
        time.sleep(0.05)
    assert text in log_path.read_text(), log_path.read_text()


def _assert_no_crash(output: str) -> None:
    assert 'context is invalid' not in output
    assert 'RCLError' not in output
    assert 'process has died' not in output
    assert 'Fatal Python error' not in output
    assert 'TypeError' not in output


def test_quiescence_rejects_new_motion_before_it_reaches_the_wire(tmp_path: Path):
    release = threading.Event()
    before, _estimate, paths, methods, server, thread, log_path, log, process = _start_held_driver(
        tmp_path, 93, release)
    context = Context()
    node = None
    executor = None
    try:
        rclpy.init(context=context, domain_id=93)
        node = rclpy.create_node('shutdown_admission_probe', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        pulse = ActionClient(node, BasePulse, '/z70/base/pulse')
        _wait_action(executor, pulse, 15, log_path)
        process.send_signal(signal.SIGINT)
        _wait_log(process, log_path, 'shutdown admission closed', 8)
        send = pulse.send_goal_async(_pulse_goal())
        executor.spin_until_future_complete(send, timeout_sec=5)
        assert send.done() and not send.result().accepted, log_path.read_text()
        session = node.create_client(TaskSession, '/z70/tasks/session')
        _wait_service(executor, session, 5, log_path.read_text())
        begin = session.call_async(TaskSession.Request(begin=True))
        executor.spin_until_future_complete(begin, timeout_sec=5)
        assert begin.done() and not begin.result().accepted, log_path.read_text()
        assert begin.result().reason == 'SESSION_STOPPED'
        control = node.create_client(SetControlSession, '/z70/control/session')
        _wait_service(executor, control, 5, log_path.read_text())
        resume = control.call_async(SetControlSession.Request(enabled=True))
        executor.spin_until_future_complete(resume, timeout_sec=5)
        assert resume.done() and resume.result().enabled is False
        assert methods == [], methods
        assert '/api/driver/rpc' not in paths
        assert '/api/driver/control/session' not in paths
        release.set()
        process.wait(timeout=10)
        _assert_clean(process, log_path.read_text())
        assert not _RecordingHTTPServer.errors, _RecordingHTTPServer.errors
    finally:
        release.set()
        _reap(process)
        _stop_http(server, thread)
        log.close()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
        _assert_user_arm_untouched(before)


def test_blocked_publisher_exits_nonzero_without_destroying_under_it(tmp_path: Path):
    release = threading.Event()
    before, _estimate, _paths, methods, server, thread, log_path, log, process = _start_held_driver(
        tmp_path, 92, release)
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and '/api/camera/frame' not in _paths:
            assert process.poll() is None, log_path.read_text()
            time.sleep(0.05)
        assert '/api/camera/frame' in _paths, log_path.read_text()
        started = time.monotonic()
        process.send_signal(signal.SIGINT)
        process.wait(timeout=14)
        elapsed = time.monotonic() - started
        output = log_path.read_text()
        assert process.returncode == 1, output
        assert 'shutdown drain incomplete' in output
        assert elapsed < 14, elapsed
        assert not release.is_set()
        assert methods == []
        _assert_no_crash(output)
    finally:
        release.set()
        _reap(process)
        _stop_http(server, thread)
        log.close()
        _assert_user_arm_untouched(before)


def test_second_signal_bounds_blocked_publisher_drain(tmp_path: Path):
    release = threading.Event()
    before, _estimate, _paths, methods, server, thread, log_path, log, process = _start_held_driver(
        tmp_path, 91, release)
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and '/api/camera/frame' not in _paths:
            assert process.poll() is None, log_path.read_text()
            time.sleep(0.05)
        assert '/api/camera/frame' in _paths, log_path.read_text()
        process.send_signal(signal.SIGINT)
        _wait_log(process, log_path, 'shutdown admission closed', 8)
        started = time.monotonic()
        process.send_signal(signal.SIGINT)
        process.wait(timeout=6)
        elapsed = time.monotonic() - started
        output = log_path.read_text()
        assert process.returncode == 1, output
        assert 'shutdown drain incomplete' in output
        assert elapsed < 4, elapsed
        assert not release.is_set()
        assert methods == []
        _assert_no_crash(output)
    finally:
        release.set()
        _reap(process)
        _stop_http(server, thread)
        log.close()
        _assert_user_arm_untouched(before)


def test_sigint_rejects_a_new_pick_while_stop_is_still_draining(tmp_path: Path):
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor

    end_release = threading.Event()
    sessions = []
    stops = []

    def on_session(request, response):
        sessions.append(bool(request.begin))
        response.accepted = True
        response.task_id = UUID(uuid=list(uuid.uuid4().bytes))
        response.reason = ''
        if not request.begin:
            end_release.wait(timeout=6)
        return response

    def on_stop(request, response):
        stops.append(int(request.scope))
        return response

    log_path = tmp_path / 'pick-admission.log'
    log = log_path.open('w')
    process = subprocess.Popen(
        [sys.executable, str(Path(__file__).resolve()), 'active-pick'],
        cwd=tmp_path,
        env=_env(tmp_path, 97),
        stdout=log,
        stderr=subprocess.STDOUT,
    )
    context = Context()
    node = None
    executor = None
    spin = None
    try:
        rclpy.init(context=context, domain_id=97)
        node = rclpy.create_node('shutdown_pick_admission', context=context)
        # Session end stays blocked while the client receives the next goal's rejection.
        group = ReentrantCallbackGroup()
        node.create_service(TaskSession, '/z70/tasks/session', on_session, callback_group=group)
        node.create_service(Stop, '/z70/stop', on_stop, callback_group=group)
        executor = MultiThreadedExecutor(num_threads=4, context=context)
        executor.add_node(node)
        spin = threading.Thread(target=executor.spin, name='pick-admission-spin', daemon=True)
        spin.start()
        client = ActionClient(node, Pick, '/z70/pick', callback_group=group)
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and not client.server_is_ready():
            assert process.poll() is None, log_path.read_text()
            time.sleep(0.05)
        assert client.server_is_ready(), log_path.read_text()
        first = client.send_goal_async(Pick.Goal(color='yellow', dry_run=False))
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not first.done():
            time.sleep(0.05)
        assert first.done() and first.result().accepted, log_path.read_text()
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and True not in sessions:
            time.sleep(0.05)
        assert True in sessions, log_path.read_text()
        process.send_signal(signal.SIGINT)
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline and not stops:
            time.sleep(0.05)
        assert stops, log_path.read_text()
        second = client.send_goal_async(Pick.Goal(color='yellow', dry_run=False))
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and not second.done():
            assert process.poll() is None, log_path.read_text()
            time.sleep(0.05)
        assert second.done() and not second.result().accepted, log_path.read_text()
        assert sessions.count(True) == 1, sessions
        end_release.set()
        process.wait(timeout=12)
        _assert_clean(process, log_path.read_text())
        assert False in sessions, sessions
    finally:
        end_release.set()
        _reap(process)
        log.close()
        if executor is not None:
            executor.shutdown(wait_for_threads=True)
        if spin is not None:
            spin.join(timeout=2)
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()


def _child_driver(port: str, estimate_path: str) -> None:
    sys.argv = [
        'driver', '--ros-args',
        '-p', f'origin:=http://127.0.0.1:{port}',
        '-p', 'camera_hz:=8.0',
        '-p', 'map_refresh_s:=0.0',
        '-p', 'state_hz:=2.0',
        '-p', f'arm_estimate_path:={estimate_path}',
    ]
    from z70_driver.driver import main
    main()


def _child_perception() -> None:
    from z70_manipulation.node import perception_main
    perception_main()


def _child_active_pick() -> None:
    from z70_manipulation.goals import BUILTIN_GOALS
    from z70_interfaces.action import Pick as PickAction

    def run(ctx, _req):
        ctx.sleep(45)
        return False, PickAction.Result()

    BUILTIN_GOALS['pick'].run = run
    BUILTIN_GOALS['pick'].timeout_s = 60.0
    from z70_manipulation.node import server_main
    server_main()


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'driver':
        _child_driver(sys.argv[2], sys.argv[3])
    elif len(sys.argv) > 1 and sys.argv[1] == 'perception':
        _child_perception()
    elif len(sys.argv) > 1 and sys.argv[1] == 'active-pick':
        _child_active_pick()
    else:
        raise SystemExit(pytest.main([__file__, *sys.argv[1:]]))
