"""SIGINT must drain publishers before the ROS context is invalidated."""
from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from z70_interfaces.msg import FrameMetadata

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
DOMAIN = 95


def _env(tmp_path: Path) -> dict[str, str]:
    env = dict(os.environ)
    env.update(
        ROS_DOMAIN_ID=str(DOMAIN),
        ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
        ROS_LOCALHOST_ONLY='1',
        ROS_LOG_DIR=str(tmp_path / 'ros-log'),
    )
    return env


def _jpeg() -> bytes:
    image = np.zeros((48, 64, 3), np.uint8)
    image[:, :, 1] = 180
    ok, encoded = cv2.imencode('.jpg', image)
    assert ok
    return encoded.tobytes()


def _assert_clean(process: subprocess.Popen, output: str) -> None:
    assert process.returncode == 0, output
    assert process.returncode != -11
    assert 'context is invalid' not in output
    assert 'RCLError' not in output
    assert 'process has died' not in output
    assert 'Fatal Python error' not in output


def test_driver_sigint_drains_camera_publish_before_context_shutdown(tmp_path: Path):
    jpeg = _jpeg()
    seen = threading.Event()

    class Handler(BaseHTTPRequestHandler):
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

        def do_GET(self):
            self._body()
            if self.path.startswith('/api/commands/events'):
                self.close_connection = True
                self._send(b'')
                return
            self._send(b'{"telemetry":{},"connection":{"state":"disconnected"}}')

        def do_POST(self):
            self._body()
            if self.path.startswith('/api/camera/frame'):
                seen.set()
                self._send(jpeg, 'image/jpeg', {'X-Roboclaw-Camera': 'front'})
                return
            self._send(b'{}')

        def log_message(self, *_args):
            return

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    port = server.server_address[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()
    log = tmp_path / 'driver-shutdown.log'
    process = subprocess.Popen(
        [sys.executable, str(Path(__file__).resolve()), 'driver', str(port)],
        cwd=tmp_path,
        env=_env(tmp_path),
        stdout=log.open('w'),
        stderr=subprocess.STDOUT,
    )
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and not seen.is_set():
            assert process.poll() is None, log.read_text()
            time.sleep(0.05)
        assert seen.is_set(), log.read_text()
        process.send_signal(signal.SIGINT)
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)
        raise AssertionError('driver did not finish bounded SIGINT shutdown:\n' + log.read_text())
    finally:
        server.shutdown()
    _assert_clean(process, log.read_text())


def test_perception_sigint_exits_without_segfault_while_publishing(tmp_path: Path):
    log = tmp_path / 'perception-shutdown.log'
    env = _env(tmp_path)
    process = subprocess.Popen(
        [sys.executable, str(Path(__file__).resolve()), 'perception'],
        cwd=tmp_path,
        env=env,
        stdout=log.open('w'),
        stderr=subprocess.STDOUT,
    )
    context = Context()
    rclpy.init(context=context, domain_id=DOMAIN)
    node = rclpy.create_node('shutdown_frame_source', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    published = []
    node.create_subscription(String, '/z70/objects/json', published.append, LATCHED)
    image_pub = node.create_publisher(CompressedImage, '/camera/front/image_raw/compressed', 2)
    meta_pub = node.create_publisher(FrameMetadata, '/camera/front/metadata', 2)
    jpeg = _jpeg()
    try:
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline and not published:
            assert process.poll() is None, log.read_text()
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
        assert published, log.read_text()
        process.send_signal(signal.SIGINT)
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)
        raise AssertionError('perception did not finish bounded SIGINT shutdown:\n' + log.read_text())
    finally:
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
    _assert_clean(process, log.read_text())


def _child_driver(port: str) -> None:
    sys.argv = [
        'driver', '--ros-args',
        '-p', f'origin:=http://127.0.0.1:{port}',
        '-p', 'camera_hz:=8.0',
        '-p', 'map_refresh_s:=0.0',
        '-p', 'state_hz:=2.0',
    ]
    from z70_driver.driver import main
    main()


def _child_perception() -> None:
    from z70_manipulation.node import perception_main
    perception_main()


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'driver':
        _child_driver(sys.argv[2])
    elif len(sys.argv) > 1 and sys.argv[1] == 'perception':
        _child_perception()
    else:
        raise SystemExit(pytest.main([__file__, *sys.argv[1:]]))
