"""HTTP & SSE simulation transport sidecar emulating the Roborock vendor adapter for z70_driver."""
from __future__ import annotations

import argparse
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import queue
import secrets
import threading
import time
from typing import Any
import uuid
import urllib.request
import urllib.error

from .world import SimWorld


def ensure_token_file(path: str | Path) -> str:
    p = Path(path)
    if p.is_file():
        return p.read_text().strip()
    p.parent.mkdir(parents=True, exist_ok=True)
    token = secrets.token_urlsafe(32)
    p.write_text(token)
    p.chmod(0o600)
    return token


def _pulse_params(method: str, params: Any) -> dict[str, float] | None:
    if method != 'app_rc_move' or not isinstance(params, list) or len(params) != 1:
        return None
    p = params[0]
    if not isinstance(p, dict):
        return None
    try:
        vel = float(p['velocity'])
        omega = float(p['omega'])
        dur = float(p['duration'])
    except (KeyError, TypeError, ValueError):
        return None
    if not all(abs(x) <= 10.0 for x in (vel, omega)) or not (0.0 < dur <= 120000.0):
        return None
    return dict(velocity=vel, omega=omega, duration=dur)


class SimServer:
    """Thread-safe HTTP server implementing the z70_driver private transport and optional /api/ros/* proxy."""

    def __init__(
        self,
        world: SimWorld | None = None,
        host: str = '127.0.0.1',
        port: int = 0,
        token: str = '',
        ros_gateway_origin: str = 'http://127.0.0.1:57020',
    ):
        self.world = world or SimWorld()
        self.host = host
        self.requested_port = port
        self.token = token
        self.ros_gateway_origin = ros_gateway_origin.rstrip('/')
        self.observation_session = str(uuid.uuid4())
        self.events_session = str(uuid.uuid4())
        self.event_sequence = 0
        self.event_lock = threading.Lock()
        self.event_clients: list[queue.Queue[bytes | None]] = []
        self.session_enabled = True
        self.stop_event = threading.Event()
        self.httpd = ThreadingHTTPServer((self.host, self.requested_port), self._make_handler())
        self.httpd.daemon_threads = True
        self._thread: threading.Thread | None = None

    @property
    def port(self) -> int:
        return int(self.httpd.server_port)

    @property
    def origin(self) -> str:
        return f'http://{self.host}:{self.port}'

    def start(self) -> 'SimServer':
        if self._thread is None:
            self._thread = threading.Thread(target=self.httpd.serve_forever, name='z70_sim_http', daemon=True)
            self._thread.start()
        return self

    def stop(self) -> None:
        self.stop_event.set()
        with self.event_lock:
            for q in self.event_clients:
                try:
                    q.put_nowait(None)
                except queue.Full:
                    pass
            self.event_clients.clear()
        self.httpd.shutdown()
        self.httpd.server_close()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=5.0)

    def emit_event(
        self,
        *,
        command_id: str,
        attempt_id: str,
        attempt_index: int,
        phase: str,
        operation: str,
        requested_at: int,
        attempted_at: int | None,
        finished_at: int | None,
        dispatch: str,
        response: str,
        error_code: str = 'NONE',
        vendor_code: int | None = None,
        params: dict[str, float] | None = None,
    ) -> None:
        with self.event_lock:
            self.event_sequence += 1
            row: dict[str, Any] = dict(
                sessionId=self.events_session,
                sequence=self.event_sequence,
                commandId=command_id,
                attemptId=attempt_id,
                attemptIndex=attempt_index,
                phase=phase,
                operation=operation,
                requestedAt=requested_at,
                attemptedAt=attempted_at,
                finishedAt=finished_at,
                writeTimeKnown=False,
                dispatch=dispatch,
                response=response,
                errorCode=error_code,
            )
            if vendor_code is not None:
                row['vendorCode'] = vendor_code
            if params is not None:
                row['params'] = params
            line = json.dumps(row).encode('utf-8') + b'\n'
            for q in list(self.event_clients):
                try:
                    q.put_nowait(line)
                except queue.Full:
                    self.event_clients.remove(q)

    def dispatch_rpc(
        self,
        method: str,
        params: Any,
        command_id: str | None = None,
        transport: str = 'local',
    ) -> dict[str, Any]:
        cid = command_id or str(uuid.uuid4())
        aid = str(uuid.uuid4())
        started_at = int(time.time() * 1000)
        pp = _pulse_params(method, params)
        attempted_at: int | None = None
        finished_at: int | None = None

        def emit(phase: str, dispatch: str, response: str, error_code: str = 'NONE',
                 vendor_code: int | None = None, attempt: str = aid) -> None:
            self.emit_event(
                command_id=cid, attempt_id=attempt, attempt_index=0, phase=phase, operation=method,
                requested_at=started_at, attempted_at=attempted_at, finished_at=finished_at,
                dispatch=dispatch, response=response, error_code=error_code,
                vendor_code=vendor_code, params=pp,
            )

        emit('accepted', 'not_sent', 'none', attempt=cid)
        attempted_at = int(time.time() * 1000)
        emit('dispatched', 'unknown', 'none')

        kind, val = self.world.handle_rpc(method, params)
        finished_at = int(time.time() * 1000)
        base = dict(requestId=cid, commandId=cid, attemptId=aid, attemptIndex=0,
                    startedAt=started_at, finishedAt=finished_at, deviceId='sim-z70-duid', transport=transport)

        def finish(dispatch: str, response: str, error_code: str = 'NONE', vendor_code: int | None = None) -> None:
            for phase in ('replied', 'finished'):
                emit(phase, dispatch, response, error_code, vendor_code)

        if kind == 'ok':
            finish('sent', 'received')
            return dict(base, dispatch='sent', response='received', result=val)

        if kind == 'vendor_error':
            vcode = int(val) if isinstance(val, int) else -10007
            finish('sent', 'vendor_error', 'VENDOR_REJECTED', vcode)
            return dict(base, dispatch='sent', response='vendor_error', vendorCode=vcode, error='Robot rejected RPC')

        finish('unknown', 'transport_error', 'TIMEOUT')
        return dict(base, dispatch='unknown', response='transport_error',
                    transportReason='timeout', error='Device transport failed')

    def _make_handler(self):
        sim = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, format: str, *args: Any) -> None:
                pass

            def _check_auth(self) -> bool:
                if not sim.token:
                    return True
                if hmac.compare_digest(self.headers.get('Authorization', ''), f'Bearer {sim.token}'):
                    return True
                self._json(403, {'error': 'Private driver bearer authentication required.', 'dispatch': 'not_sent'})
                return False

            def _send(self, status: int, content_type: str, payload: bytes, headers: dict[str, Any] | None = None) -> None:
                self.send_response(status)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(payload)))
                for k, v in (headers or {}).items():
                    self.send_header(k, str(v))
                self.end_headers()
                self.wfile.write(payload)

            def _json(self, status: int, body: Any, headers: dict[str, Any] | None = None) -> None:
                self._send(status, 'application/json; charset=utf-8', json.dumps(body).encode('utf-8'), headers)

            def _read_json(self) -> dict[str, Any]:
                length = int(self.headers.get('Content-Length', '0'))
                raw = self.rfile.read(length) if length > 0 else b'{}'
                return json.loads(raw.decode('utf-8'))

            def _proxy_ros(self, method: str) -> None:
                length = int(self.headers.get('Content-Length', '0'))
                data = self.rfile.read(length) if length > 0 else None
                headers = {'Accept': 'application/json'}
                if data is not None:
                    headers['Content-Type'] = 'application/json'
                if sim.token:
                    headers['Authorization'] = f'Bearer {sim.token}'
                req = urllib.request.Request(
                    sim.ros_gateway_origin + self.path,
                    data=data,
                    headers=headers,
                    method=method,
                )
                try:
                    with urllib.request.urlopen(req, timeout=35.0) as resp:
                        forwarded = {
                            k: v for k, v in resp.headers.items()
                            if k.lower() not in ('transfer-encoding', 'connection', 'content-length', 'content-type')
                        }
                        self._send(resp.status, resp.headers.get('Content-Type', 'application/json'), resp.read(), forwarded)
                except urllib.error.HTTPError as e:
                    self._send(e.code, e.headers.get('Content-Type', 'application/json'), e.read())
                except Exception:
                    self._json(503, {'error': 'ROS_UNAVAILABLE', 'dispatch': 'not_sent'})

            def do_GET(self) -> None:
                if self.path == '/healthz':
                    self._json(200, {'available': not sim.stop_event.is_set(), 'simulator': 'mujoco'})
                    return
                if self.path.startswith('/api/ros/'):
                    self._proxy_ros('GET')
                    return
                if not self._check_auth():
                    return
                if self.path == '/api/commands/events':
                    q: queue.Queue[bytes | None] = queue.Queue(maxsize=256)
                    with sim.event_lock:
                        sim.event_clients.append(q)
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/x-ndjson')
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                    try:
                        while not sim.stop_event.is_set():
                            try:
                                item = q.get(timeout=0.4)
                                if item is None:
                                    break
                                self.wfile.write(item)
                                self.wfile.flush()
                            except queue.Empty:
                                self.wfile.write(b'\n')
                                self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        pass
                    finally:
                        with sim.event_lock:
                            if q in sim.event_clients:
                                sim.event_clients.remove(q)
                    return

                if self.path == '/api/state':
                    now = int(time.time() * 1000)
                    status_val = sim.world.status_dict()
                    cam_val = sim.world.camera_status_int()
                    self._json(
                        200,
                        dict(
                            observationSession=sim.observation_session,
                            device=dict(
                                id='sim-z70-duid',
                                name='Roborock Saros Z70 (MuJoCo Sim)',
                                model='roborock.vacuum.a179',
                                cloudProtocolVersion='L01',
                                localProtocol='L01',
                            ),
                            connection=dict(state='connected', protocol='L01', connectedAt=now),
                            telemetry=dict(
                                status=dict(value=status_val, observedAt=now, requestedAt=now, error=None),
                                camera=dict(value=[cam_val], observedAt=now, requestedAt=now, error=None),
                            ),
                            api=dict(
                                rpc=None,
                                commandOwner='ROS',
                                rosRawRpc='/z70/driver/raw_rpc',
                                genericVendorRpc=True,
                            ),
                            camera=dict(
                                configured=True,
                                pinCached=True,
                                feedLensVerified=True,
                                acquisition=dict(available=True, stage='connected', failure='none'),
                                streamDiagnostics=None,
                                sourceAudit=dict(available=True, matches=True),
                            ),
                        ),
                    )
                    return

                if self.path == '/api/home/rooms':
                    self._json(
                        200,
                        dict(
                            source='cached',
                            rooms=sim.world.home_rooms(),
                        ),
                    )
                    return

                if self.path == '/api/map/cache/raw':
                    raw, seq, saved_at = sim.world.raw_map(refresh=False)
                    self._send(
                        200,
                        'application/octet-stream',
                        raw,
                        {
                            'X-Roboclaw-Observation-Session': sim.observation_session,
                            'X-Map-Saved-At': saved_at,
                            'X-Map-Index': 0,
                            'X-Map-Sequence': seq,
                        },
                    )
                    return

                self._json(404, {'error': 'Not found.'})

            def do_POST(self) -> None:
                if self.path.startswith('/api/ros/'):
                    self._proxy_ros('POST')
                    return
                if not self._check_auth():
                    return
                body = self._read_json()

                if self.path == '/api/driver/control/session':
                    enabled = bool(body.get('enabled', True))
                    sim.session_enabled = enabled
                    if not enabled:
                        sim.world.preview_status = 0
                        sim.world.preview_client = 'none'
                    self._json(200, {'enabled': sim.session_enabled, 'streams_released': not sim.session_enabled})
                    return

                if self.path == '/api/driver/rpc':
                    method = str(body.get('method', ''))
                    params = body.get('params', [])
                    command_id = body.get('commandId')
                    transport = str(body.get('transport', 'local'))
                    outcome = sim.dispatch_rpc(method, params, command_id=command_id, transport=transport)
                    self._json(200 if outcome['response'] == 'received' else 502, outcome)
                    return

                if self.path == '/api/camera/frame':
                    if not sim.session_enabled:
                        self._json(409, {'error': 'SESSION_STOPPED', 'dispatch': 'not_sent'})
                        return
                    jpeg, active, w, h = sim.world.render_jpeg()
                    self._send(
                        200,
                        'image/jpeg',
                        jpeg,
                        {
                            'X-Roboclaw-Frame-Id': uuid.uuid4(),
                            'X-Roboclaw-Camera-Session': sim.observation_session,
                            'X-Roboclaw-Received-At': int(time.time() * 1000),
                            'X-Roboclaw-Camera': active,
                            'X-Roboclaw-Size-Camera': active,
                            'X-Roboclaw-Image-Width': w,
                            'X-Roboclaw-Image-Height': h,
                        },
                    )
                    return

                if self.path == '/api/driver/camera/select':
                    target = body.get('camera')
                    if target not in ('front', 'gripper'):
                        self._json(400, {'error': 'Unknown camera name.', 'dispatch': 'not_sent'})
                        return
                    outcomes = []
                    if sim.world.camera != target:
                        flag = 1 if target == 'gripper' else 0
                        out = sim.dispatch_rpc('app_change_camera', {'flag': flag})
                        outcomes.append({'method': 'app_change_camera', 'outcome': out})
                        if out['response'] == 'vendor_error':
                            self._json(
                                409,
                                {
                                    'error': 'Robot refused camera selection; gripper needs pickup mode with arm out.',
                                    'dispatch': 'sent',
                                },
                            )
                            return
                    self._json(200, {'camera': sim.world.camera, 'outcomes': outcomes})
                    return

                if self.path == '/api/map/cache/refresh':
                    raw, seq, saved_at = sim.world.raw_map(refresh=True)
                    self._json(
                        200,
                        dict(cached=True, savedAt=saved_at, mapIndex=0, mapSequence=seq, bytes=len(raw)),
                    )
                    return

                self._json(404, {'error': 'Not found.'})

        return Handler


def create_sim_server(
    world: SimWorld | None = None,
    host: str = '127.0.0.1',
    port: int = 0,
    token: str = '',
) -> SimServer:
    return SimServer(world=world, host=host, port=port, token=token).start()


def main() -> None:
    parser = argparse.ArgumentParser(description='MuJoCo Roborock Z70 simulation transport sidecar.')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=57010)
    parser.add_argument('--token-file', default='')
    parser.add_argument('--ros-gateway-origin', default='http://127.0.0.1:57020')
    args = parser.parse_args()

    token = ensure_token_file(args.token_file) if args.token_file else ''

    server = SimServer(host=args.host, port=args.port, token=token, ros_gateway_origin=args.ros_gateway_origin)
    server.start()
    print(f'z70_sim MuJoCo sidecar listening on {server.origin}')
    try:
        while not server.stop_event.wait(1.0):
            pass
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == '__main__':
    main()
