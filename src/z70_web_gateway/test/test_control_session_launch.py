"""Standard ROS launch exercised exclusively against an inert loopback backend."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
import uuid

import requests


def test_standard_launch_session_stop_resume_with_fake_backend(tmp_path):
    token = 's'*43
    token_file = tmp_path/'fixture-token'; token_file.write_text(token); token_file.chmod(0o600)
    calls = []; shutdown = threading.Event(); release = threading.Event()
    camera_entered = threading.Event()
    class Backend(BaseHTTPRequestHandler):
        def log_message(self, *args): pass
        def reply(self, value, status=200):
            body = json.dumps(value).encode()
            try:
                self.send_response(status); self.send_header('Content-Length', str(len(body)))
                self.send_header('Content-Type', 'application/json'); self.end_headers(); self.wfile.write(body)
            except OSError: pass
        def do_GET(self):
            if self.path == '/api/commands/events':
                self.send_response(200); self.end_headers()
                while not shutdown.wait(.1):
                    try: self.wfile.write(b'\n'); self.wfile.flush()
                    except OSError: return
                return
            if self.path == '/api/state':
                return self.reply(dict(observationSession='offline-session-launch',connection=dict(state='connected'),
                    telemetry=dict(status=dict(value=dict(state=39,error_code=0),observedAt=int(time.time()*1000)))))
            self.reply({},404)
        def do_POST(self):
            body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            calls.append((self.path,body))
            if self.path == '/api/camera/frame':
                camera_entered.set(); release.wait(15); return self.reply({},404)
            if self.path == '/api/driver/control/session':
                return self.reply(dict(enabled=body['enabled'],streams_released=not body['enabled']))
            if self.path == '/api/driver/rpc':
                now = int(time.time()*1000)
                return self.reply(dict(commandId=body['commandId'],attemptId=str(uuid.uuid4()),attemptIndex=0,
                    startedAt=now,finishedAt=now,dispatch='sent',response='received',
                    result=dict(status=0,client_id='none') if body['method']=='get_homesec_connect_status' else ['ok']))
            self.reply({},404)
    backend = ThreadingHTTPServer(('127.0.0.1',0),Backend); backend.daemon_threads = True
    bt = threading.Thread(target=backend.serve_forever); bt.start()
    def port():
        with socket.socket() as s: s.bind(('127.0.0.1',0)); return s.getsockname()[1]
    fox_port = port()
    web_port = None
    for candidate in range(57070, 57100):
        with socket.socket() as s:
            try: s.bind(('127.0.0.1', candidate))
            except OSError: continue
            web_port = candidate; break
    assert web_port is not None, 'no isolated gateway test port available'
    launch_tmp = tmp_path/'tmp'; launch_tmp.mkdir()
    # XDG isolation keeps both estimate and chassis-calibration persistence test-owned.
    env = dict(os.environ,TMPDIR=str(launch_tmp),ROS_DOMAIN_ID='97',XDG_DATA_HOME=str(tmp_path/'data'),ROS_LOG_DIR=str(tmp_path/'ros-logs'),
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    log = tmp_path/'launch.log'
    with log.open('w') as output:
        process = subprocess.Popen(['ros2','launch','z70_bringup','bringup.launch.py',
            f'origin:=http://127.0.0.1:{backend.server_port}',f'token_file:={token_file}',
            'camera_hz:=5.0','mapping:=false','depth:=false','manipulation:=false','web_gateway:=true',
            f'web_gateway_port:={web_port}',f'foxglove_port:={fox_port}'],
            env=env,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
        headers = {'Authorization':'Bearer '+token}
        def request(method,path,body=None):
            return requests.request(method,f'http://127.0.0.1:{web_port}'+path,json=body,headers=headers,timeout=35)
        try:
            deadline = time.monotonic()+25
            while time.monotonic()<deadline:
                assert process.poll() is None, log.read_text()
                try:
                    response=request('GET','/api/ros/state')
                    if response.status_code==200 and response.json()['available']: break
                except requests.RequestException: pass
                time.sleep(.1)
            else: raise AssertionError(log.read_text())
            assert camera_entered.wait(5),log.read_text()
            before=time.monotonic()
            stopped=request('POST','/api/ros/control/session',{'enabled':False})
            assert stopped.status_code==200,stopped.text
            assert time.monotonic()-before<3,'session stop must not wait for the stalled image'
            assert stopped.json()['streams_released'] and not stopped.json()['enabled']
            assert not stopped.json()['cessation_observed']
            methods=[body['method'] for path,body in calls if path=='/api/driver/rpc']
            assert all(method in methods for method in ('app_rc_stop','app_rc_end','app_stop_grasp','app_pick_up_enter_exit'))
            release.set();time.sleep(.5)
            frame_count=sum(path=='/api/camera/frame' for path,_ in calls)
            time.sleep(.5)
            assert sum(path=='/api/camera/frame' for path,_ in calls)==frame_count==1
            resumed=request('POST','/api/ros/control/session',{'enabled':True})
            assert resumed.status_code==200 and resumed.json()['enabled']
            deadline=time.monotonic()+3
            while sum(path=='/api/camera/frame' for path,_ in calls)==1 and time.monotonic()<deadline:time.sleep(.05)
            assert sum(path=='/api/camera/frame' for path,_ in calls)>1
            assert request('POST','/api/ros/control/session',{'enabled':False}).json()['enabled'] is False
        finally:
            release.set();shutdown.set()
            os.killpg(process.pid,signal.SIGINT)
            try: process.wait(timeout=25)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
                raise AssertionError('isolated launch failed to shut down: '+log.read_text())
            backend.shutdown();backend.server_close();bt.join(3)
    assert process.returncode==0,log.read_text()
