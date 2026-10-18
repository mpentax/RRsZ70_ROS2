"""Bounded live observation with private mutable state and owned-process cleanup.
Run from the repository root (live authorization required):
pixi run bash -c 'source install/setup.bash && python src/z70_bringup/foxglove/validate_live.py'
"""
import importlib
import json
import math
import os
from pathlib import Path
import re
import signal
import shutil
import socket
import subprocess
import time
import tempfile

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
OPTIONAL_TOPICS = {
    '/camera/front/depth/points': 'optional depth worker may be unavailable',
    '/camera/gripper/image_raw/compressed': 'requires the selected gripper lens; validation never switches it',
}


def referenced_topics(value):
    topics = set()
    if isinstance(value, dict):
        for k,v in value.items():
            topics |= referenced_topics(k) | referenced_topics(v)
    elif isinstance(value, list):
        for item in value: topics |= referenced_topics(item)
    elif isinstance(value, str) and value.startswith('/'):
        topics.add(re.split(r'[.\[]',value)[0])
    return topics


def prepare_launch(scratch, environ):
    """Prepare private mutable state without reading or copying the user's state."""
    scratch = Path(scratch).resolve()
    env = dict(environ)
    for key, relative in {
        'HOME': 'home', 'XDG_DATA_HOME': 'data', 'XDG_CACHE_HOME': 'cache',
        'XDG_CONFIG_HOME': 'config', 'XDG_STATE_HOME': 'state',
        'ROS_HOME': 'ros', 'ROS_LOG_DIR': 'ros/log', 'TMPDIR': 'tmp',
    }.items():
        path = scratch / relative
        path.mkdir(parents=True, exist_ok=True)
        env[key] = str(path)
    env['ROS_DOMAIN_ID'] = '96'
    env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    env.pop('Z70_REPO', None)
    # HOME is private, but the existing optional depth interpreter remains usable.
    home = Path(environ.get('HOME') or Path.home()).resolve()
    args = ['bash', '-c',
            'source install/setup.bash && exec ros2 launch z70_bringup bringup.launch.py "$@"',
            '--', 'foxglove_port:=8876', f'token_file:={ROOT}/.local/service-token',
            f'depth_python:={home}/.venvs/roboclaw-depth/bin/python']
    # Omit calibration overrides: bringup resolves the adopted installed resources.
    return args, env


def cleanup_owned_group(proc):
    """Drain the session we launched, even if its leader has already exited."""
    for sig, timeout in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + timeout
        while True:
            proc.poll()  # Reap the leader independently of its surviving children.
            try:
                os.killpg(proc.pid, 0)
            except ProcessLookupError:
                proc.wait(timeout=5)
                return
            if time.monotonic() >= deadline:
                break
            time.sleep(.1)
    else:
        raise TimeoutError('Owned bringup process group survived bounded cleanup')
    proc.wait(timeout=5)


def validate(args, env):
    port = 8876
    with socket.socket() as check:
        check.bind(('127.0.0.1',port))
    wanted = referenced_topics(json.loads((HERE/'z70-dashboard.json').read_text())) | {'/tf','/tf_static','/joint_states'}
    observed = {}; counts = {}; subscriptions = {}; node = None; advertised = {}; result = {}
    with (HERE/'bringup-validation.log').open('w') as log:
        proc = subprocess.Popen(args, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            rclpy.init()
            node = rclpy.create_node('z70_dashboard_validator')
            deadline = time.monotonic() + 65
            def receive(topic, msg):
                counts[topic] = counts.get(topic,0) + 1
                if topic == '/diagnostics':
                    observed[topic] = [{'name':s.name,'level':ord(s.level) if isinstance(s.level,bytes) else s.level,'message':s.message,
                        'values':{v.key:v.value for v in s.values}} for s in msg.status]
                elif hasattr(msg,'data') and isinstance(msg.data,(str,int,float,bool)):
                    observed[topic] = msg.data if not isinstance(msg.data, float) or math.isfinite(msg.data) else None
                else:
                    observed[topic] = {'schema':type(msg).__name__}
            while time.monotonic() < deadline and proc.poll() is None:
                advertised = dict(node.get_topic_names_and_types())
                for topic in wanted - subscriptions.keys():
                    types = advertised.get(topic)
                    if not types: continue
                    package,_,typename = types[0].split('/')
                    cls = getattr(importlib.import_module(package+'.msg'),typename)
                    infos = node.get_publishers_info_by_topic(topic)
                    if not infos: continue
                    durable = topic in ('/tf_static','/robot_description','/map','/z70/map/markers','/z70/map/robot_pose','/z70/chassis_calibration') or all(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for i in infos)
                    qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE if durable else ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL if durable else DurabilityPolicy.VOLATILE)
                    subscriptions[topic] = node.create_subscription(cls,topic,lambda msg,t=topic:receive(t,msg),qos)
                rclpy.spin_once(node,timeout_sec=.1)
            result = dict(domain=96,foxglove_url=f'ws://127.0.0.1:{port}',
                topics={t:dict(types=advertised.get(t,[]),messages=counts.get(t,0),
                      last=observed.get(t),optional_reason=OPTIONAL_TOPICS.get(t)) for t in sorted(wanted)},
                missing_undocumented=[t for t in sorted(wanted) if counts.get(t,0)==0 and t not in OPTIONAL_TOPICS],
                bringup_exit=proc.poll())
        finally:
            try:
                try:
                    if node: node.destroy_node()
                finally:
                    rclpy.try_shutdown()
            finally:
                if result:
                    result['bringup_exit'] = proc.poll()  # Before any validator-requested signal.
                cleanup_owned_group(proc)
    result['cleanup_exit'] = proc.returncode
    (HERE/'live-validation.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='topics'},indent=2))
    print('Published',sum(bool(v['messages']) for v in result['topics'].values()),'of',len(wanted),'referenced topics')
    for t,v in result['topics'].items(): print(t,v['messages'],v['types'],v['optional_reason'] or '')
    if result['missing_undocumented'] or result['bringup_exit'] is not None: raise SystemExit(1)

def main():
    scratch = tempfile.mkdtemp(prefix='z70-dashboard-', dir=os.environ.get('TMPDIR'))
    # Exceptional exits may leave owned workers alive; retain diagnostics conservatively.
    retain_scratch = True
    try:
        args, env = prepare_launch(scratch, os.environ)
        # The observer needs the same DDS domain and private ROS logging paths.
        keys = ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE', 'ROS_HOME', 'ROS_LOG_DIR')
        previous = {key: os.environ.get(key) for key in keys}
        try:
            os.environ.update({key: env[key] for key in keys})
            validate(args, env)
        finally:
            for key, value in previous.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value
        retain_scratch = False  # validate returned only after confirmed group cleanup.
    finally:
        if not retain_scratch:
            shutil.rmtree(scratch)


if __name__ == '__main__': main()
