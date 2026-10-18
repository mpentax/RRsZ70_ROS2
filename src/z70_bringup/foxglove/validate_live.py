"""Bounded read-only bringup validation. Uses an isolated domain/port; cleans up its process group.
Run: pixi run python src/z70_bringup/foxglove/validate_live.py (live authorization required)
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
FUTURE = {
    '/map': 'z70_mapping occupancy grid',
    '/z70/map/markers': 'z70_mapping room MarkerArray',
    '/z70/map/robot_pose': 'z70_mapping robot pose',
    '/camera/front/depth/points': 'z70_mapping PointCloud2 depth',
    **{f'/camera/{c}/image_raw/compressed': 'camera bridge optional source; never switches the robot lens'
       for c in ('left', 'right', 'back', 'gripper')},
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


def main():
    os.environ['ROS_DOMAIN_ID'] = '96'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    port = 8876
    # Keep even passive calibration seed/save writes out of the user's live calibration file.
    scratch = Path(tempfile.mkdtemp(prefix='z70-dashboard-', dir=os.environ.get('TMPDIR',str(HERE))))
    with socket.socket() as check:
        check.bind(('127.0.0.1',port))
    saved = ROOT/'calibration/chassis/online.json'
    if saved.exists():
        copied = scratch/'calibration/chassis/online.json'
        copied.parent.mkdir(parents=True)
        shutil.copy2(saved,copied)
    wanted = referenced_topics(json.loads((HERE/'z70-dashboard.json').read_text())) | {'/tf','/tf_static','/joint_states'}
    args = [str(Path.home()/'.pixi/bin/pixi'), 'run', 'bash', '-c',
            'source install/setup.bash && export Z70_REPO="$1" && shift && exec ros2 launch z70_bringup bringup.launch.py "$@"',
            '--', str(scratch), f'foxglove_port:={port}', f'token_file:={ROOT}/.local/service-token',
            f'camera_info:={ROOT}/calibration/front-camera/z70_front_camera_info.yaml',
            f'calibration:={ROOT}/calibration/front-camera/front_camera.json']
    observed = {}; counts = {}; subscriptions = {}; node = None; advertised = {}
    with (HERE/'bringup-validation.log').open('w') as log:
        proc = subprocess.Popen(args, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
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
                      last=observed.get(t),future=FUTURE.get(t)) for t in sorted(wanted)},
                missing_undocumented=[t for t in sorted(wanted) if counts.get(t,0)==0 and t not in FUTURE],
                bringup_exit=proc.poll())
        finally:
            if node: node.destroy_node()
            rclpy.try_shutdown()
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try: proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM)
                    try: proc.wait(timeout=5)
                    except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGKILL); proc.wait()
    result['cleanup_exit'] = proc.returncode
    shutil.rmtree(scratch)
    (HERE/'live-validation.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='topics'},indent=2))
    print('Published',sum(bool(v['messages']) for v in result['topics'].values()),'of',len(wanted),'referenced topics')
    for t,v in result['topics'].items(): print(t,v['messages'],v['types'],v['future'] or '')
    if result['missing_undocumented']: raise SystemExit(1)

if __name__ == '__main__': main()
