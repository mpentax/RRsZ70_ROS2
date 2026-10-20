"""Shared driver state, camera frame, and map publishing across ROS consumers."""
from copy import deepcopy
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import importlib
import json
import os
import queue
import signal
import struct
import subprocess
import threading
import time
from types import SimpleNamespace
import uuid
import yaml

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from nav_msgs.msg import OccupancyGrid
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from z70_interfaces.msg import RobotState, MapSnapshot, ArmEstimate, FrameMetadata, CommandEvent, CalibrationState, OdometryStatus
from z70_interfaces.srv import RefreshMap, SelectCamera
from z70_driver.driver import Driver, LATCHED, host_time, host_ms
from z70_driver.frame_subscription import MatchedFrames
from z70_manipulation.observations import Observations
from z70_mapping.node import MappingNode


def wait_for(predicate, seconds=5.):
    deadline = time.monotonic() + seconds
    while not predicate():
        assert time.monotonic() < deadline, 'condition deadline'
        time.sleep(.01)


def synthetic_map():
    def block(kind,data,extra=b''):
        return struct.pack('<HHI',kind,8+len(extra),len(data))+extra+data
    image=block(2,bytes([1,23,0,39,255,7]),struct.pack('<iiII',100,200,2,3))
    body=image+block(8,struct.pack('<iii',10075,5025,90))+struct.pack('<HHI',1024,8,20)
    raw=b'rr'+struct.pack('<HI',20,len(body)+20)+bytes(12)+body
    return raw+hashlib.sha1(raw).digest()


@pytest.fixture
def backend():
    data:dict=dict(camera='front',calls=[],fail_state=False,fail_frame=False,wrong_tag=False,
              session=str(uuid.uuid4()),updated=int(time.time()*1000),frames=0,
              saved=int(time.time()*1000),stop=threading.Event(),events=queue.Queue(),arm_state='IN_PROCESS_POS',stale_state=False)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def reply(self,value,status=200,headers=None):
            content=value if isinstance(value,bytes) else json.dumps(value).encode()
            self.send_response(status)
            self.send_header('Content-Length',str(len(content)))
            for k,v in (headers or {}).items():self.send_header(k,str(v))
            self.end_headers();self.wfile.write(content)
        def do_GET(self):
            data['calls'].append(('GET',self.path,None))
            if self.path=='/api/commands/events':
                self.send_response(200);self.send_header('Content-Type','application/x-ndjson');self.end_headers()
                while not data['stop'].is_set():
                    try:row=data['events'].get(timeout=.2);line=json.dumps(row).encode()+b'\n'
                    except queue.Empty:line=b'\n'
                    try:self.wfile.write(line);self.wfile.flush()
                    except (BrokenPipeError,ConnectionResetError):break
                return
            if self.path=='/api/state':
                if data['fail_state']:return self.reply({},503)
                now=int(time.time()*1000)-(10000 if data['stale_state'] else 0)
                return self.reply(dict(observationSession=data['session'],connection=dict(state='connected'),telemetry=dict(
                    status=dict(value=dict(state=39,error_code=0,battery=80,common_status=256,in_cleaning=0,in_returning=0),observedAt=now),
                    camera=dict(value=[32768 if data['camera']=='gripper' else 0],observedAt=now))))

            if self.path=='/api/map/cache/raw':
                return self.reply(synthetic_map(),headers={'X-Roboclaw-Observation-Session':data['session'],'X-Map-Saved-At':data['saved'],'X-Map-Index':1,'X-Map-Sequence':4})
            if self.path=='/api/home/rooms':return self.reply(dict(rooms=[dict(id=10,name='Synthetic room')]))
            self.reply({},404)
        def do_POST(self):
            body=json.loads(self.rfile.read(int(self.headers.get('Content-Length',0))))
            data['calls'].append(('POST',self.path,body))
            if self.path=='/api/camera/frame':
                assert body=={}
                if data['fail_frame']:return self.reply({},503)
                data['frames']+=1
                height,width=data.get('image_shape',(12 if data['camera']=='front' else 10,16))
                image=np.full((height,width,3),data['frames']%255,np.uint8)
                jpeg=cv2.imencode('.jpg',image)[1].tobytes()
                return self.reply(jpeg,headers={'X-Roboclaw-Camera':'invalid' if data['wrong_tag'] else data['camera'],
                    'X-Roboclaw-Camera-Session':data['session'],'X-Roboclaw-Received-At':int(time.time()*1000)})
            if self.path=='/api/driver/camera/select':
                data['camera']=body['camera'];return self.reply(dict(camera=data['camera']))
            if self.path=='/api/map/cache/refresh':return self.reply(dict(available=True))
            if self.path=='/api/driver/rpc':
                assert body['method']=='get_room_mapping'  # No fake motion hidden inside acquisition.
                return self.reply(dict(result=[[2,10]], commandId=str(uuid.uuid4()), attemptId=str(uuid.uuid4()),
                    attemptIndex=0, startedAt=int(time.time()*1000), finishedAt=int(time.time()*1000),
                    dispatch='sent', response='received'))
            self.reply({},404)
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
    thread=threading.Thread(target=server.serve_forever);thread.start()
    data['origin']=f'http://127.0.0.1:{server.server_port}'
    try:yield data
    finally:data['stop'].set();server.shutdown();server.server_close();thread.join()


def camera_yaml(path,height):
    path.write_text(yaml.safe_dump(dict(image_width=16,image_height=height,distortion_model='plumb_bob',
        camera_matrix=dict(data=[20.,0.,8.,0.,21.,height/2,0.,0.,1.]),
        distortion_coefficients=dict(data=[.1,-.01,0.,0.,0.]),
        rectification_matrix=dict(data=[1.,0.,0.,0.,1.,0.,0.,0.,1.]),
        projection_matrix=dict(data=[20.,0.,8.,0.,0.,21.,height/2,0.,0.,0.,1.,0.]))))


def test_shared_acquisition_serialization_late_join_switch_stale_cleanup(backend,tmp_path):
    front=tmp_path/'front.yaml';gripper=tmp_path/'gripper.yaml'
    camera_yaml(front,12);camera_yaml(gripper,10)
    rclpy.init(domain_id=96)
    driver=Driver([Parameter('origin',value=backend['origin']),Parameter('camera_hz',value=8.),
                   Parameter('arm_estimate_path',value=''),
                   Parameter('state_hz',value=5.),Parameter('camera_info',value=str(front)),
                   Parameter('gripper_camera_info',value=str(gripper)),Parameter('map_refresh_s',value=60.),
                   Parameter('camera_stale_after_s',value=.4),Parameter('state_stale_after_s',value=2.)])
    with driver.estimator_lock:
        driver.estimator.set_anchor('deployed',reason='TEST_ANCHOR')
        expected_arm=driver.estimator.snapshot()
        driver.publish_estimate(expected_arm)
    mapping=MappingNode();client=rclpy.create_node('shared_test_consumers')
    executor=MultiThreadedExecutor(num_threads=5)
    for node in (driver,mapping,client):executor.add_node(node)
    thread=threading.Thread(target=executor.spin);thread.start()
    observers=[]
    try:
        wait_for(lambda:'map' in driver.samples and 'arm' in driver.samples and 'camera' in driver.samples)
        # Subscribe AFTER acquisition: latched source identity/time must be replayed, not recreated.
        original=deepcopy(driver.samples['map'][0])
        maps=[];states=[];arms=[];grids=[];raws=[];jpegs=[];infos=[];metas=[];diagnostics=[];events=[]
        client.create_subscription(MapSnapshot,'/z70/driver/map_snapshot',maps.append,LATCHED)
        client.create_subscription(RobotState,'/z70/state',states.append,LATCHED)
        client.create_subscription(ArmEstimate,'/z70/arm_estimate',arms.append,LATCHED)
        client.create_subscription(OccupancyGrid,'/map',grids.append,LATCHED)
        for camera in ('front','gripper'):
            client.create_subscription(Image,f'/camera/{camera}/image_raw',raws.append,2)
            client.create_subscription(CompressedImage,f'/camera/{camera}/image_raw/compressed',jpegs.append,2)
            client.create_subscription(CameraInfo,f'/camera/{camera}/camera_info',infos.append,2)
            client.create_subscription(FrameMetadata,f'/camera/{camera}/metadata',metas.append,2)
        client.create_subscription(DiagnosticArray,'/diagnostics',diagnostics.append,10)
        client.create_subscription(CommandEvent,'/z70/commands',events.append,256)
        observers=[Observations(client,timeout=2.),Observations(client,timeout=2.)]
        wait_for(lambda:maps and states and arms and grids and metas and infos and raws and jpegs)
        assert maps[0].evidence==original.evidence
        assert bytes(maps[0].raw)==synthetic_map() and maps[0].rooms[0].name=='Synthetic room'
        assert not maps[0].evidence.source_time_known and not maps[0].current_pose_verified
        assert list(grids[0].data)==[100,0,-1,0,0,0]
        assert states[-1].connected and states[-1].battery_fraction==pytest.approx(.8)
        assert arms[-1].validity.value==1 and arms[-1].source==1
        assert arms[-1].joint_sigma_rad==pytest.approx(np.radians([expected_arm['sigmaDeg'][j] for j in ('m3','m4','m5')]))
        assert not arms[-1].wrist_uncertainty_known
        arm_id=deepcopy(arms[-1].evidence)
        time.sleep(.5)
        assert driver.samples['arm'][0].evidence==arm_id
        before=backend['frames']
        for observer in observers:
            image,tag=observer.frame();assert tag=='front' and image.shape==(12,16,3)
            assert observer.get('/api/state')['telemetry']['status']['value']['state']==39
            assert observer.get('/api/state')['telemetry']['status']['value']['common_status']==256
            assert observer.get('/api/state')['telemetry']['status']['value']['in_cleaning'] is False
            assert observer.get('/api/arm/estimate')['joints']['m3']==pytest.approx(expected_arm['joints']['m3'])
        # Two independent ROS readers did not cause extra HTTP acquisition.
        assert backend['frames']-before<=2
        for msg in (maps[0],states[-1],arms[-1],raws[-1],jpegs[-1],infos[-1],metas[-1]):
            assert deserialize_message(serialize_message(msg),type(msg))==msg
        common=set((m.header.stamp.sec,m.header.stamp.nanosec) for m in raws)&set((m.header.stamp.sec,m.header.stamp.nanosec) for m in jpegs)&set((m.header.stamp.sec,m.header.stamp.nanosec) for m in infos)&set((m.header.stamp.sec,m.header.stamp.nanosec) for m in metas)
        assert common
        assert all(not m.exposure_time_known and not m.evidence.source_time_known for m in metas)
        assert all(m.session_scope==1 for m in metas)
        assert infos[-1].k[0]==20. and infos[-1].k[4]==21.
        observers[0].select('gripper')
        wait_for(lambda:any(m.camera==2 and m.height==10 for m in metas))
        image,tag=observers[0].frame();assert tag=='gripper' and image.shape==(10,16,3)
        assert len([c for c in backend['calls'] if c[1]=='/api/driver/camera/select'])==1
        assert all(c[2]=={} for c in backend['calls'] if c[1]=='/api/camera/frame')
        wait_for(lambda:client.count_subscribers('/z70/commands')>0)
        session=str(uuid.uuid4());command=str(uuid.uuid4());attempt=str(uuid.uuid4())
        for i,phase in enumerate(('accepted','dispatched','replied','finished'),1):
            backend['events'].put(dict(sessionId=session,sequence=i,commandId=command,attemptId=attempt,attemptIndex=0,
                phase=phase,operation='app_rc_move',requestedAt=backend['updated'],attemptedAt=backend['updated']+1,
                finishedAt=backend['updated']+20 if i>2 else None,dispatch='unknown' if i==2 else 'sent',
                response='none' if i<3 else 'received',errorCode='NONE',params=dict(velocity=0.,omega=.2,duration=500)))
        wait_for(lambda:len(events)==4)
        assert [e.phase for e in events]==[0,1,2,4]
        assert not any(e.outcome.write_time_known for e in events)
        assert events[1].outcome.dispatch==2
        # A duplicate is ignored; a skipped sequence is diagnosed, never replayed.
        row=dict(sessionId=session,sequence=6,commandId=command,attemptId=attempt,attemptIndex=0,
            phase='finished',operation='app_rc_stop',requestedAt=backend['updated'],finishedAt=backend['updated'],
            dispatch='unknown',response='transport_error',errorCode='TIMEOUT')
        backend['events'].put(row);backend['events'].put(row)
        wait_for(lambda:any(s.message=='EVENT_GAP' for d in diagnostics for s in d.status))
        wait_for(lambda:len(events)==5)
        assert len(events)==5
        # Backend restart changes acquisition sessions, even if cache/estimate values agree.
        oldsession=bytes(states[-1].evidence.session_id.uuid)
        backend['session']=str(uuid.uuid4())
        wait_for(lambda:bytes(states[-1].evidence.session_id.uuid)!=oldsession)
        wait_for(lambda:driver.samples['arm'][0].evidence.session_id==driver.acquisition_session(backend['session']))
        refresh=client.create_client(RefreshMap,'/z70/map/refresh')
        assert refresh.wait_for_service(timeout_sec=2.)
        future=refresh.call_async(RefreshMap.Request());wait_for(future.done)
        assert future.result().outcome.response==1
        wait_for(lambda:maps[-1].evidence.session_id!=original.evidence.session_id)
        assert maps[-1].evidence.received_at==original.evidence.received_at
        # Display-only stow is not planner state or observed physical stow.
        with driver.estimator_lock:
            driver.estimator.observe('app_arm_in_compartment',{},'received',command_id=str(uuid.uuid4()))
            driver.publish_estimate(driver.estimator.snapshot())
        wait_for(lambda:arms[-1].validity.value==3)
        wait_for(lambda:observers[0].arm.validity.value==3)
        assert observers[0].get('/api/arm/estimate')['state']=='IN_ROBOT'
        assert not arms[-1].joints.position
        # Correctly tagged but wrong-size calibration never reaches metric readers.
        oldcal=driver.calibrations['gripper'];wrong=deepcopy(oldcal[0]);wrong.height=100
        driver.calibrations['gripper']=(wrong,oldcal[1])
        wait_for(lambda:any(m.evidence.reason=='CALIBRATION_MISMATCH' for m in metas))
        wait_for(lambda:observers[0].image is not None and not observers[0].image[1].evidence.valid)
        observers[0].timeout=.1
        with pytest.raises(LookupError):observers[0].frame()
        driver.calibrations['gripper']=oldcal
        wait_for(lambda:observers[0].image is not None and observers[0].image[1].evidence.valid)
        # HTTP success with an old vendor observation does not refresh state.
        backend['stale_state']=True
        wait_for(lambda:states[-1].evidence.stale)
        backend['stale_state']=False
        wait_for(lambda:not states[-1].evidence.stale)
        oldstamp=deepcopy(driver.samples['state'][0].evidence.source_at)
        backend['fail_state']=True;backend['fail_frame']=True
        wait_for(lambda:states[-1].evidence.stale and any(m.evidence.stale for m in metas))
        assert not states[-1].connected and states[-1].evidence.source_at==oldstamp
        for d in diagnostics:
            roundtrip=deserialize_message(serialize_message(d),DiagnosticArray)
            assert [s.level for s in roundtrip.status] == [s.level for s in d.status]
            for status in roundtrip.status:
                if status.name.startswith('z70_driver/'):
                    assert status.level==DiagnosticStatus.WARN
        for observer in observers:observer.timeout=.1
        with pytest.raises(LookupError):observers[0].get('/api/state')
        with pytest.raises(LookupError):observers[0].frame()
    finally:
        for observer in observers:observer.close()
        driver.close();executor.shutdown();thread.join(timeout=5)
        for node in (driver,mapping,client):node.destroy_node()
        rclpy.try_shutdown()
    assert not thread.is_alive() and all(not w.is_alive() for w in driver.workers)
    assert not any(c[1]=='/api/commands/recent' for c in backend['calls'])
    assert not any(c[1]=='/api/arm/estimate' for c in backend['calls'])


def test_all_generated_observation_and_service_types_serialize():
    backend=str(uuid.uuid4())
    first=SimpleNamespace(driver_session=uuid.uuid4())
    second=SimpleNamespace(driver_session=uuid.uuid4())
    assert Driver.acquisition_session(first,backend)!=Driver.acquisition_session(second,backend)
    module=importlib.import_module('z70_interfaces.msg')
    names=['EvidenceStamp','Validity','RobotState','CameraState','FrameMetadata','ArmEstimate',
           'OdometryStatus','RoomLabel','MapSnapshot','GarmentObservation','GarmentObservations',
           'CommandOutcome','CommandEvent']
    for name in names:
        cls=getattr(module,name);msg=cls()
        assert deserialize_message(serialize_message(msg),cls)==msg
    msg=CommandEvent(operation='vendor_advanced_'+'research_'*10)
    assert deserialize_message(serialize_message(msg),CommandEvent)==msg
    for cls in (SelectCamera.Request,SelectCamera.Response,RefreshMap.Request,RefreshMap.Response):
        msg=cls();assert deserialize_message(serialize_message(msg),cls)==msg


@pytest.mark.parametrize('milliseconds',[1792074587789,1792074587123,1792074587999])
def test_backend_millisecond_timestamps_preserve_exact_source_identity(milliseconds):
    stamp=host_ms(milliseconds)
    assert stamp.sec*1000+stamp.nanosec//1_000_000==milliseconds
    assert stamp.nanosec%1_000_000==0


def test_frame_join_rejects_unknown_calibration_stale_and_mismatch():
    # Pure callback test with actual generated messages, no ROS graph needed.
    join=MatchedFrames.__new__(MatchedFrames)
    seen=[];sessions=[]
    join.callback=seen.append;join.on_session=lambda:sessions.append(True)
    join.frame=None;join.metadata=None;join.used=None;join.session=None
    image=Image();image.header.stamp=host_time(time.time());image.header.frame_id='camera_optical_frame'
    meta=FrameMetadata(header=deepcopy(image.header),calibration_id='synthetic')
    meta.evidence.valid=True;meta.evidence.received_at=host_time(time.time());meta.evidence.sequence=1
    join.image(image);join.info(meta);assert len(seen)==1
    join.info(meta);assert len(seen)==1
    meta.evidence.sequence=2;meta.evidence.valid=False;join.info(meta);assert len(seen)==1
    meta.evidence.sequence=3;meta.evidence.valid=True;meta.calibration_id='';join.info(meta);assert len(seen)==1
    meta.evidence.sequence=4;meta.calibration_id='synthetic';meta.evidence.stale=True;join.info(meta);assert len(seen)==1
    meta.evidence.sequence=5;meta.evidence.stale=False;meta.header.stamp.sec+=1;join.info(meta);assert len(seen)==1


def test_standard_launch_acquires_fake_images_maps_state_and_shuts_down(backend, tmp_path):
    backend['image_shape'] = (1200, 1600)
    token = tmp_path / 'synthetic-token'; token.write_text('offline-launch-fixture'); token.chmod(0o600)
    env = dict(os.environ, ROS_DOMAIN_ID='94', XDG_DATA_HOME=str(tmp_path / 'application-data'),
               ROS_LOG_DIR=str(tmp_path / 'ros-log'))
    env.pop('Z70_REPO', None)
    context = Context(); rclpy.init(context=context, domain_id=94)
    node = rclpy.create_node('offline_standard_launch_probe', context=context)
    executor = SingleThreadedExecutor(context=context); executor.add_node(node)
    states = []; images = []; maps = []; calibrations = []; odometry = []
    node.create_subscription(RobotState, '/z70/state', states.append, LATCHED)
    node.create_subscription(CompressedImage, '/camera/front/image_raw/compressed', images.append, 2)
    node.create_subscription(OccupancyGrid, '/map', maps.append, LATCHED)
    node.create_subscription(CalibrationState, '/z70/chassis_calibration', calibrations.append, LATCHED)
    node.create_subscription(OdometryStatus, '/z70/odom_status', odometry.append, 10)
    log = (tmp_path / 'standard-launch.log').open('w')
    process = subprocess.Popen(['ros2', 'launch', 'z70_bringup', 'bringup.launch.py',
        'origin:=' + backend['origin'], 'token_file:=' + str(token), 'foxglove_port:=0'],
        cwd=tmp_path, env=env, stdout=log, stderr=log, start_new_session=True)
    forced = False
    try:
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline and not (states and images and maps and calibrations and odometry):
            assert process.poll() is None, 'Standard launch exited during fake acquisition'
            executor.spin_once(timeout_sec=.1)
        assert states and images and maps and calibrations and odometry, 'Required launched ROS observation topics missing'
        assert odometry[-1].evidence.valid and odometry[-1].active_camera_x_known
        assert not odometry[-1].covariance_calibrated
        assert cv2.imdecode(np.frombuffer(bytes(images[-1].data), np.uint8), cv2.IMREAD_COLOR).shape[:2] == (1200, 1600)
        assert backend['frames'] > 0
        assert all(path != '/api/driver/rpc' or body['method'] == 'get_room_mapping'
                   for method, path, body in backend['calls'])
    finally:
        if process.poll() is None: process.send_signal(signal.SIGINT)
        try: process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            forced = True; os.killpg(process.pid, signal.SIGKILL); process.wait()
        log.close(); executor.shutdown(); node.destroy_node(); context.shutdown()
    assert not forced, 'Owned offline launch did not finish bounded standard shutdown'
    assert process.returncode == 0
