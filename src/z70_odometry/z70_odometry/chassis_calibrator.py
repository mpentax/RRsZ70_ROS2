"""Online RC/odometry chassis pulse gain and camera offset calibration."""
from collections import deque
import json
import hashlib
import tempfile
import math
import os
from pathlib import Path
import time
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from z70_interfaces.msg import CalibrationState, CommandEvent, OdometryStatus
from z70_odometry.calibration_state import CalibrationPublication
from z70_odometry.resources import runtime_calibration_path, seed_directory


def local_delta(a,b):
    dx=b['x']-a['x']; dy=b['y']-a['y']; th=a['yaw']
    return math.cos(th)*dx+math.sin(th)*dy,-math.sin(th)*dx+math.cos(th)*dy,math.atan2(math.sin(b['yaw']-th),math.cos(b['yaw']-th))


def quat_yaw(q) -> float:
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def stamp_ns(stamp) -> int:
    s = getattr(stamp, 'header', stamp)
    s = getattr(s, 'stamp', s)
    return s.sec * 10**9 + s.nanosec


def _arc(angle: float) -> tuple[float, float]:
    return (math.sin(angle)/angle, (1.-math.cos(angle))/angle) if abs(angle)>1e-8 else (1., 0.)


def slope(rows,command,motion,sign=None):
    rows=[r for r in rows if r[command] and (sign is None or math.copysign(1,r[command])==sign)]
    if not rows: return dict(gain=None,sigma=None,n=0)
    a=np.array([r[command] for r in rows]); b=np.array([r[motion] for r in rows]); denominator=float(a@a)
    gain=float(a@b/denominator); residual=b-a*gain
    sigma=float(math.sqrt(float(residual@residual)/(len(rows)-1)/denominator)) if len(rows)>1 else None
    return dict(gain=gain,sigma=sigma,n=len(rows))


class OnlineCalibration:
    def __init__(self,path,settle_s=2,max_gap_s=1.5,window=500,min_camera_n=4,max_camera_sigma_m=.01):
        self.path=Path(path); self.settle_s=settle_s; self.max_gap_s=max_gap_s; self.window=window
        self.min_camera_n=min_camera_n; self.max_camera_sigma_m=max_camera_sigma_m
        self.samples=[]; self.seen=[]; self.seeded=[]; self.skipped=0; self.skip_reasons={}; self.pending=[]
        self.poses=deque(maxlen=1000); self.events=deque(maxlen=1000)
        self.updated_at=None; self.calibration_id=''; self._saved_payload=None
        if self.path.exists():
            content=self.path.read_bytes(); data=json.loads(content)
            self.calibration_id=hashlib.sha256(content).hexdigest(); self.updated_at=data.get('updated_at')
            self._saved_payload={k:v for k,v in data.items() if k!='updated_at'}
            self.samples=data.get('samples',[])[-window:]; self.seen=data.get('seen',[])[-2048:]
            self.skipped=data.get('skipped',0); self.skip_reasons=data.get('skip_reasons',{}); self.seeded=data.get('seeded',[r['id'] for r in self.samples if r['id'].startswith('seed:')])
    def skip(self,reason):
        self.skipped+=1; self.skip_reasons[reason]=self.skip_reasons.get(reason,0)+1
    def add_pose(self,pose):
        if all(math.isfinite(pose[k]) for k in ['t','x','y','yaw']): self.poses.append(dict(pose))
    def add_command(self,command):
        ident=command.get('id')
        if not isinstance(ident,str) or ident in self.seen: return
        self.seen=(self.seen+[ident])[-2048:]
        self.events.append(command)
        if command.get('method')!='app_rc_move': return
        try:
            p=command['params'][0]; start=float(command['sentAt'])/1000; duration=float(p['duration'])/1000
            velocity=float(p['velocity']); omega=float(p['omega'])
            if not all(math.isfinite(x) for x in [start,duration,velocity,omega]) or duration<=0: raise ValueError()
        except (KeyError,TypeError,ValueError,IndexError): self.skip('invalid observation'); self.save(); return
        if command.get('dispatch')!='sent' or command.get('response')!='received':
            self.skip('uncertain/rejected dispatch'); self.save(); return
        self.pending.append(dict(id=ident,start=start,end=start+duration,velocity=velocity,omega=omega,duration=duration))
    def process(self,now):
        changed=False
        for command in list(self.pending):
            start=command['start']; settled=command['end']+self.settle_s
            before=[p for p in self.poses if p['t']<=start]
            after=[p for p in self.poses if p['t']>=settled]
            if not after and now<settled+5: continue
            self.pending.remove(command); changed=True
            if not before or not after:
                self.skip('silent/missing odom'); continue
            a=before[-1]; b=after[0]; points=[p for p in self.poses if a['t']<=p['t']<=b['t']]
            if start-a['t']>self.max_gap_s or b['t']-settled>self.max_gap_s or any(q['t']-p['t']>self.max_gap_s for p,q in zip(points,points[1:])):
                self.skip('silent/missing odom'); continue
            if any(p.get('rejected',False) for p in points): self.skip('rejected odom'); continue
            offsets={p.get('camera_x_m') for p in points}
            if None in offsets and now<settled+5:
                self.pending.append(command); continue
            if None in offsets or len(offsets)!=1: self.skip('unknown/changing odom camera offset'); continue
            interrupted=False
            for event in self.events:
                if event.get('id')==command['id']: continue
                t=event.get('sentAt')
                if t is None: continue
                t=float(t)/1000; method=event.get('method')
                if method=='app_rc_move' and t<start:
                    try:
                        if t+float(event['params'][0]['duration'])/1000+self.settle_s>start: interrupted=True
                    except (KeyError,IndexError,TypeError,ValueError): pass
                if a['t']<=t<=b['t'] and (method=='app_rc_move' or (method in ['app_rc_stop','app_rc_end'] and t<command['end'])):
                    interrupted=True
            if interrupted: self.skip('overlapping/interrupted motion'); continue
            dx,dy,yaw=local_delta(a,b); old=next(iter(offsets))
            yaw=sum(math.atan2(math.sin(q['yaw']-p['yaw']),math.cos(q['yaw']-p['yaw'])) for p,q in zip(points,points[1:]))
            self.add_measurement(dict(id=command['id'],command_yaw=command['omega']*command['duration'],command_forward=command['velocity']*command['duration'],yaw=yaw,camera_dx=dx+old*(math.cos(yaw)-1),camera_dy=dy+old*math.sin(yaw),start=start,end=b['t']))
        if changed: self.save()
    def add_measurement(self,row):
        if any(r['id']==row['id'] for r in self.samples): return
        self.samples=(self.samples+[row])[-self.window:]
    def summary(self):
        rows=[dict(r) for r in self.samples]; turn_count=sum(abs(r['yaw'])>1e-5 for r in rows)
        x=sigma=residual=None; forward=dict(gain=None,sigma=None,n=0)
        if rows:
            matrix=[]; target=[]
            for r in rows:
                angle=r['yaw']; arc_x, arc_y = _arc(angle)
                matrix.extend([[math.cos(angle)-1,r['command_forward']*arc_x],[math.sin(angle),r['command_forward']*arc_y]])
                target.extend([r['camera_dx'],r['camera_dy']])
            A=np.array(matrix); b=np.array(target); has_forward=any(r['command_forward'] for r in rows)
            if not has_forward: A=A[:,:1]
            coefficients,_,rank,_=np.linalg.lstsq(A,b,rcond=None)
            error=b-A@coefficients
            if rank==A.shape[1] and turn_count:
                x=float(coefficients[0]); residual=float(np.sqrt(np.mean(error**2)))
                if len(b)>rank and turn_count>1:
                    covariance=float(error@error)/(len(b)-rank)*np.linalg.pinv(A.T@A)
                    sigma=float(np.sqrt(covariance[0,0]))
            for r in rows:
                angle=r['yaw']; arc_x, arc_y = _arc(angle)
                dx=r['camera_dx']-(x or 0)*(math.cos(angle)-1); dy=r['camera_dy']-(x or 0)*math.sin(angle)
                r['forward']=(dx*arc_x+dy*arc_y)/(arc_x*arc_x+arc_y*arc_y) if arc_x*arc_x+arc_y*arc_y>1e-10 else 0.
            forward=slope(rows,'command_forward','forward')
        ready=x is not None and sigma is not None and turn_count>=self.min_camera_n and sigma<=self.max_camera_sigma_m
        return dict(n=len(rows),skipped=self.skipped,skip_reasons=dict(self.skip_reasons),yaw_positive=slope(rows,'command_yaw','yaw',1),yaw_negative=slope(rows,'command_yaw','yaw',-1),forward=forward,camera_x=dict(value_m=x,sigma_m=sigma,n=turn_count,residual_rms_m=residual,ready=ready),model='windowed_unicycle_arc_ls')
    def save(self):
        payload=dict(version=1,samples=self.samples,seen=self.seen,seeded=self.seeded,**self.summary())
        if payload==self._saved_payload:
            return
        updated_at=time.time()
        content=(json.dumps(dict(payload,updated_at=updated_at),indent=2,allow_nan=False)+'\n').encode()
        self.path.parent.mkdir(parents=True,exist_ok=True)
        temporary=None
        try:
            with tempfile.NamedTemporaryFile(dir=self.path.parent,prefix=self.path.name+'.',suffix='.tmp',delete=False) as stream:
                temporary=Path(stream.name); stream.write(content); stream.flush(); os.fsync(stream.fileno())
            os.replace(temporary,self.path)
            directory=os.open(self.path.parent,os.O_RDONLY)
            try: os.fsync(directory)
            finally: os.close(directory)
        finally:
            if temporary is not None: temporary.unlink(missing_ok=True)
        self.updated_at=updated_at; self.calibration_id=hashlib.sha256(content).hexdigest()
        self._saved_payload=json.loads(content); self._saved_payload.pop('updated_at')
    def seed(self,root):
        root=Path(root)
        for path in sorted(root.glob('2026101*/fit.json')):
            data=json.loads(path.read_text())
            for row in data.get('pulses',[]):
                measured=row.get('live_delta')
                if not measured: continue
                ident='seed:'+path.parent.name+':'+row['id']
                if ident in self.seeded: continue
                self.seeded.append(ident)
                yaw=measured['dtheta_rad']; old=measured['camera_x_m']
                self.add_measurement(dict(id=ident,command_yaw=row['omega']*row['duration_ms']/1000,command_forward=row['velocity']*row['duration_ms']/1000,yaw=yaw,camera_dx=measured['dx_m']+old*(math.cos(yaw)-1),camera_dy=measured['dy_m']+old*math.sin(yaw),source=str(path.relative_to(root))))
        self.save()


class CommandStream:
    """Pair dispatch-start and reply CommandEvents by attempt_id."""
    def __init__(self, estimator):
        self.estimator=estimator; self.session=None; self.sequence=None; self.attempts={}

    def add(self, msg):
        session=bytes(msg.evidence.session_id.uuid); seq=msg.evidence.sequence
        if self.session==session and self.sequence is not None and seq<=self.sequence: return
        if self.session is not None and (self.session!=session or seq!=self.sequence+1):
            self.attempts.clear(); self.estimator.pending.clear(); self.estimator.skip('command event gap')
        self.session=session; self.sequence=seq
        ident=session.hex()+':'+bytes(msg.outcome.attempt_id.uuid).hex()
        if msg.phase==1 and msg.evidence.source_time_known and msg.evidence.source_clock==1:
            t=msg.evidence.source_at.sec*1000+msg.evidence.source_at.nanosec/1e6
            self.attempts[ident]=t
            while len(self.attempts)>256: self.attempts.pop(next(iter(self.attempts)))
        elif msg.phase==2:
            start=self.attempts.pop(ident,None)
            if start is None: return
            duration=msg.requested_duration.sec*1000+msg.requested_duration.nanosec/1e6
            self.estimator.add_command(dict(id=ident,method=msg.operation,
                params=[dict(velocity=msg.requested_twist.linear.x,omega=msg.requested_twist.angular.z,duration=duration)],
                sentAt=start, dispatch={0:'not_sent',1:'sent',2:'unknown'}[msg.outcome.dispatch],
                response={0:'none',1:'received',2:'vendor_error',3:'transport_error'}[msg.outcome.response]))


def main():
    class Calibrator(Node):
        def __init__(self):
            super().__init__('chassis_calibrator')
            path=self.declare_parameter('output_file',str(runtime_calibration_path())).value
            seeds=self.declare_parameter('seed_directory',str(seed_directory())).value
            self.estimator=OnlineCalibration(path,settle_s=float(self.declare_parameter('settle_s',2.).value),max_gap_s=float(self.declare_parameter('max_gap_s',1.5).value),min_camera_n=int(self.declare_parameter('min_camera_n',4).value),max_camera_sigma_m=float(self.declare_parameter('max_camera_sigma_m',.01).value))
            if seeds: self.estimator.seed(seeds)
            else: self.estimator.save()
            self.publication=CalibrationPublication()
            self.command_stream=CommandStream(self.estimator)
            self.status={}
            self.pub=self.create_publisher(CalibrationState,'/z70/chassis_calibration',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(CommandEvent,'/z70/commands',self.commands,256)
            self.create_subscription(Odometry,'/odom',self.odom,100)
            self.create_subscription(OdometryStatus,'/z70/odom_status',self.quality,100)
            self.create_timer(.2,self.update)
        def commands(self,msg):
            self.command_stream.add(msg)
        def odom(self,msg):
            p=msg.pose.pose; stamp=stamp_ns(msg); t=stamp/1e9
            entry=dict(t=t,stamp_ns=stamp,x=p.position.x,y=p.position.y,yaw=quat_yaw(p.orientation),rejected=msg.pose.covariance[0]>1000,camera_x_m=None)
            status=self.status.get(stamp)
            if status: entry['camera_x_m']=status.get('camera_x_m')
            self.estimator.add_pose(entry)
        def quality(self,msg):
            if not msg.evidence.valid or msg.evidence.stale: return
            stamp=stamp_ns(msg)
            camera_x=msg.active_camera_x_m if msg.active_camera_x_known and math.isfinite(msg.active_camera_x_m) else None
            status=dict(stamp_ns=stamp,camera_x_m=camera_x,state='ok' if msg.pair_accepted else 'rejected')
            self.status[stamp]=status
            while len(self.status)>1000: self.status.pop(next(iter(self.status)))
            for p in reversed(self.estimator.poses):
                if p.get('stamp_ns')==stamp:
                    p['camera_x_m']=status.get('camera_x_m'); p['rejected']=p['rejected'] or status.get('state')=='rejected'; break
        def update(self):
            self.estimator.process(self.get_clock().now().nanoseconds/1e9)
            self.estimator.save()
            self.pub.publish(self.publication.snapshot(self.estimator,self.get_clock().now().to_msg()))
    rclpy.init(); node=Calibrator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.estimator.save(); node.destroy_node(); rclpy.try_shutdown()

if __name__=='__main__': main()
