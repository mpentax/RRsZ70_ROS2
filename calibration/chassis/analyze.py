#!/usr/bin/env python3
"""Live ROS odom is primary; whole-pair floor VO is an independent cross-check.
Offline-only manifests remain supported. No hardware or service access.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import cv2
import numpy as np
REPO=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(REPO/'ros2/src/z70_odometry'))
from z70_odometry.floor_vo import FloorCamera,FloorOdometry
def delta(a,b):
    dx=b['x_m']-a['x_m']; dy=b['y_m']-a['y_m']; th=a['yaw_rad']
    return dict(dx_m=math.cos(th)*dx+math.sin(th)*dy,dy_m=-math.sin(th)*dx+math.cos(th)*dy,dtheta_rad=math.atan2(math.sin(b['yaw_rad']-th),math.cos(b['yaw_rad']-th)))



def gain_fit(rows,command_key,motion_key,sign=None):
    selected=[r for r in rows if r[command_key]!=0 and (sign is None or np.sign(r[command_key])==sign)]
    if not selected: return None
    command=np.array([r[command_key]*r['duration_ms']/1000 for r in selected]); observed=np.array([r[motion_key] for r in selected])
    gain=float(command@observed/(command@command)); ratios=observed/command
    durations={str(int(ms)):dict(n=sum(r['duration_ms']==ms for r in selected),mean_gain=float(np.mean([ratio for r,ratio in zip(selected,ratios) if r['duration_ms']==ms]))) for ms in sorted({r['duration_ms'] for r in selected})}
    means=[d['mean_gain'] for d in durations.values()]
    trend=(max(means)-min(means))/abs(gain) if gain else None
    return dict(gain=gain,n=len(selected),residual_rms=float(np.sqrt(np.mean((observed-gain*command)**2))),per_pulse_gain_std=float(np.std(ratios,ddof=1)) if len(ratios)>1 else None,duration_groups=durations,duration_relative_range=trend)


def fit_motion(rows,override=None):
    turns=[r for r in rows if r['velocity']==0 and r['omega']!=0]; forwards=[r for r in rows if r['omega']==0 and r['velocity']!=0]
    fitted=sigma=residual=None
    if turns:
        a=np.array([[math.cos(r['dtheta_rad'])-1,math.sin(r['dtheta_rad'])] for r in turns]); t=np.array([[r['camera_dx_m'],r['camera_dy_m']] for r in turns]); denom=float(np.sum(a*a))
        if denom>1e-6:
            fitted=float(np.sum(a*t)/denom); e=t-a*fitted
            residual=float(np.sqrt(np.mean(np.sum(e*e,axis=1))))
            sigma=float(np.sqrt(np.sum(e*e)/max(1,2*len(turns)-1)/denom))
    x=override if override is not None else fitted
    for r in rows:
        th=r['dtheta_rad']; r['dtheta_deg']=math.degrees(th)
        # Rebase the measured camera motion using the fitted offset (not command integration).
        r['dx_m']=r['camera_dx_m']+(x or 0)*(1-math.cos(th)); r['dy_m']=r['camera_dy_m']-(x or 0)*math.sin(th)
    return dict(camera_x_m=x,camera_x_source='override' if override is not None else 'pure_rotation_fit',rotation_fitted_camera_x_m=fitted,camera_x_fit_sigma_m=sigma,camera_rotation_residual_rms_m=residual,rotation_pairs=len(turns),yaw=gain_fit(turns,'omega','dtheta_rad'),yaw_positive=gain_fit(turns,'omega','dtheta_rad',1),yaw_negative=gain_fit(turns,'omega','dtheta_rad',-1),forward=gain_fit(forwards,'velocity','dx_m'),forward_positive=gain_fit(forwards,'velocity','dx_m',1),forward_negative=gain_fit(forwards,'velocity','dx_m',-1),forward_lateral_rms_m=float(np.sqrt(np.mean([r['dy_m']**2 for r in forwards]))) if forwards else None,forward_yaw_rms_deg=float(np.sqrt(np.mean([r['dtheta_deg']**2 for r in forwards]))) if forwards else None)


def verdict(fit):
    reasons=[]
    for key,minimum in [('yaw_positive',4),('yaw_negative',4),('forward',4)]:
        g=fit[key]
        if not g or g['n']<minimum: reasons.append(f'{key}: need >= {minimum} measured samples')
        if g:
            if g['gain']<=0: reasons.append(f'{key}: nonpositive gain')
            if g['per_pulse_gain_std'] is None or g['per_pulse_gain_std']>.2*abs(g['gain']): reasons.append(f'{key}: scatter unavailable or >20%')
            if len(g['duration_groups'])<3: reasons.append(f'{key}: duration dependence untested')
            elif g['duration_relative_range'] is None or g['duration_relative_range']>.2: reasons.append(f'{key}: duration-group gain range >20%')
    rms=fit['camera_rotation_residual_rms_m']
    if rms is None or rms>.002: reasons.append('rotation-circle RMS unavailable or >2 mm')
    a,b=fit['yaw_positive'],fit['yaw_negative']
    if a and b and abs(a['gain']-b['gain'])>.1*max(abs((a['gain']+b['gain'])/2),1e-12): reasons.append('positive/negative yaw gains differ >10%')
    return dict(decision="DON'T ADOPT" if reasons else 'ADOPT',reasons=reasons or ['all documented sample, residual, asymmetry, scatter and duration criteria pass'],duration_criterion='three durations; range of duration-group means <=20% of fitted gain')


def historical_live(folder,row):
    comparison=folder/'ros-comparison.jsonl'
    if comparison.exists():
        for line in comparison.read_text().splitlines():
            r=json.loads(line)
            if r['id']==row['id']: return dict(dx_m=r['dx_m'],dy_m=r['dy_m'],dtheta_rad=math.radians(r['yaw_deg']),camera_x_m=r['camera_x_m'],window=r['window'],pre_receipt_ns=r['pre_receipt_ns'],post_receipt_ns=r['post_receipt_ns'])
    paths=[folder/(row['id']+f'-ros-{phase}.json') for phase in ['before','after']]
    if all(p.exists() for p in paths):
        a,b=[json.loads(p.read_text())['/odom'] for p in paths]
        return dict(delta(a,b),camera_x_m=.16,window='recorded settled pre/post ROS snapshots',pre_receipt_ns=a['received_ns'],post_receipt_ns=b['received_ns'])
    return None


def analyze(manifest,calibration,camera_x_m=None):
    manifest=Path(manifest); inputs=json.loads(manifest.read_text())
    if not isinstance(inputs,list): raise ValueError('manifest must be a JSON list')
    cam=FloorCamera.load(calibration); vo=FloorOdometry(cam,camera_x_m=0,n_features=6000)
    offline=[]; live=[]; rejected=[]; incomplete=[]
    for i,row in enumerate(inputs):
        item={k:row.get(k) for k in ['before','after','velocity','omega','duration_ms']}; item['id']=str(row.get('id',i))
        for name in ['velocity','omega','duration_ms']:
            item[name]=float(item[name])
            if not math.isfinite(item[name]): raise ValueError('commands must be finite')
        if item['duration_ms']<=0 or (item['velocity']!=0 and item['omega']!=0): raise ValueError('use positive-duration separate straight/rotation pulses')
        if 'odom_before' in row and 'odom_after' in row:
            measured=dict(delta(row['odom_before'],row['odom_after']),camera_x_m=row['odom_before']['camera_x_m'])
        else: measured=historical_live(manifest.parent,item)
        if measured:
            th=measured['dtheta_rad']; old_x=measured['camera_x_m']
            live.append(dict(item,source='live_ros_odom',live_delta=measured,dtheta_rad=th,camera_dx_m=measured['dx_m']+old_x*(math.cos(th)-1),camera_dy_m=measured['dy_m']+old_x*math.sin(th)))
        elif 'odom_before' in row: incomplete.append(dict(item,reason='no settled post-pulse odom'))
        frames=[]; reason=None
        for name in ['before','after']:
            path=manifest.parent/item[name] if item[name] else None
            image=cv2.imread(str(path)) if path and path.exists() else None
            if image is None: reason='missing JPEG'; break
            if image.shape[:2]!=(cam.H,cam.W): reason='calibration/image dimensions differ'; break
            frames.append(image)
        motion=None if reason else vo.estimate(*frames)
        if motion is None: rejected.append(dict(item,reason=reason or 'floor VO rejected')); continue
        offline.append(dict(item,source='whole_pair_vo',camera_dx_m=motion.dx,camera_dy_m=motion.dy,dtheta_rad=motion.dtheta,inliers=motion.inliers,matches=motion.matches,rms_m=motion.rms_m))
    # A missing live pulse never becomes an offline value in an otherwise live trial.
    primary=live if live else ([] if any('odom_before' in r for r in inputs) else offline)
    fit=fit_motion(primary,camera_x_m); cross_fit=fit_motion(offline,camera_x_m)
    by_id={r['id']:r for r in offline}
    for r in live:
        cross=by_id.get(r['id']); r['cross_check']=cross
        if cross: r['cross_check_difference']=dict(yaw_deg=r['dtheta_deg']-cross['dtheta_deg'],dx_m=r['dx_m']-cross['dx_m'],dy_m=r['dy_m']-cross['dy_m'])
    return dict(input_pairs=len(inputs),accepted_pairs=len(offline),rejected_pairs=len(rejected),live_pulses=len(live),primary_source='live_ros_odom' if live or any('odom_before' in r for r in inputs) else 'whole_pair_vo_only',fit=fit,offline_cross_check_fit=cross_fit,pulses=primary,rejected=rejected,incomplete=incomplete,verdict=verdict(fit),model='gains multiply command times seconds; camera_y=0; pure rotations assumed. Live camera motion recovered by undoing recorded ROS offset; fit uncertainty excludes calibration error/slip. VO failures do not remove live measurements.')


def write_report(result,folder):
    folder=Path(folder); folder.mkdir(parents=True,exist_ok=True)
    (folder/'fit.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    f=result['fit']; v=result['verdict']; lines=['# Chassis trial results','',f"**{v['decision']}** — "+'; '.join(v['reasons']),'',f"Primary: {result['primary_source']}. Attempted pairs: {result['input_pairs']}; live measurements: {result['live_pulses']}; whole-pair VO accepted/rejected: {result['accepted_pairs']}/{result['rejected_pairs']}.",'',f"camera_x_m: {f['camera_x_m']}; fit sigma: {f['camera_x_fit_sigma_m']} m; rotation-circle RMS: {f['camera_rotation_residual_rms_m']} m.",'','| Fit | n | Gain | Per-pulse gain std | Duration means (ms: gain) |','|---|---:|---:|---:|---|']
    for key in ['yaw_positive','yaw_negative','forward','forward_positive','forward_negative']:
        g=f[key]
        lines.append(f"| {key} | {g['n']} | {g['gain']:.6f} | {g['per_pulse_gain_std']} | "+', '.join(f"{ms}: {d['mean_gain']:.6f}" for ms,d in g['duration_groups'].items())+' |' if g else f'| {key} | 0 | unavailable | unavailable | unavailable |')
    lines+=['',f"Forward lateral RMS: {f['forward_lateral_rms_m']} m; yaw RMS: {f['forward_yaw_rms_deg']} deg.",'','| Pulse | ms | Live/primary yaw deg | Fitted-base dx m | dy m | Pair VO yaw deg |','|---|---:|---:|---:|---:|---:|']
    for r in result['pulses']:
        c=r.get('cross_check'); lines.append(f"| {r['id']} | {r['duration_ms']:g} | {r['dtheta_deg']:.5f} | {r['dx_m']:.6f} | {r['dy_m']:.6f} | "+(f"{c['dtheta_deg']:.5f}" if c else 'unavailable')+' |')
    lines+=['','VO rejected/missing: '+', '.join(r['id']+': '+r['reason'] for r in result['rejected']), '',result['model'],'', 'Duration trend criterion: '+v['duration_criterion']+'. No constants are automatically changed.','Physical release: UNVERIFIED (RPC delivery is not physical completion).']
    (folder/'RESULTS.md').write_text('\n'.join(line.rstrip() for line in lines)+'\n')
    if v['decision']=='ADOPT':
        (folder/'chassis_measured.json').write_text(json.dumps(dict(camera_x_m=f['camera_x_m'],yaw_positive_gain=f['yaw_positive']['gain'],yaw_negative_gain=f['yaw_negative']['gain'],forward_gain=f['forward']['gain'],source='live measured calibration; review before applying'),indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('manifest',type=Path); parser.add_argument('--calibration',type=Path,default=REPO/'calibration/front-camera/front_camera.json'); parser.add_argument('--camera-x-m',type=float); parser.add_argument('--output',type=Path)
    args=parser.parse_args(); result=analyze(args.manifest,args.calibration,args.camera_x_m)
    write_report(result,args.output.parent if args.output else args.manifest.parent)
    if args.output and args.output.name!='fit.json': args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,indent=2,allow_nan=False))

if __name__=='__main__': main()
