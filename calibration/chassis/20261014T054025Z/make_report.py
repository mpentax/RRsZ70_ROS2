"""Generate the partial-trial report from retained measurements."""
import json,math
from pathlib import Path
from datetime import datetime,timezone
P=Path(__file__).resolve().parent
s=json.loads((P/'summary.json').read_text()); f=s['fit']
fit=json.loads((P/'fit.json').read_text())
rows=[]
for p,r,d in zip(json.loads((P/'pairs.json').read_text()),s['ros_comparison'],s['rejection_diagnostics']):
    a=next((a for a in fit['pulses'] if a['id']==p['id']),None)
    rows.append(f"| {p['id']} | {p['duration_ms']} | {p['omega']:+.1f} | {'accepted' if a else 'REJECTED'} | {d['inliers']} | {1000*d['rms_m']:.3f} | {a['dtheta_deg'] if a else 'unavailable'} | {r['yaw_deg']:.4f} | {1000*r['dx_m']:.3f}, {1000*r['dy_m']:.3f} |")
asym=abs(f['yaw_positive']['gain']-f['yaw_negative']['gain'])/((abs(f['yaw_positive']['gain'])+abs(f['yaw_negative']['gain']))/2)*100
pooled=f['yaw']['per_pulse_gain_std']/abs(f['yaw']['gain'])*100
ts=datetime.now(timezone.utc).isoformat()
report=f'''# Chassis odometry trial — 20261014T054025Z

## Outcome: stopped on offline VO rejection; DO NOT ADOPT

Authorized supervised trial with Michael present. Robot was already off dock, arm stowed,
one patterned board flat and fixed ahead, and approximately 1 m clearance confirmed.
Three pulses attempted; three complete image pairs; two accepted, one rejected; zero incomplete attempts.
No ambiguous RPC delivery during motion; no retries, compensating moves, autonomous cleaning, or arm commands.
The first +600 ms turn was rejected, so the remaining nine turns and all six forward pulses were not executed.
The short brief overrides the older protocol: results are committed without images and camera defaults stay unchanged.

## Setup and baseline

Docker app and go2rtc were initially stopped. Started existing containers via `docker compose up -d app go2rtc`;
app became healthy and both restart counts remained zero. MCP supplied every manual robot observation and RPC.
No `.local/` secrets were inspected. The authorized ROS bridge uses its existing credential-file configuration.
Initial fresh status: battery 100%, raw state 3, error 0, not cleaning or returning.
Arm estimate: restored dead-reckoned IN_ROBOT, consistent with Michael's physical confirmation, not joint feedback.
Setup image: one high-contrast patterned board centered ahead, mostly flat; loose objects and storage farther back.
No live person/pet or manipulator visible; the display contains pictured people. Metric clearance relies on Michael.

Baseline accepted: 2636 inliers, 0.722 mm RMS, camera translation
{1000*math.hypot(s['settling'][0]['dx_m'],s['settling'][0]['dy_m']):.4f} mm and yaw {s['settling'][0]['yaw_deg']:.6f} degrees.
All three post-pulse settling checks passed the <2 mm / <0.3 degree requirements.

## Pulse results

| Pair | Duration ms | Omega command | Offline | Inliers | RMS mm | Offline yaw deg | ROS yaw deg | ROS base dx,dy mm |
|---|---:|---:|---|---:|---:|---:|---:|---|
{chr(10).join(rows)}

Offline analysis used `calibration/chassis/analyze.py`, 6000 ORB features, zero camera offset for the initial
camera-motion estimate, unchanged >=80 inliers / <=3 mm RMS. Trace-only offline diagnosis of the exact growing
manifest found the rejected pair had 691 matches, 114 inliers, and 4.119 mm RMS: rejection was the RMS limit,
not the inlier-count limit. No threshold was relaxed and no rejected result was fitted as zero motion.
The rejected pair's displacement is unavailable; live ROS's {s['ros_comparison'][2]['yaw_deg']:.4f} degrees is a separate incremental estimate.

Exception-only VLM inspection of both rejected-pair images found the board centered before and near the right
edge after. It appeared mostly flat, with slight border waviness; no obvious human/pet intrusion or strong motion
blur. Images do not prove board displacement or a unique physical cause of the elevated residual.
Reduced common board coverage / viewpoint change is plausible, not demonstrated as the cause.

## Provisional fit and adoption criteria

- camera_x_m: **{f['camera_x_m']:.6f} m**, fit-only sigma **{f['camera_x_fit_sigma_m']:.6f} m**.
- Rotational circle residual RMS: **{1000*f['camera_rotation_residual_rms_m']:.3f} mm**.
- Positive yaw gain: **{f['yaw_positive']['gain']:.6f}**, n=1.
- Negative yaw gain: **{f['yaw_negative']['gain']:.6f}**, n=1.
- Pooled yaw gain: {f['yaw']['gain']:.6f}; pooled per-pulse gain std {f['yaw']['per_pulse_gain_std']:.6f}
  ({pooled:.2f}% of absolute pooled gain). This is not per-sign scatter.
- Forward gain, scatter, lateral drift and yaw drift: **unavailable**, no forward pulses.
- Duration-group means: 400 ms positive yaw +7.644774 degrees (n=1), negative -7.134266 degrees (n=1).
  600 ms has no accepted offline pair; 800 ms was not attempted. Duration dependence is unavailable.

| Criterion | Observed | Verdict |
|---|---|---|
| >=4 accepted turns each sign | 1 positive, 1 negative | FAIL |
| >=4 accepted forward | 0 | FAIL |
| Circle RMS <=2 mm | {1000*f['camera_rotation_residual_rms_m']:.3f} mm on two turns | Numerical pass only; insufficient sample size |
| Positive/negative gain difference <=10% | {asym:.2f}% relative to mean absolute gain | Provisional numerical pass only |
| Per-sign gain scatter <=20%, no duration trend | Single sample per sign; accepted 400 ms only | NOT EVALUABLE |

Overall adoption **FAIL**. camera_x, per-sign gains and their sigma are provisional, model-conditioned observations.
Slip, lateral camera offset, raised/warped paper, floor calibration and viewpoint selection can bias them.
No defaults changed; no adopted `chassis_measured.json` was created.

## Live ROS end-to-end comparison

Started the actual native `pixi run bringup foxglove_port:=8774` in isolated ROS_DOMAIN_ID=94.
Verified `/odom` and `/z70/odom_status` messages arrived before first pulse; recorder continuously flushed both topics
with receipt stamps, header/status stamps, full poses/covariance and counters to `ros-odom.jsonl`.
The recorder also saved `/z70/floor_mosaic` as local `floor-mosaic.png`, with `mosaic-metadata.json`.

Matched ROS windows use the last recorded odom receipt before each move request and the last receipt no later than
the second after-frame receipt. World dx/dy are rotated into the pre-pulse base frame and yaw wrapped.
These are receipt-time windows, not known exposure-time windows. Exact bounds are in `ros-comparison.jsonl`.
ROS yaw differs from accepted offline pairs by
{(s['ros_comparison'][0]['yaw_deg']-fit['pulses'][0]['dtheta_deg']):+.4f} and
{(s['ros_comparison'][1]['yaw_deg']-fit['pulses'][1]['dtheta_deg']):+.4f} degrees respectively.

Live nodes retain the guessed camera_x=0.16 m and default 3000 features; offline uses 6000 features and fits
camera_x=0.281229 m. Therefore ROS lateral base translations are not directly comparable with the fitted offline
base translations. Incremental live matches and a single whole-pulse pair have different overlap/acquisition windows.
Live acceptance does not override the required offline rejection stop rule.

Recorded {s['ros_status_messages']} status messages, all `ok`, zero rejected/init events delivered to this recorder.
Observed rejection rate **0/{s['ros_status_messages']} = 0%** excluding init. Final node counters were
{s['ros_accepted']} accepted and {s['ros_rejected']} rejected (initial events preceded recorder subscription).
Measured unique status-stamp cadence: **{s['ros_delivered_hz']:.4f} Hz**, not assumed nominal 3 Hz.
Offline whole-pulse pair rejection rate: **1/3**, distinct from the live-frame rate.

## Timing deviations

Tool round-trip orchestration did not achieve the brief's ~10-second pulse cadence.
Actual pulse start intervals: {s['pulse_start_intervals_s']} seconds.
Baseline frame receipt separation: {s['baseline_receipt_interval_s']:.3f} s rather than about 1 s.
Post-pulse after-frame separations: {s['after_pair_receipt_intervals_s']} s rather than about 1 s.
Each bounded pulse was followed by the required >=2-second wait and an explicit stop; RC start had >=5 seconds
before motion. These longer stationary windows passed settling, but should not be described as a tight 10-second loop.
Retained MCP receipt times are not camera exposure times.

## Stop, release and process cleanup

On rejection, sent explicit final `app_rc_stop []`, then `app_rc_end []`; both returned sent/received `["ok"]`.
Every start/move/stop/end request and reply is retained in `rpc.jsonl` (sequence numbers 1,2,3, no reuse).
Post-end status showed battery 99%, error 0, no cleaning/returning, but raw state remained **39** on repeated reads,
including observedAt=1791956767523, instead of returning to initial state 3.
ACKs and stationary images do not establish mode exit; **RC release remains unverified**.
Michael was told to check physical stationarity and remote-control exit. No additional arm or compensating commands.

Sent SIGINT to the owned ROS launch PID and SIGTERM to the owned recorder, retaining the final mosaic.
Verified all recorded owned process PIDs had exited. Stopped the two Docker services started for this trial;
read-back: both exited, restart counts 0. MCP host child was pre-existing and left untouched.
Image files remain local and git-ignored; build/recorder logs and transient latest/pending files are not committed.

robot released: UNVERIFIED — {ts}; stop/end acknowledged, physical RC release not established (last state 39).
'''
(P/'RESULTS.md').write_text(report)
print(report)
