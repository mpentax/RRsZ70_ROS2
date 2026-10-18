> **Historical / superseded guidance — 2026-10-18 curation.** Original body below is preserved verbatim. Measurements, failed fits, cleanup uncertainty, test counts and commands describe only the dated experiment/source revision, not current runtime state or reusable authorization. Current guidance: [odometry guide](../../../src/z70_odometry/README.md), [camera guidance](../../../research/cameras/FINDINGS.md).

# Supervised chassis odometry trial — 20261013T151331Z

## Outcome: aborted after first pulse; do not adopt constants

Michael confirmed presence, physical arm stow, flat/fixed boards 0.15–0.8 m ahead, approximately 1 m clearance with the cat removed, and acceptance of undocking/RC mode. The setup image showed the boards and no identifiable cat or person. No arm commands, compensating moves, retries, autonomous cleaning, or direct robot HTTP/shell calls were issued.

The first scheduled positive turn (`velocity=0`, `omega=+0.5`, `duration=400`, `seqnum=1`) returned `dispatch=sent`, `response=received`, `result=["ok"]`. However, neither the offline pair nor ROS showed a meaningful turn. Fresh status changed from 3 before RC start to 39 after the pulse, with no reported error; state 39 is the remote-pickup state, not proof of a completed chassis RC transition. The physical response to the requested pulse was therefore unclear. The trial stopped before the next scheduled pulse, without repeating the command. No root cause was established; an ACK does not establish execution or measured pulse gain.

Counts: 1 motion attempt of 18 scheduled; 1 complete pair; 1 VO-accepted pair; 0 VO-rejected pairs; 0 incomplete attempts. VO acceptance here means a valid near-stationary image registration, not successful commanded motion.

## Baseline and settling

- Baseline: accepted, 3,074 inliers, RMS 0.399 mm, camera dx +0.00179 mm, dy +0.00613 mm, yaw +0.000472°. Passed translation <2 mm and |yaw| <0.3°.
- Pre-pulse settled comparison: accepted, 3,067 inliers, RMS 0.373 mm, yaw −0.004794°; passed.
- Post-pulse settling: accepted, 3,047 inliers, RMS 0.382 mm, camera dx +0.00544 mm, dy −0.01785 mm, yaw +0.001697°; passed.
- Final observation versus settled after-frame: accepted, 2,916 inliers, RMS 0.400 mm, yaw −0.006857°; passed the stationary check. This does not prove remote-control release.

All frames are 1600×1200 and retained locally. Receipt timestamps are not known exposure timestamps; lens identity and capture provenance remain unverified. VLM was used for setup and for the anomalous near-zero pulse response, not routine per-pulse analysis. It found no obvious immediate obstruction or displaced boards explaining the result.

## Offline pair and live ROS comparison

| Measurement | Offline before/after pair | Live ROS matched receipt window |
|---|---:|---:|
| dx | camera −0.00761 mm | guessed-offset base +0.09037 mm |
| dy | camera −0.00236 mm | guessed-offset base −0.17636 mm |
| yaw | −0.000503° | +0.033989° |
| Pair quality | 3,170 inliers; RMS 0.376 mm | no rejection in pulse window |

Both observations indicate no meaningful turn, rather than the requested positive turn. They do not establish a true zero command gain.

ROS pre-pose receipt: 1791904539218167000 ns; pulse RPC start: 1791904539597 ms; ROS post-pose receipt: 1791904566546622000 ns; settled after-frame receipt: 1791904566717 ms. ROS world translation was rotated into the pre-pulse base frame and yaw wrapped. See `ros-comparison.json` for exact stamps. Offline uses camera offset zero and 6,000 ORB features; live ROS uses the unchanged guessed offset 0.16 m and 3,000 features, accumulating successive frames. Their translations are therefore not directly equivalent calibrated base measurements.

The brief's tight-loop timing was not achieved: the first after-frame arrived 14.912 s after RPC start, and the two after-frame receipts were 12.208 s apart rather than about 1 s. Baseline receipts were 13.817 s apart. The prescribed 5 s RC-start wait and minimum 2 s post-reply wait were performed, but tool-turn overhead added substantial delay before explicit stop and capture. No unlogged motion was sent. These deviations weaken temporal comparison and are recorded rather than presented as a nominal 10 s pulse loop.

## Fit and adoption criteria

`fit.json` includes the observed pair, but the fitted camera offset is unidentifiable: yaw excitation is below the analyzer's identifiability threshold. Its `camera_x_m=0` is an explicit analysis override for reporting camera displacement, not a measured offset or replacement default. The fit is marked `unidentifiable_fit=true`.

- camera_x ± fit sigma: unavailable.
- Rotation circle RMS: unavailable.
- Positive yaw sample count: 1. The raw numerical ratio is −0.0000439065, noise-level and not a validated response gain; scatter unavailable.
- Negative yaw gain/scatter: unavailable; 0 samples.
- Forward gain/scatter, lateral drift and yaw drift: unavailable; 0 samples.
- Duration-group means: only one +400 ms near-stationary pair; its yaw is −0.000503°. No duration dependence can be assessed.

Adoption requires ≥4 accepted samples in each turn direction and forward, circle RMS ≤2 mm, per-sign yaw gains within 10%, per-pulse gain scatter ≤20%, and no systematic duration trend. Counts fail (1 positive, 0 negative, 0 forward); the remaining criteria are unavailable, not passing. Do not adopt the numerical ratio or offset override. No camera_x defaults or pulse constants were changed, and no measured-constants file was created.

## Live recording and cleanup

The trial started `pixi run bringup` with isolated ROS_DOMAIN_ID=93 and Foxglove port 55008, alongside a passive rclpy recorder. Both `/odom` and `/z70/odom_status` were verified before motion. `ros-odom.jsonl` covers the trial and cleanup observations. Final `/z70/floor_mosaic` image is retained as `floor-mosaic.png` with metadata; it is an appearance mosaic from guessed-offset live odometry, not calibrated navigation data.

Recorded status: 1 init and 482 ok events; 0 rejected. Live VO rejection rate excluding init: 0/482 = 0%. Delivered status rate from unique image stamps: 2.939 Hz. This stationary/near-stationary rate does not validate odometry during actual chassis motion.

Every RPC, including the initial per-pulse stop and final stop/end, is retained in `rpc.jsonl` with exact params and full replies. Reply start/finish times are authoritative command timestamps; the outer logging timestamp can be later because replies were logged in the following tool turn.

Final `app_rc_stop` and `app_rc_end` both returned received `ok` ACKs. Fresh status at 2026-10-13T15:17:11.920Z still reported state 39, battery 73%, error 0, not cleaning or returning. Physical/mode release remains unverified. Michael was asked to physically stop if moving and exit remote-control/pickup mode in the app. No additional mode-switch or arm command was sent.

Both trial-owned background jobs were terminated. A process-list check found no surviving owned launch/node PIDs or session recorder. Pre-existing ROS processes were left untouched. Images are excluded from the result commit; unrelated untracked files are untouched.

robot released: UNVERIFIED — 2026-10-13T15:17:11.920Z; stop/end ACKed, but status remained 39 and no physical release confirmation was received.
