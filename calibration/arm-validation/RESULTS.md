> **Historical / superseded guidance — 2026-10-18 curation.** Original body below is preserved verbatim. Measurements, failed fits, cleanup uncertainty, test counts and commands describe only the dated experiment/source revision, not current runtime state or reusable authorization. Current guidance: [current arm model](../../docs/z70-arm-model.md), [camera/calibration guide](../../research/cameras/FINDINGS.md).

| Snapshot | Value |
| --- | --- |
| Scope | PARTIAL snapshot; final acquisition is not complete |
| Log | /Users/mpentax/Code/RoboRockZ70-Connector/calibration/arm-validation/log.jsonl |
| Session | /Users/mpentax/Code/RoboRockZ70-Connector/calibration/arm-validation/session.json |
| Log bytes read once | 80651 |
| Parsed records | 84 |
| Malformed/truncated lines excluded | 0 |
| Last parsed timestamp | 2026-10-13T11:41:31.725000+00:00 |
| Session phase/status | limited_front_camera_pose_validation / limited_wrist_sweep_complete |
| Report black-side filter (m) | 0.015 |
| Accepted pose endpoints | 8 |
| Accepted measured command deltas | 5 |
| Rejected/unavailable evidence rows | 30 |
| Chassis pulses logged / accepted | 0 / 0 |
| App model state | Never used as measured pose or to infer offsets |
| Intrinsics caveat | Fitted from ONE image (frame-3, Telegram re-encode), two coplanar A4 ChArUco boards on the floor. Floor-plane use only. See README.md for accuracy. |

| Measured command line | Move | Marker dx (m) | Marker dy (m) | Marker dz (m) | Marker displacement norm (m) | Relative rotation magnitude (deg) |
| --- | --- | --- | --- | --- | --- | --- |
| 78 | M5 + | -0.0017898 | 0.000233713 | -0.000566201 | 0.00189172 | 13.9143 |
| 79 | M5 + | -0.00224831 | -0.00106854 | 0.00183137 | 0.00309041 | 10.9507 |
| 80 | M5 - | 0.00173368 | 0.000748268 | -0.00121962 | 0.0022479 | 9.09446 |
| 81 | M5 - | 0.00214892 | 0.000500692 | -0.000752342 | 0.00233121 | 12.2595 |
| 82 | M5 - | 0.00179737 | 4.30178e-05 | 0.00011093 | 0.0018013 | 13.508 |

| Step translation | Mean (m) | Sample std (m) | n | Measured / app-predicted | Interpretation |
| --- | --- | --- | --- | --- | --- |
| forward | unavailable | unavailable | 0 | unavailable | marker-centre displacement, NOT wrist displacement |
| back | unavailable | unavailable | 0 | unavailable | marker-centre displacement, NOT wrist displacement |
| down | unavailable | unavailable | 0 | unavailable | marker-centre displacement, NOT wrist displacement |
| up | unavailable | unavailable | 0 | unavailable | marker-centre displacement, NOT wrist displacement |

| Step rotation | Mean (deg) | Sample std (deg) | n | Measured / app-predicted | Interpretation |
| --- | --- | --- | --- | --- | --- |
| M5 + | 12.4325 | 2.09561 | 2 | 1.24325 | pure single-joint relative rotation magnitude; sign is command sign |
| M5 - | 11.6207 | 2.27509 | 3 | 1.16207 | pure single-joint relative rotation magnitude; sign is command sign |
| M4 + | unavailable | unavailable | 0 | unavailable | pure single-joint relative rotation magnitude; sign is command sign |
| M4 - | unavailable | unavailable | 0 | unavailable | pure single-joint relative rotation magnitude; sign is command sign |
| M3 + | unavailable | unavailable | 0 | unavailable | pure single-joint relative rotation magnitude; sign is command sign |
| M3 - | unavailable | unavailable | 0 | unavailable | pure single-joint relative rotation magnitude; sign is command sign |

| Coverage | Commands logged (incl. setup) | Measurement-phase commands | Accepted deltas | Minimum planned deltas | Status |
| --- | --- | --- | --- | --- | --- |
| forward | 25 | 0 | 0 | 9 | partial |
| back | 1 | 1 | 0 | 9 | partial |
| down | 20 | 1 | 0 | 9 | partial |
| up | 0 | 0 | 0 | 9 | partial |
| M5 + | 3 | 3 | 2 | 6 | partial |
| M5 - | 3 | 3 | 3 | 6 | partial |
| M4 + | 0 | 0 | 0 | 4 | partial |
| M4 - | 0 | 0 | 0 | 4 | partial |
| M3 + | 0 | 0 | 0 | 4 | partial |
| M3 - | 0 | 0 | 0 | 4 | partial |

| Slack | Unit | Mean loss | Sample std | Reversals with steady data | Equivalent commands | Definition |
| --- | --- | --- | --- | --- | --- | --- |
| forward | m | unavailable | unavailable | 0 | unavailable | steady mean minus first reversal step; equivalent loss is not a confirmed extra-command count |
| back | m | unavailable | unavailable | 0 | unavailable | steady mean minus first reversal step; equivalent loss is not a confirmed extra-command count |
| down | m | unavailable | unavailable | 0 | unavailable | steady mean minus first reversal step; equivalent loss is not a confirmed extra-command count |
| up | m | unavailable | unavailable | 0 | unavailable | steady mean minus first reversal step; equivalent loss is not a confirmed extra-command count |
| M5 + | deg | unavailable | unavailable | 0 | unavailable | steady mean minus first reversal step; equivalent loss is not a confirmed extra-command count |
| M5 - | deg | 3.78932 | unavailable | 1 | 0.294116 | steady mean minus first reversal step; equivalent loss is not a confirmed extra-command count |
| M4 + | deg | unavailable | unavailable | 0 | unavailable | steady mean minus first reversal step; equivalent loss is not a confirmed extra-command count |
| M4 - | deg | unavailable | unavailable | 0 | unavailable | steady mean minus first reversal step; equivalent loss is not a confirmed extra-command count |
| M3 + | deg | unavailable | unavailable | 0 | unavailable | steady mean minus first reversal step; equivalent loss is not a confirmed extra-command count |
| M3 - | deg | unavailable | unavailable | 0 | unavailable | steady mean minus first reversal step; equivalent loss is not a confirmed extra-command count |

| Deployed-pose repeatability | Distinct accepted cycles | Max pairwise spread (mm) | Camera xyz sample std (mm) | Coverage | Method |
| --- | --- | --- | --- | --- | --- |
| unavailable | 0 | unavailable | unavailable | 0 / 3 | No explicitly tagged accepted endpoints |

| Chassis pulse | Gain unit | Command | Duration (s) | Mean gain | Sample std | n | Evidence |
| --- | --- | --- | --- | --- | --- | --- | --- |
| rotation | deg / (omega*s) | unavailable | unavailable | unavailable | unavailable | 0 | No accepted board_pose_before/after with common board_id, metric pose and <=2 px RMS |
| translation | mm / (velocity*s) | unavailable | unavailable | unavailable | unavailable | 0 | No accepted board_pose_before/after with common board_id, metric pose and <=2 px RMS |

| Chassis duration linearity | n | Durations | Affine slope | Intercept | Residual RMS (deg or mm) | Decision |
| --- | --- | --- | --- | --- | --- | --- |
| rotation | 0 | 0 | unavailable | unavailable | unavailable | inconclusive: require at least 3 durations and 6 accepted pulses for a descriptive fit |
| translation | 0 | 0 | unavailable | unavailable | unavailable | inconclusive: require at least 3 durations and 6 accepted pulses for a descriptive fit |

| Offset-dependent result | Value | Reason |
| --- | --- | --- |
| Wrist positions / wrist-step constants | unavailable | Independent full marker-to-wrist transform unavailable |
| Arm-derived camera extrinsics / fit residuals | unavailable | No measured wrist positions; app_model_state is a prediction, not independent calibration evidence |
| Pure joint rotations | offset not required | Requires rigid marker downstream of jogged joint; upstream attachment does not observe that joint. Low motion alone is not a valid zero-step measurement. |

| Decision | Outcome / rule |
| --- | --- |
| Protocol rule | Adopt as-is only if EVERY step ratio is 0.85–1.15 and sample std <= 20% of mean, with planned direction/joint/sign coverage |
| Current decision | INCONCLUSIVE: criteria lack planned coverage; wrist-step adoption and arm-derived extrinsics unavailable; partial acquisition |
| Measured constants | unavailable; no YAML or vendor constants adopted/corrected |
| Direction repeat interpretation | Initial 3-step block plus two repeats: minimum 9 per direction; M5 6 per sign; M3/M4 4 per sign |
| Zero versus unavailable | Counts may be zero; unavailable measurements are never replaced by zero motion |

| Decision criterion | Step ratio | Sample dispersion | Applicability |
| --- | --- | --- | --- |
| forward | unavailable: fewer than two accepted steps | unavailable: fewer than two accepted steps | marker-space only; wrist criterion blocked by unknown transform |
| back | unavailable: fewer than two accepted steps | unavailable: fewer than two accepted steps | marker-space only; wrist criterion blocked by unknown transform |
| down | unavailable: fewer than two accepted steps | unavailable: fewer than two accepted steps | marker-space only; wrist criterion blocked by unknown transform |
| up | unavailable: fewer than two accepted steps | unavailable: fewer than two accepted steps | marker-space only; wrist criterion blocked by unknown transform |
| M5 + | outside 0.85–1.15 | within 20% | pure joint orientation; wrist offset not required |
| M5 - | outside 0.85–1.15 | within 20% | pure joint orientation; wrist offset not required |
| M4 + | unavailable: fewer than two accepted steps | unavailable: fewer than two accepted steps | pure joint orientation; wrist offset not required |
| M4 - | unavailable: fewer than two accepted steps | unavailable: fewer than two accepted steps | pure joint orientation; wrist offset not required |
| M3 + | unavailable: fewer than two accepted steps | unavailable: fewer than two accepted steps | pure joint orientation; wrist offset not required |
| M3 - | unavailable: fewer than two accepted steps | unavailable: fewer than two accepted steps | pure joint orientation; wrist offset not required |

| Quality rule | Definition |
| --- | --- |
| Acceptance | Explicit accepted tag or accepted analysis, verified rigid/planar target, known black side, marker identity, two-frame <=1 px settling and <=2 px reprojection RMS |
| Joint low-motion screen | Angles <= max(1 degree, 3x endpoint settling-rotation noise) excluded as unresolved, not measured zero; not a vendor accuracy criterion |
| Repeatability tags | repeatability: {group: identical physical reset endpoint, cycle: distinct deployment cycle}; no groups inferred from app_model_state |
| Chassis tags | board_pose_before/after: {accepted: true, board_id: same stationary floor board, rvec: Rodrigues, tvec_m: board-to-camera, reprojection_rms_px: <=2}; board z is floor normal |
| Slack gaps | Missing/rejected endpoints break contiguous runs; no bridged command gaps or inferred extra commands |

| Excluded log line | Timestamp | Method | Reason |
| --- | --- | --- | --- |
| 9 | 2026-10-13T09:22:53.330000+00:00 | get_camera_frame | no explicitly accepted marker pose |
| 10 | 2026-10-13T09:23:42.594000+00:00 | get_camera_frame | no explicitly accepted marker pose |
| 12 | 2026-10-13T09:28:44.150000+00:00 | app_arm_out_compartment | no explicitly accepted marker pose |
| 13 | 2026-10-13T09:32:36.987000+00:00 | app_arm_direction_move | no explicitly accepted marker pose |
| 23 | 2026-10-13T09:42:38.201000+00:00 | app_arm_direction_move | no explicitly accepted marker pose |
| 24 | 2026-10-13T09:47:01.052803+00:00 | user_side_photo | no explicitly accepted marker pose |
| 36 | 2026-10-13T09:51:39.351000+00:00 | app_arm_direction_move | no explicitly accepted marker pose |
| 40 | 2026-10-13T10:28:31.977000+00:00 | app_arm_out_compartment | marker_not_detected |
| 41 | 2026-10-13T10:29:28.238000+00:00 | app_arm_direction_move | marker_not_detected |
| 42 | 2026-10-13T10:30:24.040000+00:00 | app_arm_direction_move | marker_not_detected |
| 43 | 2026-10-13T10:31:37.617000+00:00 | app_arm_direction_move | marker_not_detected |
| 44 | 2026-10-13T10:32:58.288000+00:00 | app_arm_direction_move | marker_not_detected |
| 45 | 2026-10-13T10:33:47.777000+00:00 | app_arm_direction_move | marker_not_detected |
| 46 | 2026-10-13T10:34:53.974000+00:00 | app_arm_direction_move | marker_not_detected |
| 47 | 2026-10-13T10:36:16.563000+00:00 | app_arm_direction_move | marker_not_detected |
| 48 | 2026-10-13T10:37:22.566000+00:00 | app_arm_direction_move | marker_not_detected |
| 49 | 2026-10-13T10:39:04.851000+00:00 | app_arm_direction_move | Nonplanar target; reprojection error above 2 px analysis threshold |
| 52 | 2026-10-13T10:57:06.152000+00:00 | app_arm_out_compartment | marker_not_detected |
| 53 | 2026-10-13T10:59:25.121000+00:00 | app_arm_direction_move | marker_not_detected |
| 54 | 2026-10-13T11:01:34.346000+00:00 | app_arm_direction_move | marker_not_detected |
| 55 | 2026-10-13T11:02:27.310000+00:00 | app_arm_direction_move | marker_not_detected |
| 56 | 2026-10-13T11:03:21.517000+00:00 | app_arm_direction_move | marker_not_detected |
| 57 | 2026-10-13T11:04:14.274000+00:00 | app_arm_direction_move | marker_not_detected |
| 58 | 2026-10-13T11:05:22.915000+00:00 | app_arm_direction_move | marker_not_detected |
| 59 | 2026-10-13T11:06:16.212000+00:00 | app_arm_direction_move | marker_not_detected |
| 60 | 2026-10-13T11:07:42.700000+00:00 | app_arm_direction_move | marker_not_detected |
| 62 | 2026-10-13T11:11:08.203433+00:00 | analysis_direction_limit | no explicitly accepted marker pose |
| 63 | 2026-10-13T11:11:37.772000+00:00 | app_arm_direction_move | marker_not_detected |
| 75 | 2026-10-13T11:24:17.023000+00:00 | app_arm_direction_move | marker_not_detected |
| 77 | 2026-10-13T11:28:07.127000+00:00 | app_arm_move | low rotation unresolved (not a measured zero step); verify downstream attachment, limit and pose noise |

| Parse exclusion line | Reason |
| --- | --- |
| none | No malformed/truncated lines in snapshot |
