# Prospective pitch-plus-roll trial — blocked preflight

## Outcome

**INCONCLUSIVE: no calibration target visible from the robot's current docked station.** Fresh front preflight and final images both show the kitchen, no floor ChArUco board and no visible deployed pincher. Full-frame DICT_4X4_1000 detection found zero markers in the preflight. No observed route to the prior board station was available; blind navigation was not performed.

The requested achieved target (retained `065-g250-45-40-nominal.jpg`) and comparator were **not acquired**. Nominal labels were not used as absolute commands or measured angles. There are **2 new front originals, 0 G originals, 0 setup/route jogs, 0 roll jogs, 0 successful/rejected jogs, 0 target brackets, 0 comparator brackets and 0 refits**. No prediction RMS/p95/max or fresh physical length is available. `prediction-analysis.json` represents unavailable metrics as null, not zero.

## Frozen models and non-leakage

Before further capture/cleanup, copied offline candidate/protocol/model-selection/G-lens/target-design/pairing artifacts under `frozen-models/`, with SHA-256 in `hashes.json`. Both front scenarios remain fixed (`fixed`, `fcyk1`), including their K/D and rigid T_G_M. The selected candidate has f824.966702, cy583.778554, k1−0.070084527. `evaluation-models.json` is a later extraction from that pre-capture immutable `models.json` snapshot, not a fit to fresh images.

- `candidate-proposal.json`: `25502221d17127348d698e525aa4e2a58ca589b789272a8155a19cb9f2eea5f5`
- `models.json` (both scenarios): `f896422423da0a6c5e8959c463cdf63427538d17ba16930b3a91cbd9ec3bc7a4`
- G `proposed-camera-info.json`: `bfe0fcb2cb5ebcd29a16dec5d25c953a3fe7067b31410bb40c05f148048bf43b`

The docked station is a new block: no old T_F_W was treated as current. No new floor-only registration was possible without floor pixels. No holdout marker pixels were used for T_F_W, T_G_M, intrinsics or model selection. Attachment rigidity/remount continuity could not be inspected because the arm was not deployed; the old T_G_M therefore remains an untested mount-continuity assumption, not a validated current transform.

Original pilot stationarity policy retained: 3mm translation and 12.779567° orientation, not 1° precision. No bracket exists on which to apply it. Exact exposure times remain unknown. No simultaneous-camera claim.

## What this says about the candidates

This attempt provides **no new optical discriminator**: it cannot say whether the selected model predicts this unseen pitch/roll case better or worse. Prior reported pitch failure and DO_NOT_ADOPT_PRODUCTION disposition remain unchanged. The prior conditional common-station L1 near180mm was not revalidated; physical L2, M4 zero, M5 axial station and exact pad geometry remain unknown. No anatomical dimensions are inferred from missing imagery.

## Cleanup and verified final state

Only MCP observations/actions were used, including the existing `live.mjs` MCP SDK helper. No direct robot HTTP/SDK/go2rtc fallback, service restart, Docker operation, grip, deployment, joint jog, cleaning, navigation or dock command.

Sent explicit stow, pickup-mode exit and program-mode exit; selected front and retained final front image. Read-back after consumer idle:

- Battery49%, raw state8 (still docked), error0, cleaning0, returning0.
- Live arm feedback `data:[0,0,0]`; final capture estimator IN_ROBOT with zero joints (prediction, not encoder truth). Image shows no visible deployed manipulator, consistent with stow but not alone proof.
- Camera status11685 (front), final1600×1200 front image, both z70 and z70_jpeg consumer lists empty.
- `get_grab_status` returned `unknown_method`; live arm feedback was subsequently read through authoritative `get_state`. Do not claim that RPC succeeded.

**Off-dock finish was not achieved:** the robot remained where preflight found it, docked. Moving it merely to satisfy off-dock cleanup would not recover the unseen board and would add unnecessary motion. No docking command was sent.

## Artifacts and preservation

All writes are contained in this new session directory: originals, MCP `log.jsonl`, frozen snapshots/hashes, `session.json`, `preflight-observations.json`, `prediction-analysis.json`, `final-state.json`, verification files and this report. Seven frozen source artifacts rechecked unchanged; all **92/92** prior originals verified against their existing SHA-256 manifest. No production/shared library/docs edits or commits.

Acquisition can resume once the board is again observable from a safe stationary station. This is a target-visibility dependency, not a battery, transport or calibration-success claim.
