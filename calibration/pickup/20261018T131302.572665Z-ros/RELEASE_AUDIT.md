# Gemini stacking/release audit

## Scope and sources

Offline investigation only: no robot/camera/service/Docker contact and no production-source edits. The existing working tree contains Gemini/other-session changes and was left intact.

- Hermes session: `20261018_151907_ab7744`, model `gemini-4-argon`, title “ROS2 planning for stacking three dodecahedrons”.
- Original report: database message 24801; preserved/replayed version 24919. Original command/tool results were inspected, rather than counting replayed transcript copies as new experiments.
- Hardware timeline: this directory's `timeline.jsonl`, 447 records.
- Manual release experiments are in the conversation, not this timeline. Absence from the timeline alone does NOT mean they were never attempted.
- Source: current `src/z70_manipulation/z70_manipulation/executor.py`, `planner.py`, and `src/z70_driver/z70_driver/estimator.py`.
- Protocol references: `research/android-program/editor-generated-fixtures.json`, `docs/archive/arm-program-research.md`, and original Android disassembly at `/Volumes/GOLD16-HFSP/WD16GOLD01/Michael/s/Z70Android/targeted-plugin/index.android.hasm` (H line references below).

## Verdict

Gemini tried several real release sequences. It did not establish that every API was exhausted or that firmware universally locks the gripper after `app_grip`. The strongest established problems are in our executor: placement is not executed, release is not verified, and the executor advances toward another object with the first object still held. Successful pickup/hold is supported; accurate carrier alignment, successful tier placement and a completed stack are not.

## Reconstructed actions

All times below are UTC from retained records.

| Time / source | Actual action and evidence |
|---|---|
| Before the main run | Multiple deployment attempts; user heard “move the robot to a less cluttered area”. The user eventually deployed through the official app. An ACK alone had not established deployment. |
| 13:14:47, timeline 232 | `app_grip {}` on Purple. |
| 13:14:49 and 13:14:52, 234/237 | Two explicit `app_arm_direction_move {direction:1}` lifts. |
| 13:15:00–13:15:13, 241–247 | Front-image checks reported Purple held and absent from its original floor spot, including the 10-second hold. These are model-dependent visual estimates, not encoders. |
| 13:15:29, 287/292 | Yellow aligned to the default floor-pick point. A tier-2 placement `[220,75]` was logged but never commanded. |
| 13:15:31 and 13:15:33, 294–297 | Two `app_open_gripper {}` attempts; both returned unknown dispatch / transport timeout. |
| 13:15:37 onward, 298–346 | Snapshot named “placed”, another upward step, reverse chassis pulses, then joint traversal using the pre-lift starting angles. No release observation authorized these dependent stages. |
| 13:16:04–13:17:11, 347–447 | Proceeded to Blue, tried opening/descent/grip and three lifts. Blue hold verification failed. Aborted with `tiers_completed:2`, despite no established Purple placement. |
| Conversation 24760–24761 | Submitted M6-open program `arm_joints`, `value:[0,0,0,1]`, `step:1`, `program_id:null`. Received `["ok"]`, physical completion unknown. |
| Conversation 24766–24775 | Entered programming mode, submitted M6-open again, waited four seconds, exited programming mode, re-entered pickup mode, sent three downward Cartesian commands. Later image still showed Purple aloft; jaws were cropped. This multi-command sequence does not isolate the program's admission/execution result. |
| Conversation 24776–24777 | Requested pickup flag 2, then open with 20-second timeout. Open timed out. |
| Conversation 24784–24791 | Requested pickup flag 1, then open with 20-second timeout. Open timed out. Later image still showed Purple aloft, with jaws cropped. Requested mode is not proof the transition completed. |
| Conversation 24792–24798 | `app_put_down {flag:0}`, then `app_stop_grasp []` approximately 0.9 seconds later (script elapsed 0.96 seconds). Later image showed Purple apparently floor-supported in a substantially changed scene. It did not demonstrate elevated in-place release or exact motion duration. |

### What the photographs actually support

A fresh offline review of the retained images found Purple apparently held in both `tier-2-pre-release-front.jpg` and `tier-2-placed-front.jpg`, left-offset from Yellow's image centre. Their silhouettes meet/overlap, but neither physical support nor depth alignment is established. Thus the report's “directly above Yellow, verified” claim is too strong. The small bearing/range errors were calculated relative to the wrong alignment target, not a measured held-object-to-support transform.

`after-m6-program.jpg` does not show the gripper/object at all. `after-program-down.jpg` and `after-flag1-open.jpg` show Purple aloft but crop the jaws: they support no demonstrated release, not a proved closed-jaw firmware latch. Squishy-object adhesion/wedging is not excluded. `after-short-putdown.jpg` supports a floor-supported outcome, not the entire release trajectory.

## Confirmed script defects

1. **Placement plan is decorative.** `executor.py:724–732` calls `align_front(base_color)`, reads/logs `placement_state`, but never calls `move_arm` with that pose. `planner.py:203–240` computes placement/clearance that the live path does not execute. Default alignment uses the floor-pick point.
2. **Unknown release becomes success.** Gripper timeout is returned for observation (`executor.py:58–59`), but stack sends two opens, sets `gripped=False` unconditionally (`730–734`), increments tiers (`742`) and ultimately defines `stack_verified` by its counter (`751–752`). A filename is not a verification.
3. **Stale starting pose after lifts.** Grip invalidates the planning pose (`estimator.py:161`), and hand updates cannot recover UNKNOWN (`109–110`, `135–136`). Stack later supplies pre-grip descent angles as `start_joints` (`executor.py:748`), bypassing the estimate read (`341–352`). Do not fix this by pretending the estimator remained valid: retain observation-backed pose provenance or use a verified recovery pose.
4. **Per-object geometry is reused.** `pregrip_jaw` is assigned only if None (`717–718`), so the second object's fallback held test may use the first grasp's geometry after intervening chassis motion.
5. **Evidence files overwrite earlier tiers.** Fixed `1-pregrip`, `2-lifted`, `3-front-verify` and `4-held-10s` names are reused. The timeline contains Purple and Blue attempts, but the final common filenames need not show Purple. Use tier/attempt subdirectories.
6. **Tests do not prove loaded release.** Existing stack coverage mainly checks dry-run/planning and interface wiring. The fake updates arm state on a jog even after grip invalidation (`test_executor.py:105–110,119–121`), unlike production. Add behavior tests for no-op opens, retained object, unsupported placement and UNKNOWN pose.

### Executed offline reproduction

`audit_stack_release.py` runs the real `Executor.stack` with explicitly synthetic stage inputs and no ROS/network/robot access. Successful pickups are injected; all opens time out and no release observations are supplied. Actual output:

```json
{"tiers_completed":3,"stack_verified":true}
```

Logged placement states were `[220,75]` and `[210,85]`; actual move targets were only hover `[190,65,0]` and descent `[250,105,0]`. This proves the executor's false-success and missing-placement control-flow bugs, not any physical firmware behavior.

Run from repository root:

```sh
PYTHONPATH=src/z70_manipulation:src/z70_description ~/.pixi/bin/pixi run python calibration/pickup/20261018T131302.572665Z-ros/audit_stack_release.py
```

## API conclusions: known versus untested

- `{}` is correct for `app_open_gripper`: original Android wrapper #5522, H230906–230917. Changing to arbitrary parameters is not an evidenced fix.
- M6 program open `[0,0,0,1]` and close `[0,0,0,2]`, both `step:1`, are real editor-generated records (`editor-generated-fixtures.json:140–200`). Serialization/UI intent does not prove physical execution.
- No `app_get_program_runtime` tool-call occurrence was found in the inspected transcript export. Gemini checked submission ACKs but did not inspect runtime `program_state` / per-command results. The Android program UI does poll runtime; see `arm-program-research.md:108–144`.
- No actual program-M6-close trial was found. Gemini offered it at the end of its report; offering an experiment is not performing it. A fresh program-close → program-open experiment therefore remains distinct from trying program-open after `app_grip`.
- A mode request ACK is not proof of mode readiness. Record fresh `common_status` and decode `(common_status >> 10) & 3` alongside the raw value. The APK names telemetry mode 1 manual and mode 2 auto; do not assume every command flag directly predicts observed mode during a transition.
- The Android bundle also exports a five-input `remoteManualArmPickUp` wrapper that emits `app_arm_move {m:[arg1,arg2,arg3,arg4,arg5]}` (H224875–224876, H231765–231788). No named consumer established fourth/fifth slot meaning. The similarly named `_remoteManualArmPickUp` manager actually accepts THREE inputs and calls the separate `appArmMove` serializer (H858530–858557, H230521–230539); UI callers use these for M3/M4/M5 (e.g. H1284273–1284297 for M5). The five-slot schema is therefore not an evidenced hidden gripper API. Do not assign M6/open/speed meanings to its extra slots or probe them blindly.
- `app_put_down` is demonstrated as a recovery route with unwanted motion, not a stationary jaw-open primitive. Flag 1 appears in the official continuation branch (H1227820–1227830), while flag 0 starts put-down (H1227832–1227842); this does not justify trying flag 1 as an invented elevated-open command.

## Next trial: release qualification before stacking

This is a proposed supervised protocol, NOT motion authorization or a ready-to-run unverified robot script.

1. Begin with a fresh observed deployed setup, one object, room clear and a low catch area, not a three-object tower. Keep one execution owner; route hardware access through RoboClaw MCP/ROS, not ad hoc credential-reading HTTP snippets.
2. Capture empty-jaw geometry and raw mode/status. Test one known open command once, then observe actual jaw gap. Do not wait twenty seconds merely for a reply to a command known often not to reply.
3. Qualify programming execution with a source-evidenced small command and runtime query, independently of release. Preserve fresh runtime/per-command results plus before/after images; ACK/runtime labels still do not replace physical observation.
4. On a fresh setup, test the paired editor-generated program M6-close then M6-open, with the object supported or at a low observed clearance. Observe after each command and compare empty versus loaded jaw behavior. Do not mix program tests with three downward moves before the first observation. This remains an unvalidated candidate, not an asserted solution.
5. Compare against a separate fresh `app_grip` → `app_open_gripper` sequence with identical observations. Confirm mode transitions through telemetry. Change one variable per experiment. If jaws visibly open but foam remains attached, investigate deformation/wedging/adhesion instead of calling it firmware refusal.
6. Capture durable evidence per attempt: command/parameters, outcome, requested and observed mode, runtime state/results, pre/post frame identities, jaw aperture, object retained/released, wrist/chassis displacement, final support. Use tier/attempt folders and keep failed attempts.
7. Only after a non-spinning loaded release is physically demonstrated, resume a two-object placement. Compute alignment from the current measured held-object column/TCP, execute the actual placement motion from a valid pose, establish support, open, observe detachment and stability, then withdraw. Advance `tiers_completed` only after this evidence. Unknown release means placement incomplete; resolve it by observation/recovery rather than advancing blindly or treating every data gap as an emergency.
8. Attempt the third object only after stable two-object support. Fresh per-object `pregrip_jaw`/tracking and pose provenance are required.

## Prioritized implementation changes

- Replace the unconditional open/success block with a release result that retains `held/unknown/released` and evidence; no tier increment or next pickup while release is unestablished.
- Execute—not merely log—the intended placement, or redesign placement around measured held-object geometry. Do not simply add `move_arm(place_state, current)` because `current` is stale after lifts.
- Reset per-object verification state and namespace snapshots by tier/attempt.
- Add regression scenarios for no-op opening, empty-looking cropped frames, object still attached after jaw opening, loss of pose after lift, and stable support versus merely nearby coloured blobs. Full stack success must depend on observed support/stability, not a loop counter.
- Keep program/runtime experimentation separate from the production stack until qualified. No newly guessed RPC, recovery/reset command or put-down interruption should become an automatic elevated-release fallback.

Production source was not patched or deployed in this investigation. Elevated in-place release and stacking remain unvalidated.
