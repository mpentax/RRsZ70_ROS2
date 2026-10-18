# Resumed prospective pitch-plus-roll validation

## Outcome — partial optical discriminator, no adoption

Acquired four new F/G/G/F brackets: two at an achieved pitch-plus-roll **neighbor** of the archived target, and a roll-zero comparator before/after. The exact archived `065-g250-45-40-nominal.jpg` achieved pose was **not reproduced**. Route jog refusals/partial movement invalidate absolute nominal labels; the final estimator label 250/95/40 is a route label, not measured joint angles. G-board observed target-neighbor departure from the archived target is about **32–34 mm and 7–8°**, depending on floor detection/registration. Do not report these captures as actual 250/45/40 or as a direct replication of the old failing pitch bracket.

**Disposition: DO_NOT_ADOPT_PRODUCTION / exact-target validation incomplete.** Candidate improvement on recovered roll brackets does not generalize to the comparator, and detector/registration sensitivity is substantial. No marker registration, front/G lens, arm geometry or physical dimension was fitted/adopted. Production patch remains null.

## Acquisition and scope

- 47 durable originals: **21 front, 26 gripper**; actual delivered JPEG sizes are 1600×1200 and 1600×896, respectively. Shared camera selected once per bracket leg, duplicate G exposures retained. No per-frame switch/restore, service restart or 3 Hz poller.
- **17 setup/route jogs + 8 roll jogs**, all 25 RPCs ACKed. ACKs are not motion completion; exact full/refused/partial jog totals remain null. The direct M3 route's 190→200 nominal step changed observed G-board orientation only about one degree rather than a full step. Subsequent recovery used raised-pitch route segments, preserving every frame and refusing to turn a dead-reckoned label into measured angles. G-board gaps at route17 and route20 were retained, not replayed blindly.
- **0 chassis pulses**, grips, picks, autonomous arm programs, cleaning or dock/navigation commands. Deployment's built-in obstruction check slightly changed board-relative chassis pose; registered the post-deployment stationary block separately.
- Four complete acquisition brackets: comparator-out [22,23,24,25], roll-neighbor1 [30,31,32,33], roll-neighbor2 [34,35,36,37], comparator-return [42,43,44,45]. Two repeats at roll, one comparator per approach/return direction—not 2–3 repetitions per direction. No broad sweep. Deployment09:49:32Z → measurement end10:00:21Z (10.82min), initial stow10:00:30Z, final cleanup stow10:04:18Z (14.76min after deployment).
- Board observed at fresh preflight, not inferred from old station: dense legacy 10×14 DICT_4X4_1000, checker20mm / black15mm; elevated ID68 black15mm kept separate from floor IDs. Full sheet framed but glare and shallow floor angle caused sparse, variable detections. Immediate workspace remained clear in inspected frames; background supervisor/cat did not enter the arm path.
- Live observations/actions/captures exclusively through RoboClaw MCP, including retained `live.mjs` MCP SDK helper and final MCP read helper. No direct HTTP/SDK/go2rtc fallback, Docker or shared-controller changes.

## Non-leakage and new W block

Copied both immutable model snapshots before measurement capture. Both K/D and each model's own rigid T_G_M are unchanged. The initial setup image preceded the copied snapshot, but was not used for model selection or marker fitting; all bracket images are new prospective exposures after the model freeze.

The predeployment registration in `new-block-floor-registration.json` is **setup-only** and not the evaluation registration. Deployment can rotate the chassis. Each model's evaluation T_F_W was therefore computed from **floor pixels only in postdeployment premeasurement front frames 002 and018**, never ID68 pixels. The old archived T_F_W appears only in the explicit archived-pose-departure diagnostic. Each held-out T_W_G is a converged positive-depth fit to the corresponding two G floor observations only. Then predict every ID68 corner using `T_F_W @ T_W_G @ T_G_M`. Tiny-marker PnP translations/orientations/branches are diagnostic only, never external ground truth.

Hashes, byte-identical to prior immutable snapshots:

- Both scenario evaluation snapshot: `f47a28c7e362aeeb1cd160b3aff147639e59580f40ed0e4ffad1fbeed135989a`
- Full models.json: `f896422423da0a6c5e8959c463cdf63427538d17ba16930b3a91cbd9ec3bc7a4`
- G proposed camera-info: `bfe0fcb2cb5ebcd29a16dec5d25c953a3fe7067b31410bb40c05f148048bf43b`

All **13 frozen artifacts/code hashes reverified unchanged**, including frozen analyzer, shared G-pose/project code and acquisition helper. Post-acquisition evaluation/recovery code hashes are retained separately in manifest.json; that code did not refit optical models.

## Primary detector results (preserved)

Original archived detector policy retained without replacing observations. Of four brackets:

- Comparator-out and comparator-return pass the original 3mm floor/G/marker translation, 3° floor/G orientation and12.779567° marker-orientation rules.
- Roll1 lacks a primary floor pose in F-after033; ID68 actually decoded in the raw full-frame detections, but marker metric association was unavailable without floor registration.
- Roll2's F-floor before/after fit differs about7mm, exceeding3mm. Its marker and duplicate G were stable; still rejected, not retroactively passed by low marker noise.

Identical accepted comparator exposures (16 corner errors = two brackets × two front frames × four corners):

| Frozen model | RMS px | p95 px | max px |
|---|---:|---:|---:|
| Old front `fixed` with its frozen T_G_M |12.551793|14.499597|14.635204|
| Selected `fcyk1` with its frozen T_G_M |14.635678|16.756896|16.878977|

Primary accepted roll metrics are **null**, not zero. Rejected roll2 raw diagnostic RMS is21.060269px old /18.424721px selected, not an accepted validation score. `prospective-evaluation.json` retains every raw all-four-corner vector, untrimmed metrics, depths and PnP branches.

## Offline detector-recovery sensitivity (not a replacement pass)

Because the board is visibly present, used a board-only crop/upscale/CLAHE/threshold sweep on retained images. No new captures, manual synthetic corners, marker fitting, lens selection or optical refits. Kept all candidate detections and primary outputs unchanged. Applied the same recovery method to training floor frames and all bracket front frames. This is **post-acquisition detector/registration sensitivity**, not a preregistered new detector or proof that the primary rejection was physical movement.

With recovered floor corners, comparator-out and both roll-neighbor repeats pass the unchanged stationarity limits. Comparator-return now fails the floor translation test (~4.36mm), demonstrating detector dependence. Roll2 passes narrowly (~2.95–2.97mm).

| Recovery-only accepted subset | Model | RMS px | p95 px | max px |
|---|---|---:|---:|---:|
| Roll neighbors, same two brackets /16 corners |old|15.641584|16.671024|16.925182|
| Roll neighbors, same two brackets /16 corners |selected|12.378336|13.575879|13.670694|
| Comparator-out only /8 corners |old|7.727210|8.923693|9.220863|
| Comparator-out only /8 corners |selected|9.039969|10.242418|10.538699|
| All three recovery-accepted brackets /24 corners |old|13.528095|16.576418|16.925182|
| All three recovery-accepted brackets /24 corners |selected|11.374935|13.519448|13.670694|

Do not mix primary and recovery rows to create a preferred aggregate. The candidate is worse on both versions' comparator and better on the recovery roll neighbors; neither restores the original failing pitch bracket. The old front also has nontrivial residuals: these data do not certify either chain.

Observed G-board camera rotation between comparator and rolled neighbor: **40.107631°**, with optical-center displacement5.987mm (diagnostic observed trajectory, not a1° joint accuracy certificate). Marker corners cover roughly **x518.58..662.12, y241.98..366.81**. Recovery prediction depths span roughly0.106..0.124m old,0.107..0.125m selected; all evaluated marker/G/floor target corners have positive depth. This does not validate upper/edge lens regions or independently surveyed metric scale.

Elevated marker's black square appears planar, fully visible and backed against the gripper, with jaws open; no visible flex in inspected target image. Exact physical continuity of sticker placement versus the old calibration cannot be proven from images. T_G_M mount-continuity is still an untested assumption, not newly fitted to suppress these errors. Print scale/flatness, sparse glare-affected floor detection, optical extrapolation, sequential exposure timing and body rocking remain systematics. Rough60mm exterior camera height is not an exact optical constraint. Physical L2, joint zeros, wrist/M5 axial stations and exact pads remain unidentified.

## Cleanup verified

Final MCP readback in `final-state.json` / `cleanup-verification.json`:

- Battery**63%**, state**3**, error_code**0**, cleaning0, returning0; common_status49154 → grab-mode bits10–11 **0**.
- Explicit final stow and settling, front image046 with no deployed arm, and final estimator IN_ROBOT corroborate stow. Raw arm `[0,0,0]` and zero estimator uncertainty are **not encoder truth**.
- Program runtime `program_state:0`, no active program. The guessed `app_set_program_mode` returned unknown_method (not success); corrected to evidenced `app_program_enter_exit {flag:0}`. A later cleanup transition through states39/41 and pickup-exit vendor−10007 was resolved by `app_stop`, readback state3/runtime0, then explicit final stow. All responses retained; no uncertain motion replay.
- Front camera status11685; both `z70` and `z70_jpeg` consumers**[]** after owned snapshot consumers expired. No MCP camera-stop tool exists in this server; idle/empty consumers are the verified release result, not a claimed service shutdown.
- Robot remains off dock at owner's station; no docking/base movement commanded by this trial (apart from deployment's vendor built-in check). No shared production/library/docs edits or commits.

## Files and reproduction

Everything created is under this session directory:47 raw originals; MCP log; frozen snapshots/hashes/protocol; primary and recovery observations; two evaluation result/registration sets; counts/observed-span/cleanup/manifest JSON; five local analysis/test helpers plus MCP final-read helper; this report.

From repo root:

```
calibration/arm-validation/.venv/bin/python calibration/arm-calibration/20261015T094801Z-prospective-pitch-roll-resume/test_validation.py -v
```

**5/5 tests pass**: frozen hash preservation, G-pose independence from F pixels, explicit rejection retention, metrics recomputation from all raw marker corner errors, final idle/front/empty-consumer state. Test output in tests.txt. All47 original SHA256s/dimensions in manifest.json. Source snapshots remain byte-identical to prior archive; no prior archive-writing command was invoked.
