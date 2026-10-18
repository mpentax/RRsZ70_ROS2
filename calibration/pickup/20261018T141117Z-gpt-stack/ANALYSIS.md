# Latest Gemini release review and GPT trial

## Evidence review

Source session `20261018_151907_ab7744`, messages 25449–25534 and final report 25611; retained photographs in `calibration/arm-validation/gripper-api-20261018/`.

The owner's eyewitness account establishes that Purple and then Yellow dropped off the intended support and rolled away. Both command traces show grip, two upward Cartesian hand steps, chassis corrections, then manual-mode `app_put_down {flag:0}`. Neither shows a pre-release descent. Yellow's downward commands were BEFORE pickup and are not a corrective placement descent.

The release primitive is a useful real discovery: Manual Arm Control mode (`app_pick_up_enter_exit {flag:1}`, observed grabMode1) + `app_put_down {flag:0}` opens the hand in place. In auto mode2, put_down can lower/rotate the chassis. Normal release must not be confused with recovery-wizard `app_open_gripper {}`.

Both releases lacked measured support/contact and accurate held-object-to-support depth alignment. Images show residual horizontal offsets/clearance, but the claimed 15–25mm gaps/depth offsets are not independently measured. These are not force-controlled poses, and marker-derived grasp aim has >=15mm provisional uncertainty. A nominal40mm down command cannot be prescribed for a claimed15–20mm gap; it may overshoot. Search legal discrete joint configurations and observe a suitable approach instead of blindly lowering once.

### Evidence-timing mistake

Gemini's frame27 was requested concurrently with the yellow release; receipt was7ms after RPC request and5ms before RPC completion. Exposure time was unknown. It is NOT reliable post-release evidence and cannot establish the detailed mid-air trajectory. Frame28 arrived208.07 seconds later with a cat behind the targets. User's eyewitness account is ground truth for rolling away; neither still supports precise bounce velocity or flight timing.

## GPT scope

User authorized code improvement and supervised live stacking, and will return all objects to frame. Use blue base, purple middle, yellow top (latest trial order). Keep person/pet clearance, bounded commands, shared MCP/ROS authority and stop capability. No new motion while the owner is placing targets; recheck the scene before live execution.

Initial retained front frame425 shows all three objects, no visible person or cat, no visible arm. State reports robot idle,error0,battery74%, common49154, planning pose UNKNOWN/restored. This does not prove future clearance or authorize fabricated pose anchors. The new stack dry-run `0858451b-1e50-4cc0-9295-9dad630db17e` succeeded with dry_run_valid true; no physical stack is implied.

## Required behavior changes

- Measure held lower contour/support upper contour in the same exposure; preserve a valid depth basis instead of projecting held pixels onto floor.
- Align to measured held column, execute a real reachable placement approach, reobserve near contact, settle, and release only from a credible supported configuration.
- Treat missing contact/release/pose evidence as an incomplete placement to resolve—not success or a global emergency.
- Require genuinely later frames for detachment and stable stack confirmation before counting tiers.
- Reset per-object geometry and namespace tier/attempt images.
- Never plan post-lift traversal from pre-lift angles or call an ACK a physical completion.

## Verification and real trial outcome

- TypeScript typecheck/tests: 290 passed.
- Isolated driver/gateway tests: 80 passed.
- Current manipulation suite (after camera-selection, pose-capture, and pale-facet fixes): **65 passed, 3 xfailed in 30.76s**. Focused perception/placement: 13 passed. `git diff --check` clean. Tests are offline logic verification, not proof of a tower.
- Built ROS workspace and exercised the real stack action. Automatic pickup did not reliably hold Purple; corrected the pale-facet segmentation and retained the failed run at `../20261018T143248.735375Z-ros/`.
- Parent manually regrasped Purple using observed bounded commands. `14-retry-lift.jpg` through `30-face-seating.jpg` document carrying, approaching Blue, actual M4 lowering steps and wrist-roll seating adjustment. Unlike Gemini's releases, lowering was executed before opening.
- Manual-mode release returned `["ok"]`; `31-released-on-blue.jpg` appeared to show Purple on Blue, but the hand was not yet clear. **This was not a verified completed tier.** After withdrawing the open hand, `32-two-tier-clear.jpg` showed Purple off the intended support. Apparent image contact had not established centered load-bearing support; front-to-back seating and face orientation remained unresolved.
- Regrasped Purple (`43-second-pickup.jpg`) and lifted again (`44-second-carry.jpg`, `45-carry-current.jpg`). The last explicit arm command was at 15:07:14.860Z; first subsequent chassis pulse was at 15:12:27.352Z, a **312.492-second gap**. Frame45 at 15:10:32 still showed the carried object and state mode1.
- Frame46-left-route-1 showed Purple down in front of/obscuring Blue with the hand absent. Subsequent state was mode0/common_status49154/error0. No deliberate stow command appears in this trial's MCP log. The completed offline source/log audit found no explicit stow or mode-exit mapping in the pulse path. Inactivity stow is consistent with the interval but remains unproven: there was no observation at the hypothetical 300-second boundary. Standalone BasePulse starts/moves/stops/ends RC on every call (`commands.py:35–36,334–343`), without the five-second entry settling used by manipulation `Executor.start_rc()` (`executor.py:222–244`). RC mode transitions are another plausible trigger, but earlier pulses preserved the held object, so there is no demonstrated unconditional RC→stow rule. Neither cause is established and neither is claimed fixed. Parent spent too long reasoning while a payload was held. Future trials should prepare the route before grasping, reacquire mode/hold after a long pause, and qualify a single RC lifecycle for carrying before adopting it; do not invent arm-jog keepalives.
- Issued priority `stop {scope:3}` at 15:13:28Z: all three stop/end outcomes returned NONE, but the tool itself reported cessation_observed:false. Later readback reported robot_state3/IDLE, mode0, grab0, error0. Frame48 showed no arm and Purple/Yellow floor-supported; Blue was occluded and cannot be independently located from that image. No subsequent motion was sent.

**Result: no verified two-tier or three-tier stack. Do not run an unattended stacking demo or claim the script physically qualified.** The current source includes supported-placement/release verification logic, but precision seating and the unexpected stow need resolution before further live carrying. The running pick server may predate the final perception source edits; passing current-source tests does not establish the loaded process version.

### Lessons from the parent trial

- A front-camera silhouette overlap can be side/near-face contact, not a stable support polygon. Check front-to-back alignment and object face orientation, and prove stability only after the hand is clear.
- Cartesian lift-column depth cannot be carried unchanged through subsequent M4 joint steps. Those steps change both height and reach. A nominal/width-derived correction is not a fresh measured object pose.
- A bottommost contour corner is a support/contact feature, not automatically the object's centre of mass or grasp centre. Pale facets and deformation must remain visible in the segmentation.
- Stage planning and code work before grasping; keep the carrying/placement loop short and continuously observed. Do not leave a held payload across multi-minute reasoning gaps or invent unvalidated keepalive RPCs.
- Preserve post-release, post-withdrawal and settled evidence separately. Frame31's apparent two-object stack did not survive frame32 and must never be counted as success.
