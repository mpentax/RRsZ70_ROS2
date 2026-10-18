# RoboClaw control-stack audit: what to replace, simplify or keep

Date: 2026-10-13. Reviewer: Claude (Hermes subagent). This review was **read-only**. I made no robot, go2rtc, service or cloud calls, and I did not read `.env` or `.local/`.
I inspected checkout `efc8cf5` plus the untracked working tree. I ran `npm test` once: **311 tests, 310 pass, 1 fail** (see §4).

---

## TL;DR

1. **The TypeScript gateway is already mostly thin.** Commit `e4bb653` deleted about 7.6k lines of leases, holds, pulse workflows and the cockpit. Still in it: about 2k lines of dead modules (with about 1k lines of tests on them), stale docs and a README that still describes the deleted authority layer.
2. **The real ceremony now lives in `integrations/dimos/`.** About 5k lines of Python, a QUIC relay, file queues, a second LLM (Copilot via the OpenCode CLI), receipts and epochs. The Hermes plugin built on it **auto-denies permission and runs in inference-only mode**, so **Hermes currently cannot actuate the robot at all.**
3. **The fastest working loop needs almost no new code.** Register the existing `src/mcp-main.ts` as a Hermes `mcp_servers` entry (opencode already does this). Set `ROBOCLAW_MCP_IMAGE_HANDOFF=private-file`. Then Hermes can loop `get_camera_frame` → `vision_analyze(imagePath)` → `robot_rpc`. The commands below have already been confirmed on hardware.

---

## 1. Architecture map (as built)

### Command path that actually exists today

```
opencode (opencode.json → mcp.roboclaw)            ← the path the 2026-10-06 live trial used
  └─ stdio MCP  src/mcp-main.ts → src/mcp-server.ts  (4 tools)
       └─ HTTP + bearer to 127.0.0.1:57010  (Docker 'app', src/service.ts)
            ├─ /api/v1/rpc  dispatch()  service.ts:176-203
            │    ├─ transport "local" (default): RobotClient  src/robot.ts  → TCP 58867, L01/1.0 framing+AES (own TS impl)
            │    └─ transport "cloud": CloudControlClient  src/cloud-control.ts → MQTT rr/m/i/... (own TS impl)
            ├─ /api/state  → 3 parallel RPCs: get_status, app_get_arm_joints_data, get_camera_status  service.ts:204-212
            └─ /api/camera/frame → go2rtc /api/frame.jpeg?src=z70   service.ts:346-357, src/go2rtc.ts:124-157
```

- **Protocol implementation:** the TypeScript code is the project's own, ported from `copystring/ioBroker.roborock` and cross-checked against python-roborock (`src/local.ts:1-6`, `src/robot.ts:1-6`). It does **not** use python-roborock.
  - L01 means AES-256-GCM with nonces (`local.ts:76-111`).
  - V1 means AES-128-ECB (`local.ts:85-104`).
  - Cloud MQTT auth is copied from go2rtc's driver (`cloud-control.ts:28-45`).
  - The live device runs **L01 local**. That was verified on 10-06; see `docs/research/z70-programming-live-trial-2026-10-06.md`.
- **Video:** go2rtc 1.9.14 with an in-memory `PATCH /api/streams` source `roborock://<broker>?u&s&k&did&key&pin` (`go2rtc.ts:29-48,201-216`).
  - go2rtc does the WebRTC signalling with the robot over Roborock **cloud MQTT**. Snapshots go through `/api/frame.jpeg`.
  - The config is `deploy/go2rtc.yaml`, which has no preload.
  - Each snapshot cold-starts a remote-viewing session. The docs warn this can disturb arm mode.
  - The front camera returns 1600×1200 (`docs/hardware-trial-2026-10-03.md:12`).
- **MCP tool surface:** `src/mcp-server.ts:157-205`, exactly 4 tools.
  - `get_state`
  - `refresh_status`
  - `get_camera_frame`: inline image, or `imagePath` when `ROBOCLAW_MCP_IMAGE_HANDOFF=private-file` (`:182-186`)
  - `robot_rpc {method, params, transport?, timeoutMs?, deviceId?}`: any vendor method, no allowlist (`mcp-actions.ts`)
- **Named HTTP aliases** (`vendor-rpc.ts:4-52`) are one-RPC mappings used by the browser buttons and old clients.

### DimOS/Hermes path (wired into Hermes today)

```
Hermes plugin roboclaw-acp (integrations/dimos/hermes_plugin/__init__.py)
  └─ spawns ACP agent process  roboclaw_dimos.acp_interface
       └─ `docker run` a relay client container (acp_interface.py DockerRelay)
            └─ WebTransport/QUIC relay (TLS, token files) → DimOS controller (controller.py)
                 ├─ HostedCopilotModel → file queue → OpenCode CLI → GitHub Copilot  (model.py, hosted_inference.py)
                 └─ Z70Adapter → file queue → host_bridge.py → stdio MCP (the same src/mcp-main.ts) → HTTP → TCP
```

- `~/.hermes/config.yaml` enables the `roboclaw-acp` plugin and the `roboclaw` toolset. It has **no `mcp_servers` entry** for RoboClaw.
- The plugin's `request_permission` always returns `cancelled` (`hermes_plugin/__init__.py:18-20`).
- New ACP sessions default to `inference-only` (`acp_interface.py` `new_session`), and the plugin never switches modes.
- **Net effect: Hermes → robot is impossible by design right now.** Hermes can only ask a *different* LLM to reason in text.
- Even in `tools` mode, `Z70Adapter.call` strips images before the model sees them (`adapter.py:9-11,26-27`). The controller's system prompt also says "media references are not vision evidence" (`controller.py:10-12`). **So the DimOS loop is blind.**

Six process and transport hops, plus a second LLM, sit in front of the same `robot_rpc` that opencode calls directly. This is the single largest piece of overengineering in the repo.

---

## 2. Roborock protocol layer: what is proven vs speculative

### Proven on hardware (owner-observed or vendor-ACKed with an observed effect)

| Command | Evidence | Notes |
|---|---|---|
| `get_status []` (local L01) | Every trial | Returns state, battery, error. State 8 = charging, 39 = RC/manual, 42 = programming. |
| `app_rc_start` → **wait ~5 s** → `app_rc_move [{velocity:0.1,omega:0,duration:800,seqnum}]` | `README.md:487-492` | Forward motion observed. **Without the 5 s wait, no motion.** |
| `app_rc_move` with `velocity:0, omega:0.5, duration:800` | `README.md:494-498` | About 15° in-place turn observed. |
| `app_rc_stop`, `app_rc_end` | Same trials, `native_slam/PRINTER_FREE.md:35` | ACKed. Cessation was eyeballed only. |
| `app_program_enter_exit {flag:1}` | 10-06 trial | ACK, state 8→42. **It drove the robot off the dock.** |
| `app_start_program {cmd_ids:[...]}` with `delay`, `sound`, `arm_out`, `arm_joints` M4 `[0,1,0,0]` / `[0,2,0,0]` `step:1`, `arm_in` | 10-06 trial table | Each motion confirmed by Michael. The up→delay→down chain also worked. |
| `app_get_program_runtime {}`, `app_get_program_soundlist {}`, `app_get_programs_summary {}`, `app_get_program {program_id}` | 10-06 trial | Reads work. Runtime `program_state` goes 2 = executing → 0 = done, with a per-record `result`. |
| go2rtc snapshot (front camera) | 10-03 trial | A 1600×1200 JPEG was received and viewed. One later capture failed. |

### Works but useless

- `app_get_arm_joints_data {}` **always returns `[0,0,0]`**, including with the arm deployed and held at an offset (10-06 finding 5).
- `/api/state` still calls it on every read (`service.ts:210`). Drop it.

### Failed or never validated

- **The 10-03 "undock" via a service-owned pulse workflow** returned HTTP 502 with unknown outcome, then possibly moved.
  - The trial then **stalled because the old command-authority layer kept an "unresolved" record**, and only a browser operator assertion could clear it (`docs/hardware-trial-2026-10-03.md:31-39`).
  - Root cause was never diagnosed; the diagnostics didn't capture the failing stage. Likely suspect: issuing `app_rc_*` while docked or charging without the 5 s settle.
  - **In short, the gating, not the robot, ended that trial.**
- **Not tried on hardware at all:**
  - `app_charge` (dock)
  - gripper `app_grip` / `app_open_gripper`
  - `app_arm_out_compartment` / `app_arm_in_compartment` outside programming mode
  - `app_arm_direction_move`, `app_arm_move` (manual M3–M5)
  - `app_pick_up*` / `app_put_down*`
  - `app_change_camera` (arm camera)
  - `app_stop`, `app_stop_grasp`
  - cloud transport for control
  - arm-camera frames
- **Static Android disassembly only (speculative):**
  - everything in `docs/arm-*-research.md`
  - the M3/M5/M6 vectors
  - `arm_move` end-effector records
  - the manual joint controls in `src/arm-joint-protocol-research.ts`
- **10-06 session left open:** the trial paused with the robot in programming mode (state 42, off-dock). Flag 0 was never sent (`z70-programming-live-trial-2026-10-06.md` "Paused state"). Check the robot's physical state before the next run.

### Real bugs in the transport path

1. **Default timeout is 5 s, and a timeout destroys the whole TCP session.**
   - `robot.ts:202` defaults to `timeoutMs = 5000`, and `robot.ts:348-351` fails the session on any response timeout. That rejects every other in-flight request.
   - Named aliases call `dispatch(mapping)` with no `timeoutMs` (`service.ts:366-367`).
   - The 30 s arm timeout in `vendor-commands.ts:14-20` is only reachable through `RobotClient.command()` / `CloudControlClient.command()`, which **nothing calls**.
   - Result: gripper or arm aliases time out at 5 s and kill the session, even though the code comment says they take more than 15 s.
   - Fix: pass a per-method default timeout, or have the agent always send `timeoutMs`. Delete `vendor-commands.ts`.
2. **`get_camera_frame` is tagged `destructiveHint: true`** (`mcp-server.ts:30,175`). Approval-gating MCP clients will prompt on every snapshot. Reclassify it as read-only and note its side effect in the tool description.
3. **`publicRpcValue` drops any key matching `nonce|token|secret|...`** (`rpc-outcome.ts:20-27`). That is fine for credentials, but it silently removes legitimate vendor fields named `nonce`. Low priority.
4. **`/api/state` blocks on three RPCs**, one of them useless (above).

---

## 3. Inventory of gates, arbiters, leases, receipts and consent layers

### Still live in code

| # | Mechanism | Location | Verdict |
|---|---|---|---|
| 1 | Bearer token + cookie session + Host/Origin check | `src/service-auth.ts`, `service.ts:227,241` | **Load-bearing** (the service binds a LAN-capable port). Keep. |
| 2 | Service token file mode/owner/size checks (exactly 43 bytes, 0600, uid) | `mcp-server.ts:46-58` | Ceremony. A plain file read is enough. |
| 3 | MCP origin must be a numeric loopback on port 57000–57100 | `mcp-server.ts:33-44` | Ceremony. One env var is enough. |
| 4 | MCP route allowlist (`READ_PATHS` + rpc) | `mcp-server.ts:31,69-73` | Harmless. Keep or drop. |
| 5 | `ACTUATION`/destructive annotation on the camera snapshot | `mcp-server.ts:175` | **Harmful ceremony**: approval friction on the observation tool. |
| 6 | `physicalCompletion: "unverified"` stamped on every response, plus `lensVerification`, `capturedAt:null`, `feedLensVerified:false` | `rpc-outcome.ts:17`, `mcp-server.ts:109-111,183-190`, `service.ts:184,253,258,354-356` | Ceremony. These are constant fields that carry no information. |
| 7 | Method regex, param size limits, queue bounds | `robot.ts:203-206,237-248`; `service.ts:177-181` | Structural, cheap. Keep. |
| 8 | Credential redaction of vendor results | `rpc-outcome.ts:20-27` | Mildly load-bearing (the local key can echo back). Keep a simpler version. |
| 9 | Paranoid secret store: symlink/ancestor/hardlink refusal, exact schemas, race-safe token creation, migration validation | `src/secret-store.ts` (373 LOC, 22 tests) | Ceremony for a single-user Mac mini. About 60 lines of "read/write JSON 0600" would do. |
| 10 | Retired-route 410 responses | `service.ts:250-251`, `vendor-rpc.ts:49-50` | Ceremony. Delete the routes and let them 404. |
| 11 | Login rate limit (60 s) and 5-attempt cap | `service.ts:300,309` | Fine (it protects the Roborock account). |
| 12 | DimOS `tool_mode` inference-only default + ACP per-prompt `request_permission` | `receipts.py:55-57`, `acp_interface.py` `prompt` | **Blocking ceremony.** Combined with #13, it makes actuation unreachable. |
| 13 | Hermes plugin auto-denies permission | `hermes_plugin/__init__.py:18-20` | **Blocking.** |
| 14 | Durable receipts: flock'd single controller, `controller_busy` rejection, content-hash duplicate detection, epochs, `interrupted_unknown` on restart | `receipts.py` (120 LOC) | Ceremony. |
| 15 | File-queue transports with expiry (≤76 s), ownership/symlink checks, `.working` recovery | `queue_transport.py`, `host_bridge.py`, `model.py` | Ceremony. Exists only because of the container split. |
| 16 | Image stripping before the model sees results | `adapter.py:9-11` | **Harmful.** It blinds the agent. |
| 17 | TLS-loopback relay origin + private token files | `headless.py:13-25` | Ceremony for a localhost relay. |
| 18 | Legacy gate config keys (`actuationEnabled`, `supervisedMode`, ...) accepted and ignored | `config.py:22-30` | Dead. Delete. |

### Deleted from code but still in the docs

These either lived only in the uncommitted working tree or were removed in `e4bb653`. **They are not in `src/` now**, but 25 docs still describe them as policy:

- `command-authority` / `command-arbiter` and durable uncertainty journals (`docs/command-arbiter.md`, `docs/shared-command-authority.md`)
- motion holds (`control-hold.ts`)
- chassis leases (`chassis-session.ts`)
- arm busy gate (`arm-controller.ts`)
- the pulse/undock workflow (`control.ts`)
- the pin-go gate
- the navigation controller
- supervised docking with recovery challenge tokens and an operator "physically docked, stationary and stowed" checkbox (`docs/supervised-docking.md`, `docs/robotics-review-final.md`)
- the measured-completion observer that required three independent sensor channels plus a "controller terminal certificate" (`docs/measured-completion.md`)
- supervised risk acceptance (`docs/supervised-risk-acceptance.md`)
- the ROS leased action bridge
- the NVIDIA licence acknowledgement gate

**`README.md:186-216` still tells agents to use `reconcile_commands` and a "durable service command authority".** Anything that reads the README gets steered back into the old model.

**Verdict:** apart from #1, #7, #8 and #11, every mechanism listed is ceremony. None of it improves physical safety. The real safety controls are the physical stop button, a person watching, and short bounded `duration` values on `app_rc_move` (the robot enforces `duration` itself). The software gates mostly converted "unknown" into "stuck".

---

## 4. Test suite assessment

**Totals:**
- Node: 311 (`npm test`: 310 pass, 1 fail).
- `research/*.test.ts`: 50 more, outside `npm test`.
- Python: about 117 `def test_` across `integrations/dimos/tests` and `native_slam/tests`.

**Failing test:** `src/local-selection.test.ts:9`.
- `SecretStore.saveDevice` (`secret-store.ts:140-145`) doesn't call `#prepare()` before `#write`, but `loadDevice` does.
- On a fresh directory the save fails. I suspect `#prepare` is what creates or validates the state directory, but I didn't trace it.
- In production the service calls `initialize()` first, so it's latent. Still, the suite is red right now.

| Category | Files (tests) | Assessment |
|---|---|---|
| **Real logic: protocol/crypto/framing** | `local.test.ts` (15, incl. upstream crypto vectors), `robot.test.ts` (23, framing, fragmentation, keepalive, out-of-order replies), `cloud-control.test.ts` (6) | **Keep.** These protect the one component proven on hardware. Trim the "never retries" and "no replay" variants; about 6 of them say the same thing. |
| Real logic, low value | `cloud.test.ts` (14, login flow), `go2rtc.test.ts` (7), `service.test.ts` (7), `photo-protocol`, `v1-map-summary`, `private-map-raster` | Keep a few. They're fine. |
| **Pinning hardening/paranoia** | `secret-store.test.ts` (22: symlinks, hardlinks, races, migration CLI output), `mcp-server.test.ts` (origin/port rules), `rpc-outcome.test.ts`, `service-config`, `local-origin`, `credentials` | Mostly pins defensive behaviour and exact strings. Delete about 80% of it. |
| **Pinning philosophy, not behaviour** | Test names like "unknown timeout never latches later calls", "vendor rejection is … not a lockout", "obsolete motion holds … are not interpreted", "movement mapping … removes experimental policy limits", Python `test_legacy_disabled_flags_do_not_gate_transport` | These assert the *absence* of the old gates. They are harmless, but they exist to argue with the previous model. Fold them into one test or delete them. |
| **Testing dead code** | `camera.test.ts` (31), `vision.test.ts` (10), `photos.test.ts` (8), `map-list-probe` (5), `active-map-evidence` (4), `arm-feedback` (3), `preview` (3), `diagnostics` (1) | **65 tests (about 1,060 LOC) on modules no entrypoint imports.** Delete them with the modules. |
| Synthetic contracts for things that don't exist | `camera-observation.test.ts` (13: calibration and lens-provenance contracts with synthetic pinholes) | Ceremony. There is no Z70 calibration. Delete until one exists. |
| DimOS Python | ACP/receipts/queue/relay tests: duplicate IDs, epochs, symlink/fifo/hardlink refusal, "EOF after publish is unknown" | Tests the ceremony layer itself. Delete it with the layer. |
| `native_slam` Python (about 568 LOC) | Synthetic aiortc loopback and synthetic cuVSLAM tracks; "no real capture occurred" | Research. Move it out of the control-plane repo. |
| `research/*.test.ts` | Offline map/click/route reverse-engineering | Already outside `npm test`. Fine as an archive. |

**Bottom line:** roughly 45–50 tests protect real behaviour (the protocol codec, the session state machine, cloud login, go2rtc config). Roughly 260 Node tests plus about 117 Python tests pin hardening, dead code, synthetic contracts or the ceremony layers. **Not one test exercises the robot.** No hardware smoke script exists that doesn't route through the gated paths.

---

## 5. Lines of code by area (`wc -l`, comments included)

| Area | Source | Tests |
|---|---:|---:|
| TS transport (local TCP, cloud MQTT, RPC envelope, aliases) | 1,259 | 1,037 |
| TS HTTP service, auth, state store, cloud login | 1,497 | 937 |
| TS camera (go2rtc + frame metadata contracts) | 528 | 343 |
| TS MCP | 228 | 82 |
| TS map cache (on the production path, not needed for the loop) | 685 | 404 |
| TS CLI scripts, probes, onboarding | 1,427 | 247 |
| **TS dead modules (no entrypoint reaches them)** | **2,003** | **1,061** |
| Web UI (`web/src`, already a minimal RPC console) | 582 | 273 |
| **DimOS integration (`roboclaw_dimos`)** | **1,950** | **1,142** |
| DimOS `native_slam` (Py + C++) | 1,165 | 568 |
| Hermes ACP plugin | 65 | — |
| `robotics/` (MoveIt synthetic arm sim) | 625 | — |
| `research/` (TS + Android program recovery) | 1,252 | incl. |
| `deploy/` scripts | 916 | — |
| Docs (`docs/**/*.md`) | 3,999 | — |

**The production TS dependency closure of `service-main` + `mcp-main` is 4,197 LOC across 30 files.** The minimal robot loop needs about 1,500 of those: `robot.ts`, `local.ts` codec, `go2rtc.ts`, `service.ts` (rpc/state/frame), `service-auth.ts`, `mcp-server.ts` and the store.

The dead TS modules are:
- `camera.ts`, `preview.ts`: the old direct-RPC WebRTC path, which go2rtc replaces
- `vision.ts`: the Ollama VLM
- `photos.ts`, `arm-feedback.ts`, `diagnostics.ts`
- `active-map-evidence.ts`, `native-map-diff-pose-evidence.ts`, `map-list-probe.ts`, `map-raster-fixture.ts`
- eight `*-research.ts` files that `research/*.test.ts` imports from `src/`

`vendor-commands.ts` is also effectively dead (only `.command()` uses it, and nothing calls that).

---

## 6. Recommendation, ranked by impact

### P0: get a validated Hermes loop this week (mostly configuration, small code)

1. **Wire Hermes directly to the existing MCP server.**
   - Add an `mcp_servers.roboclaw` entry to `~/.hermes/config.yaml` that mirrors `opencode.json`: `node src/mcp-main.ts` with `ROBOCLAW_MCP_ORIGIN`, `ROBOCLAW_MCP_TOKEN_FILE` and `ROBOCLAW_MCP_IMAGE_HANDOFF=private-file`.
   - Hermes then gets `get_state`, `get_camera_frame` and `robot_rpc`.
   - The perception step is `get_camera_frame` → `vision_analyze(imagePath)`. The private-file handoff was built for exactly this case (commit `91bf4c7`).
   - **Disable the `roboclaw-acp` plugin.**
2. **Fix the 5 s session-killing timeout** (§2, bug 1).
   - Make the default timeout per method: 30 s for `app_arm_*`, `app_grip`, `app_open_gripper`, `app_pick_up*`, `app_put_down*` and `app_start_program`; 5 s otherwise.
   - Better still, don't tear down the session on a single response timeout. Reject only that request.
3. **Mark `get_camera_frame` read-only** and drop `app_get_arm_joints_data` from `/api/state`.
4. **First end-to-end script for Hermes**, with a person present and a hand on the robot:
   1. `get_status`.
   2. Snapshot, then `vision_analyze`.
   3. If docked: `app_rc_start`, then a 5 s wait, then `get_status` (expect 39).
   4. `app_rc_move {velocity:0, omega:0.5, duration:800, seqnum:n}`.
   5. Snapshot, then `vision_analyze` to confirm the scene rotated.
   6. `app_rc_stop`, `app_rc_end`.
   7. `app_charge`, then poll `get_status` until state 8. This is the first time docking would be validated.

   All of these are known-good wire values. **That is a validated perceive → act → verify loop with zero new abstractions.**
5. **Second loop (arm):**
   1. `app_program_enter_exit {flag:1}`. Expect it to undock.
   2. `app_start_program {cmd_ids:[{cmd_id:"arm_out",id:1}]}`.
   3. Poll `app_get_program_runtime` until `program_state` is 0.
   4. Snapshot.
   5. `arm_joints` M4 up 1, snapshot.
   6. `arm_in`.
   7. `app_program_enter_exit {flag:0}`, which has never been tested.

### P1: add thin, honest helper tools (about 150 LOC in `mcp-server.ts`)

These are convenience tools on top of `robot_rpc`. They are not gates.

- `drive(velocity, omega, durationMs)`: sends `app_rc_start` if state ≠ 39, waits 5 s, sends `app_rc_move` with an auto `seqnum`, then `app_rc_stop`. Its only bound is a duration cap of about 2 s, which is a real physical-safety limit.
- `arm_program(cmd_ids[])`: enters programming mode if needed, sends `app_start_program`, polls the runtime to a terminal state, and returns the per-record `result` values.
- `stop()`: sends `app_rc_stop` + `app_stop` + `app_stop_grasp` concurrently and returns the raw results.
- `snapshot()`: returns `imagePath` by default.
- `dock()`.
- Return raw vendor results. No `physicalCompletion` and no `lensVerification` fields.

### P2: delete (no behaviour loss)

1. **`integrations/dimos/` in its entirety** from the control path: ACP interface, relay controller, receipts, file queues, `HostedCopilotModel`/OpenCode inference, `hermes_plugin/`. About 3,100 LOC including tests. If DimOS skills are wanted later, a DimOS module can call the same MCP server directly.
2. **`integrations/dimos/native_slam/`, `robotics/moveit_sim/`, `research/android-program/`**: move them to a separate research repo or branch. They are synthetic or offline only (about 2,400 LOC).
3. **The dead TS modules** listed in §5 and their 65 tests (about 3,000 LOC). Also move the `src/*-research.ts` files into `research/`.
4. **`vendor-commands.ts`**, and the `VendorCommand`/`.command()` paths in `robot.ts:228-232` and `cloud-control.ts:190-193`.
5. **The 410 retired routes** and most named aliases in `vendor-rpc.ts`. Keep the browser `buttons` in `web/src/rpc.ts`.
6. **Docs:** archive the 25 historical policy and review docs under `docs/archive/`.
   - Keep `thin-api.md`, the 10-06 live trial, the README's chassis-timing facts and `research/z70-programming-mode-capabilities.md`.
   - **Rewrite `README.md` §"Local MCP adapter"**, which still describes the deleted authority layer.
7. **Shrink the hardening tests:** keep about 5 of the 22 secret-store tests and drop the `camera-observation` contracts.

### P3: keep as-is

- `src/local.ts` codec and `src/robot.ts` session (minus the timeout teardown). They are proven on hardware, and their tests are real.
- `src/cloud.ts` login and `src/cloud-control.ts` (cloud fallback is useful; it just hasn't been exercised).
- go2rtc for video. It works, and replacing it means reimplementing Roborock WebRTC signalling.
- Service bearer auth, the minimal web RPC console, and the Docker compose split (app + go2rtc).
- The map cache. It's harmless, but leave it out of the first loop.

### On replacing the transport with python-roborock

**Don't do it for the first loop.** The TS L01 implementation is the one component already verified on this exact robot. Switching now adds risk for no gain.

Revisit python-roborock (`RoborockLocalClientV1.send_command`, MQTT client) only if Michael wants the control plane in Python. For example: a ~200-line Python MCP server that owns the TCP session and calls go2rtc for frames. That would retire the Node service entirely. It's a reasonable phase 2, not a prerequisite.

---

## Evidence index

- Thin-gateway refactor: `git show --stat e4bb653` (+1,802 / −7,614).
- Hermes wiring: `~/.hermes/config.yaml` (plugin `roboclaw-acp` enabled, no `mcp_servers`); `opencode.json` (direct `mcp.roboclaw`).
- Hardware evidence:
  - `docs/hardware-trial-2026-10-03.md`
  - `docs/research/z70-programming-live-trial-2026-10-06.md`
  - `README.md:487-498`
  - `integrations/dimos/native_slam/PRINTER_FREE.md:35`
- Test run: `npm test`, 311 tests / 310 pass / 1 fail (`local-selection.test.ts:9`).
