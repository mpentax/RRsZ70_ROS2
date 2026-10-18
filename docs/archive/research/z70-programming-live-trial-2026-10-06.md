> **Historical / superseded — retained in place.** Current owners: [ROS guide](../../ros2.md), [command/API contract](../../thin-api.md), [arm model](../../z70-arm-model.md).
> The record below preserves its original observations, source pins and reported results. Its plans, commands, permissions and deployment statements are not present-day instructions or reusable live authorization.

Operator-reported entry undock, unsaved accepted payloads, runtime and zero-joint observations remain unchanged. Final unverified stow/unexited session describe that session only, not the current robot state or a universal firmware result.

## Historical record

# Supervised Z70 programming-mode live trial — 2026-10-06

Status: paused awaiting operator stow/stationary confirmation; cancellation and exit tests not performed. Operator: Michael; requested live tests, reported robot initially docked and supervised. All robot interactions through RoboClaw MCP `robot_rpc` or `get_state`, local L01; no direct HTTP/shell robot transport, no auto retries, no camera activation, no program saving/deletion. Parent exclusively controls robot. Read-only parallel source audit checked primary Android disassembly. Firmware version not queried; model roborock.vacuum.a179.

## Boundary and source evidence

Checkout HEAD efc8cf5; untracked research preserved. Running roboclaw-app-1 healthy. Host/container SHA256 parity verified for src/mcp-server.ts, src/vendor-rpc.ts, src/robot.ts. Thin generic RPC forwards original method/params, no workflows. Each reply reports physicalCompletion=unverified; physical evidence below is explicitly operator-reported.

Protocol references: z70-programming-mode-capabilities.md; arm-program-research.md. Primary index.android.hasm: entry wrapper H230922–230946; runtime H230763–230774; sound list H230779–230790; inventory H230795–230811; start H231042–231064; minimal cmd_ids envelope H1432004–1432018; fixture records H1430234–1430252; M3/M4/M5/M6 mapping H1429772–1429790; direction enum H1428502; ordinary AppStop=app_stop H348793. Raw joint query H231211–231222. Model RecordRealPos is predicted state, not an RPC.

## Evidence ledger

Times below are service startedAt Unix milliseconds; request IDs are service correlation, not firmware run IDs. All listed mutations received ["ok"]. Runtime command result 0 interpreted as vendor-reported success, not measured pose.

| Trial | Mutation timestamp / requestId | Vendor/runtime evidence | Operator evidence |
|---|---|---|---|
| Baseline | runtime 1791299989445 / 85dc4e67-c6e4-4eb0-9b96-1b312c8c655d | state8, battery100, error0; runtime {program_state:0,result:null}; arm {data:[0,0,0]} | Initially docked, supervised |
| Programming entry | 1791299996353 / 1869dbeb-9abc-40eb-a9d0-3abe9814c98f | flag1 ACK; status8→42; runtime1/null, later0/null at1791300065085 | Moved off dock, then stationary; arm stowed; area clear |
| Single delay | 1791300070055 / 4bb86e07-e9b2-482b-95f3-ffd2eab120c2 | [{cmd_id:delay,time:3,id:8}]; at +7.7s runtime0, command result0 | No individual physical timing measured |
| Delay→sound | 1791300100033 / 031c3476-083b-402a-9e5e-c7f05ebbac80 | at+5.2s runtime2, delay0,sound2; at+15.2s runtime0,both0 | Heard sound; stationary, arm stowed, area clear |
| Arm-out only | 1791300204667 / 4fbee99c-4b59-4309-9a6c-f9e983ee9c9a | at+12.7s runtime2/entry2; at+20.3s runtime0/entry0; generic joint data stayed zero | Extended and stationary; area clear |
| M4 up1 | 1791300247046 / 1a4b4c76-c72a-46d0-a4fc-6690616a7db8 | runtime0/entry0 at+5.3s | One small upward movement, stationary |
| M4 down1 | 1791300309049 / 5044dbc1-4364-4774-84c5-f44cdcfc7c81 | runtime0/entry0 at+4.8s | Small downward reversal; stationary and clear |
| M4 up→delay3→down | 1791300380590 / 7b731bed-6813-4c3c-975a-5f797ce6913a | runtime0/all three0 at+6.5s | Operator missed observation; NOT physically verified |
| Same sequence, explicitly requested fresh repeat | 1791300463449 / 18df3d9d-048b-48c4-82ef-ef449523465b | runtime0/all three0 at+11s; joint query at+6.1s zero | Confirmed up, pause, down, stationary |
| Held-offset M4 up1 (operator-requested diagnostic refinement) | 1791300554945 / c6c7b224-3e69-479b-9507-578c510b2ed3 | runtime0/entry0; raw joints at1791300567558 and1791300619029 both {data:[0,0,0]} | Confirmed held higher and stationary; NO return sent before either query |
| Arm-in only | 1791300627925 / 6df09aa1-4a1b-47f6-a447-056d21f58e8f | at+6.4s runtime2/entry2; at1791300691929 (+64s) runtime0/entry0 | Confirmation prompt received no answer within120s; physical stow unverified |

One initial intended delay-start tool invocation was explicitly skipped by runtime for incoming research completion; fresh runtime read preceded actual submission. No uncertain-dispatch mutation was retried.

## Exact accepted payloads

Method app_program_enter_exit; params {"flag":1}.
Method app_start_program, params respectively:

```json
{"cmd_ids":[{"cmd_id":"delay","time":3,"id":8}]}
{"cmd_ids":[{"cmd_id":"delay","time":3,"id":1},{"cmd_id":"sound","value":"code_mode_other_2","id":2}]}
{"cmd_ids":[{"id":1,"cmd_id":"arm_out"}]}
{"cmd_ids":[{"cmd_id":"arm_joints","id":6,"value":[0,1,0,0],"step":1}]}
{"cmd_ids":[{"cmd_id":"arm_joints","id":6,"value":[0,2,0,0],"step":1}]}
{"cmd_ids":[{"cmd_id":"arm_joints","id":1,"value":[0,1,0,0],"step":1},{"cmd_id":"delay","time":3,"id":2},{"cmd_id":"arm_joints","id":3,"value":[0,2,0,0],"step":1}]}
{"cmd_ids":[{"id":1,"cmd_id":"arm_in"}]}
```

## Read API discoveries

- app_get_program_soundlist {} returned23 sound keys including code_mode_other_2 (used successfully).
- app_get_programs_summary {} returned presets1 Say Hi,2 Deliver items,3 Dance and one user copy. No inventory mutation.
- app_get_program {program_id:1} returned actual stored cmd_ids and Blockly blocks. Read only, NOT run. Stored sequence: arm_out,delay3,arm_move up1/down1,sound,arm_move up1/down1,delay2,sound,arm_move up1/down1,delay3,arm_move up1/down1,arm_in. This strengthens envelope/arm_move evidence beyond static fixture, but arm_move itself not live-qualified in this trial.
- Direct get_status [] at1791300480423: state42,battery98,error0,common_status16578, no measured joint vector. Do not infer docking from charge_status1 (operator observed off dock).
- Runtime returns command records with original values plus result; no firmware run identifier observed. IDs6 and8 work in single-record programs; sequential1..3 work for chains. General ID bounds not established.

## Findings and limits

1. Entry can physically undock from charging. It is NOT inert setup. In this trial it did not deploy arm.
2. Unsaved single-record and three-record programs work with only cmd_ids; no program_id/name/Blockly metadata needed for tested starts.
3. Runtime state2 and per-command result2 observed in flight; state0 plus result0 after. Runtime0 can retain prior records; it is NOT proof of inactive programming session or measured queue emptiness.
4. Native M4 relative step up/down and robot-side up/pause/down confirmed by operator. No calibrated degrees, timing, repeat counts>1, motion-overlap/settling guarantee or absolute-position control established.
5. Joint data zero both stowed/deployed is insufficient alone. Stronger held-offset experiment also returned zero twice despite independently reported changed held pose. Thus this endpoint did not provide useful M4 position feedback in tested mode. Meaning of three values unresolved; do not generalize to all firmware/modes.
6. No cameras activated (avoids mode disturbance). Observations are operator reports, not images or measured trajectories.
7. Gripper, M3/M5, arm_move, larger steps, chassis program motions, save/delete and motor-emergency-stop tests remain untested.

## Paused state

Operator stow-verification prompt timed out after120s. No further mutations issued. Fresh get_state at1791300832843: state42,battery97,error0,in_cleaning0,in_returning0; joint data still zero. Last runtime read reports arm_in success with program_state0. Physical final stow/stationarity not independently confirmed. Programming flag0, app_stop, and charging were NOT sent; programming session remains unexited by this controller. No queued future robot actions or scheduled trial continuation. Cancellation/exit qualification deferred until operator supplies current physical state and supervision. Test task remains open, not marked complete.
