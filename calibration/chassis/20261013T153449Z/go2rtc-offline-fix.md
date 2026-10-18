> **Historical / superseded guidance — 2026-10-18 curation.** Original body below is preserved verbatim. Measurements, failed fits, cleanup uncertainty, test counts and commands describe only the dated experiment/source revision, not current runtime state or reusable authorization. Current guidance: [odometry guide](../../../src/z70_odometry/README.md), [camera guidance](../../../research/cameras/FINDINGS.md).

# Offline go2rtc lifecycle fix

## Outcome / scope

Pinned upstream: `b5948cfb25404cc5cb37b166ecaa2dca20b11d4b`; source archive SHA-256 `78aa79bcedec8f155e4060a379613979b0b3ee48ff62ee5164bafc0ac6532386`, enforced by Docker ADD checksum and independently checked during source inspection.

- Pipe startup now receives the already-parsed `starttimeout` (default remains 30s), bounds the entire `magic.Open`, cancels/force-kills the timed-out direct child, closes stdout, joins the opener and waits for actual command reaping before returning. Successful startup stops its timer. Default pipe `WaitDelay` becomes 1s only when unset, bounding inherited command-I/O waits; explicit nonzero `killtimeout` remains honored.
- A separate shell `Reaped()` channel is needed because existing `Done()` signals cancellation before `Cmd.Wait` necessarily completes. Existing shell cancellation semantics are otherwise unchanged.
- Successfully added HTTP MJPEG/keyframe consumers now have request-context teardown with exactly-once removal and a joined cancellation callback. Silent cancellation unblocks `WriteTo` and releases the last producer. Keyframe cancellation skips subsequent transcoding/response output.
- Fixed `WriteBuffer` close-before-WriteTo state and synchronized its writer snapshot during Close: cancellation between AddConsumer and WriteTo cannot be lost.
- No new retries, reconnects, preview restarts, exposure/config changes, or robot control. Existing upstream reconnect behavior is not rewritten. Existing MQTT-baseline `regression` target still fails for shared identity; lifecycle baselines are independent targets.

Changed only: `deploy/patch-go2rtc.go`, `deploy/go2rtc.Dockerfile`; added `deploy/go2rtc-pipe-test.go`, `deploy/go2rtc-mjpeg-test.go`, this report. No commits.

## Reproduce without uploading secrets

All builds used a deliberately restricted scratch context containing only these deploy build files, not the repository context. Run from `/Users/mpentax/Code/RoboRockZ70-Connector`:

```sh
python3 -c "import pathlib,shutil; p=pathlib.Path('/Users/mpentax/.hermes/cache/scratch/go2rtc-build-context/deploy'); p.mkdir(parents=True,exist_ok=True); [shutil.copyfile('deploy/'+f,p/f) for f in ['go2rtc.Dockerfile','patch-go2rtc.go','go2rtc-clientid-test.go','go2rtc-pipe-test.go','go2rtc-mjpeg-test.go']]"
C=/Users/mpentax/.hermes/cache/scratch/go2rtc-build-context
D=$C/deploy/go2rtc.Dockerfile
S=/Users/mpentax/.hermes/cache/scratch
# Expected RED; each fails with exit code 1 and no exported image:
docker build --progress=plain --target lifecycle-baseline --iidfile "$S/go2rtc-lifecycle-red.iid" -f "$D" "$C"
docker build --progress=plain --target mjpeg-baseline --iidfile "$S/go2rtc-mjpeg-red.iid" -f "$D" "$C"
docker build --progress=plain --target regression --iidfile "$S/go2rtc-mqtt-red.iid" -f "$D" "$C"
# GREEN; all returned 0:
docker build --progress=plain --target lifecycle-race --iidfile "$S/go2rtc-lifecycle-race.iid" -f "$D" "$C"
docker build --progress=plain --target build --iidfile "$S/go2rtc-lifecycle-green.iid" -f "$D" "$C"
docker build --progress=plain --iidfile "$S/go2rtc-runtime-green.iid" -f "$D" "$C"
```

`pipe-baseline` additionally isolates the pipe-only RED for reviewers. The initial pipe-only RED was run as `lifecycle-baseline` before adding MJPEG tests, using `--iidfile "$S/go2rtc-pipe-red.iid"`; it returned 1. The initial pipe-only patched `build` command used `--iidfile "$S/go2rtc-pipe-green.iid"`; first returned 1 on a typed-nil producer cleanup panic, then returned 0 after checking the opener error before Stop. Tests were not weakened. MJPEG-only RED was then run before its implementation, both without and with the close-before-WriteTo test; both returned 1. Final combined lifecycle RED was rerun after implementation and still exercised every intended failure.

Exact Go test commands inside build stages (all `RUN --network=none`):

```sh
# Initial pipe RED and pipe-baseline:
go test -v -count=1 -timeout=30s ./internal/exec -run '^TestLifecyclePipe'
# MJPEG-only RED:
go test -v -count=1 -timeout=30s ./internal/mjpeg -run '^TestLifecycleMJPEG'
# Combined lifecycle RED:
go test -v -count=1 -timeout=30s ./internal/exec ./internal/mjpeg -run '^TestLifecycle'
# Legacy MQTT RED (baseline helper applied):
go test ./pkg/roborock/iot
# Initial pipe GREEN:
go test -v -count=1 -timeout=30s ./pkg/mqtt ./pkg/roborock/iot ./pkg/shell ./internal/exec
# Final GREEN, patched source:
go test -v -count=1 -timeout=30s ./pkg/mqtt ./pkg/roborock/iot ./pkg/shell ./pkg/core ./internal/exec ./internal/mjpeg
# Repeated race-detector GREEN:
go test -race -v -count=3 -timeout=90s ./internal/exec ./internal/mjpeg -run '^TestLifecycle'
# Full binary compilation, offline:
CGO_ENABLED=0 go build -trimpath -ldflags="-s -w" -o /out/go2rtc .
```

## Measured evidence / IDs

| Execution | RC | BuildKit failing/successful stage or exported IID |
|---|---:|---|
| Initial pipe RED | 1 | `[lifecycle-baseline 2/2]` (#17); no image exported |
| Initial patched pipe attempt (typed-nil bug) | 1 | `[build 1/2]` (#16); no image exported |
| Corrected pipe GREEN | 0 | `sha256:944e26f08c1883287729e0748a8f333747ac3e3d4ca090bc4ee3b819a6c20fa2` |
| MJPEG RED, including close-before-WriteTo | 1 | `[mjpeg-baseline 2/2]` (#18); no image exported |
| Final combined lifecycle RED | 1 | `[lifecycle-baseline 2/2]` (#18); no image exported |
| Legacy shared MQTT-ID RED | 1 | `[regression 1/1]` (#17); no image exported |
| Final normal tests + binary + repeated race tests | 0 | `sha256:44311502ee6729126c6fb0176fa77cfb362ad39c6de140e13a5a7321097b5ae8` |
| Final build stage export (cached from above) | 0 | `sha256:007b72409e95e202efce85864db93715d46f86b8f6d08f5466140f20b9fb5b24` |
| Final runtime-image build (tests/binary cached from genuine execution above) | 0 | `sha256:536edc341c4ffdc790f4acdc695c621ec7a3d2cfd361e028ea766c3a9fbe5818` |

Failed builds have no image IDs; BuildKit step labels above identify their failing stages, not persistent globally unique image IDs. Runtime-image labels were read back with `docker image inspect` and matched the pinned revision and `unique-mqtt-client-id,bounded-exec-pipe,request-canceled-mjpeg`.

Baseline failures were behavioral: silent and partial pipe headers exceeded a configured 1s deadline; pending AddConsumer remained stuck; both HTTP handlers remained waiting after request cancellation; Close before WriteTo lost cancellation; shared MQTT ID stayed shared. Patched pipe silent/partial timeout tests took about 1s each and checked immediate ESRCH for the direct child (no live or zombie child). Pending recovery test then explicitly opened a successful in-memory Y4M fixture twice on the same stream, removing consumers between opens; no automatic retry was used. Successful-source test remained alive beyond its startup deadline. Early-child-exit test stayed bounded. MJPEG tests waited through 100ms of silence after registration, canceled context, verified handler completion, producer Stop, and an empty consumer list. Race-detector tests passed on three repetitions with no race reports. Existing unique-MQTT-ID and upstream shell/core package tests passed; MQTT package itself has no upstream test files.

## Limitations / stopped-state verification

This proves offline lifecycle cleanup, not restored camera media or the cause of original robot-side UDP silence/remote-view announcement. No TCP HTTP listener was created: handler tests use `httptest.NewRequest` and cancel its request context directly, with an in-memory negotiated producer. Real transport/client disconnects, firmware preview release, decoder throughput, descendants beyond the direct child, RTSP/backchannel behavior, and real camera reopening remain unvalidated. Original source media silence remains unresolved. No production stack was started, no robot credentials were read or supplied, no robot/MCP/local HTTP contact or running-service exec occurred. Source/module/image retrieval uses public network; test, patch and compilation stages have network disabled.

Final read-only `docker ps -a --format '{{.Names}} {{.Status}}'` showed `roboclaw-app-1` and `roboclaw-go2rtc-1` both `Exited (0)`. Runtime image was built but neither loaded into an existing service nor started. `git diff --check` for owned deploy files returned 0.
