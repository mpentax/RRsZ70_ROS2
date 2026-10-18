> **Historical / superseded guidance — 2026-10-18 curation.** Original body below is preserved verbatim. Measurements, failed fits, cleanup uncertainty, test counts and commands describe only the dated experiment/source revision, not current runtime state or reusable authorization. Current guidance: [odometry guide](../../../src/z70_odometry/README.md), [camera guidance](../../../research/cameras/FINDINGS.md).

# go2rtc Roborock producer: no automatic preview reopen

## Outcome

Offline RED/GREEN and three race-detector repetitions verified against exact upstream commit `b5948cfb25404cc5cb37b166ecaa2dca20b11d4b` (v1.9.14), archive SHA256 `78aa79bcedec8f155e4060a379613979b0b3ee48ff62ee5164bafc0ac6532386`.

Roborock worker exit, with either nil or error result from Start, now stops the old connection and returns the producer to its empty/idle state without invoking GetProducer or scheduling reconnect. A fresh explicit Stream.AddConsumer after caller cleanup opens once again. Other schemes retain upstream reconnect and track-transfer behavior. Stale workers are rejected before the Roborock-specific branch.

No services or robot sessions were started, no robot MCP/local HTTP used, no running-service Docker exec, no secret/.env/.local reads, and no commits. Docker activity was build and image metadata only. No service deployment was performed.

## Root cause and minimal change

Pinned `internal/streams/producer.go:160-174` always calls `reconnect(workerID, 0)` after Start returns. The existing reconnect path invokes GetProducer again and then retries with timers when dialing fails. For Roborock, opening a producer starts remote preview; this is a preview-producing operation, not merely reconnecting an inert transport.

`deploy/patch-go2rtc.go` adds a fail-closed exact-anchor patch immediately after the existing stale-worker check, inside reconnect's existing mutex, before GetProducer or its retry scheduling. `strings.HasPrefix(p.url, "roborock:")` matches the exact scheme and does not match `roborock-fixture:`. The branch increments workerID, calls the old connection's Stop, clears connection/receivers/senders, sets stateNone, and returns. No added production timers, loop, gate, or architecture changes. Existing MQTT client-ID, pipe timeout/child-reaping and MJPEG cancellation patches are retained unchanged.

## Behavioral tests

New `deploy/go2rtc-streams-test.go` uses real Stream.AddConsumer/GetProducer and a memory-only handler registered under `roborock`:

- `TestLifecycleRoborockNoAutomaticReopen`, subtests `normal-exit` and `transport-error`: negotiated consumer, started fixture, deliberate Start return, exactly one Dial, stopped old connection, nil connection/empty producer track state, detached consumer track, caller removal leaves zero consumer registrations, and no further Dial during 1.1 seconds spanning upstream's first retry interval. Next explicit AddConsumer starts a second fixture, with total Dial count exactly two.
- `TestLifecycleOtherProducerStillReconnects`: `roborock-fixture:` still dials a replacement, stops the old connection, and transfers its consumer track to the replacement.
- `TestLifecycleRoborockStaleWorkerPreservesNewGeneration`: demand/removal/new demand creates a newer generation; deterministic stale reconnect completion cannot stop it, change its state/connection, detach its consumer or issue another Dial.

Fixture Start is channel-controlled; no network listeners, credentials, broker or hardware. Sleeps exist only in tests, not the patch.

## Reproduction commands

Restricted context directory used:

`/Users/mpentax/.hermes/cache/scratch/go2rtc-no-reopen/context`

It contains only these six explicitly copied files under `deploy/`:

`go2rtc.Dockerfile`, `patch-go2rtc.go`, `go2rtc-clientid-test.go`, `go2rtc-pipe-test.go`, `go2rtc-mjpeg-test.go`, `go2rtc-streams-test.go`.

The source/archive, base images and Go modules may be downloaded during preparation. Every test/patch/binary-build RUN uses `--network=none`.

```sh
CTX=/Users/mpentax/.hermes/cache/scratch/go2rtc-no-reopen/context
DF="$CTX/deploy/go2rtc.Dockerfile"
# Expected exit 1; not a runnable baseline image:
docker build --progress=plain --target streams-baseline \
  -t roboclaw-go2rtc-no-reopen:baseline -f "$DF" "$CTX"
# Expected exit 0; includes package tests and binary compilation:
docker build --progress=plain --target lifecycle-race \
  -t roboclaw-go2rtc-no-reopen:race -f "$DF" "$CTX"
# Runtime artifact only; never started:
docker build --progress=plain \
  -t roboclaw-go2rtc-no-reopen:green -f "$DF" "$CTX"
docker image inspect roboclaw-go2rtc-no-reopen:green \
  roboclaw-go2rtc-no-reopen:race \
  --format '{{.RepoTags}} {{.Id}} {{json .RepoDigests}}'
```

Observed RED output, before implementing the producer patch and again with the final baseline stage:

```text
TestLifecycleRoborockNoAutomaticReopen/normal-exit:
  automatic preview-producing reconnect: Dial count=2, want 1
TestLifecycleRoborockNoAutomaticReopen/transport-error:
  automatic preview-producing reconnect: Dial count=2, want 1
PASS: TestLifecycleOtherProducerStillReconnects
PASS: TestLifecycleRoborockStaleWorkerPreservesNewGeneration
FAIL github.com/AlexxIT/go2rtc/internal/streams
```

Observed GREEN commands inside the image build:

```sh
go test -v -count=1 -timeout=30s ./pkg/mqtt ./pkg/roborock/iot \
  ./pkg/shell ./pkg/core ./internal/exec ./internal/mjpeg ./internal/streams
CGO_ENABLED=0 go build -trimpath -ldflags="-s -w" -o /out/go2rtc .
go test -race -v -count=3 -timeout=90s \
  ./internal/exec ./internal/mjpeg ./internal/streams -run '^TestLifecycle'
```

All executed tests passed; `pkg/mqtt` has no test files. Existing MQTT client-ID test passed. Existing pipe/MJPEG lifecycle cases passed in normal tests and all three race repetitions. Streams normal suite completed in 2.211s, streams race suite in 7.634s; pipe race in 13.924s; MJPEG race in 1.627s. No race detector findings. Runtime binary compiled successfully. `git diff --check -- deploy/patch-go2rtc.go deploy/go2rtc.Dockerfile` passed.

## Images verified by read-back

- Runtime `roboclaw-go2rtc-no-reopen:green`: `sha256:bb2dee73530d9e9bd4b34dededac62a9db39b8e215b1c407614629e6d24d0b94`.
- Test/build `roboclaw-go2rtc-no-reopen:race`: `sha256:7e9d6fca17448113acc31da52ea463c1beb10d0b74b6b1e38949637be30dbffd`.
- No baseline image produced, because expected RED fails its build.

## Issues and limits

1. Pinned upstream `internal/streams/stream_test.go` itself does not compile: four one-result assignments call current two-result New/GetOrPatch/Patch APIs. Attempting assignment-only compatibility also revealed obsolete unsupported-source assumptions. Final Dockerfile transparently moves that original file outside Go's test glob in the common source stage, for both RED and GREEN; its two legacy tests (`TestRecursion`, `TestTempate`) are not claimed as verified. Existing streams API tests and new behavioral regressions remain enabled. Production source is unchanged in baseline.
2. Producer Stop detaches consumer tracks. The generic producer does not own the Stream consumer registration or caller buffers: the caller must still RemoveConsumer (as tested), or the preserved HTTP cancellation patch performs that cleanup when its request is canceled. This patch does not claim immediate automatic closure of every consumer request when a producer exits.
3. A negotiated producer that remains inside Start indefinitely but delivers no media is not a worker exit; media silence remains unresolved. No new watchdog/timer was added.
4. No physical preview-stop, announcement suppression, camera media, authentication, or network/hardware validation is claimed. Existing Roborock Stop closes MQTT/WebRTC but does not explicitly send a remote-preview-stop RPC; that behavior was not changed.
5. Exact original URL scheme only: redirect/exec wrappers are intentionally not classified as Roborock here. Non-Roborock retry scheduling was left intact; the regression verifies successful replacement and transfer, not every backoff duration.

## Files

Modified: `deploy/patch-go2rtc.go`, `deploy/go2rtc.Dockerfile`.
Created: `deploy/go2rtc-streams-test.go`, this report.
Existing `deploy/go2rtc-clientid-test.go`, `deploy/go2rtc-pipe-test.go`, `deploy/go2rtc-mjpeg-test.go` were not modified.
