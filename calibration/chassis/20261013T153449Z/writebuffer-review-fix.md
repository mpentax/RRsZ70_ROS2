> **Historical / superseded guidance — 2026-10-18 curation.** Original body below is preserved verbatim. Measurements, failed fits, cleanup uncertainty, test counts and commands describe only the dated experiment/source revision, not current runtime state or reusable authorization. Current guidance: [odometry guide](../../../src/z70_odometry/README.md), [camera guidance](../../../research/cameras/FINDINGS.md).

# WriteBuffer blocking-review fixes

## Outcome and scope

Both reported defects reproduced on the pre-fix patch and are fixed. Only `deploy/patch-go2rtc.go`, `deploy/go2rtc-mjpeg-test.go`, and this report were changed in the repository. No staging or commits. Existing MJPEG test bodies and all patch sections preceding the WriteBuffer changes were compared byte-for-byte against the index and remain unchanged. No robot/MCP access, service HTTP, production starts, restart, compose-up, running-service exec, or secret-file reads occurred.

## Fix and synchronization argument

- `writerMu` protects writer publication and a terminal `closed` flag. It is never held during transport I/O. `Close` atomically marks closed and snapshots the underlying closer, releases `writerMu`, and calls the transport's `Close` **before** acquiring the write/flush mutex `mu`.
- `Reset` holds `mu`, checks closure under `writerMu`, and publishes its target before copying buffered data. Thus an already-blocked Reset flush is cancellable through that target, rather than through the obsolete bytes.Buffer.
- `Write` rejects closure with a terminal error without touching the writer or flusher. Closed `Reset` does not replace the writer or flush buffered bytes. Existing transport errors are retained; otherwise closure records `io.ErrClosedPipe`.
- `Close` subsequently acquires `mu`, joining an admitted Write/Flush/Reset before completing. The existing HTTP cleanup joins its context callback. Queued Sender callbacks can still drain after Connection.Stop, but their writes are rejected, so they cannot touch the ResponseWriter after handler completion. No Sender/Connection rewrites, new gate, watchdog, or retry were added.
- `WriteTo` reads the terminal error under `mu` after its wait, rather than racing with late callbacks.
- Lock order for Write/Reset is `mu` then briefly `writerMu`. Close drops `writerMu` before calling the closer or acquiring `mu`, so it creates no reverse-order lock dependency.

## Regression schedules

New tests use only in-memory objects; they open no listener:

1. `TestLifecycleWriteBufferCloseCancelsBlockedTransport`: Write and buffered Reset each enter a closable transport that blocks until Close. Close must cancel and join both schedules.
2. `TestLifecycleWriteBufferTerminal`: closed Write must return zero/ErrClosedPipe without writing/flushing; closed Reset must discard buffered output, and WriteTo must return ErrClosedPipe.
3. `TestLifecycleMJPEGQueuedCancellation`: actual registered MJPEG consumers, real Sender queues, real MJPEG HTTP framing and cancelHTTPConsumer. The first HTTP frame is blocked in flight when cancellation begins; a queued callback is held until handler-equivalent WriteTo + joined cleanup completes, then released along with another queued frame. Expected writes/flushes/late operations are 1/1/0. A second consumer on the same producer still receives a frame. The fixture uses Sender.Input directly to avoid unrelated upstream Receiver graph races during removal.

The existing close-before-WriteTo and both silent HTTP endpoint regressions were preserved verbatim and pass.

## Pinned source and offline execution

Upstream: `b5948cfb25404cc5cb37b166ecaa2dca20b11d4b` (v1.9.14).

Reused dependency-populated, unpatched source image:
`roboclaw-go2rtc-diagnostics:source@sha256:a193160fc5fa116bce34be6020953cce493e48e0054413e8de8bf76e99b56701`.
Its source acquisition is the existing deployment Dockerfile's checksummed archive (`78aa79bcedec8f155e4060a379613979b0b3ee48ff62ee5164bafc0ac6532386`). Before patching, the build printed `pkg/core/writebuffer.go` SHA-256 `9358f57ebbe675ef80dcf4df95dc02d661c68e4b7d3cf528011673a1c368d4d2`, matching the local pinned pristine source.

Restricted build context: `/Users/mpentax/.hermes/cache/scratch/go2rtc-writebuffer-review` (explicit patch/test files and build recipes only). All patch, test, and compiler RUN instructions used `--network=none`; Docker builds also used `--network=none`. No docker run/exec or service deployment. Docker frontend/base metadata resolution occurred normally. The broader pre-existing diagnostic suite includes an inert Pion loopback fixture in the isolated build network; no external peers or services.

### Actual RED before implementation

Executed with the original patch preserved as scratch `patch-red.go`:

```sh
docker build --network=none --progress=plain --target transport -t roboclaw-go2rtc-writebuffer:transport-red /Users/mpentax/.hermes/cache/scratch/go2rtc-writebuffer-review
```

RC **1**, expected: both `/write` and `/reset` failed with `WriteBuffer.Close could not cancel the blocked transport`. The fixture releases its own transport to unwind the broken patch.

After the first defect's RED/GREEN cycle, the second defect's new tests were run against that preserved original patch:

```sh
docker build --network=none --progress=plain --target cancellation -t roboclaw-go2rtc-writebuffer:cancellation-red /Users/mpentax/.hermes/cache/scratch/go2rtc-writebuffer-review
```

RC **1**, expected: closed Write returned `10,<nil>` and wrote/flushed; closed Reset flushed one buffered frame and WriteTo returned nil; queued cancellation reported `HTTP writes=3 flushes=3 late=4; want 1,1,0`. No compilation or fixture failure masked these regressions.

### Actual GREEN and race repetition

```sh
docker build --network=none --progress=plain --target transport -t roboclaw-go2rtc-writebuffer:transport-green /Users/mpentax/.hermes/cache/scratch/go2rtc-writebuffer-review
docker build --network=none --progress=plain --target race -t roboclaw-go2rtc-writebuffer:race /Users/mpentax/.hermes/cache/scratch/go2rtc-writebuffer-review
docker build --network=none --progress=plain -f /Users/mpentax/.hermes/cache/scratch/go2rtc-writebuffer-review/FullDockerfile --target race -t roboclaw-go2rtc-writebuffer:full-race /Users/mpentax/.hermes/cache/scratch/go2rtc-writebuffer-review
```

All RC **0**. The transport-green image is the intermediate first-defect fix; the race/full-race images contain the final fixes.

Executed final inner commands, all RC **0**:

```sh
go test -v -count=1 -timeout=45s ./pkg/core ./pkg/flv ./pkg/mjpeg ./pkg/magic ./pkg/mp4 ./pkg/mpegts ./pkg/y4m ./internal/mjpeg
CGO_ENABLED=0 go build -trimpath -o /out/go2rtc .
go test -race -v -count=3 -timeout=90s ./pkg/core ./pkg/flv ./pkg/mjpeg ./pkg/magic ./pkg/mp4 ./pkg/mpegts ./pkg/y4m ./internal/mjpeg
# Full recipe installs the existing lifecycle + diagnostics patch/tests too:
go test -v -count=1 -timeout=90s ./pkg/mqtt ./pkg/roborock/... ./pkg/webrtc ./pkg/shell ./pkg/core ./pkg/flv ./pkg/mjpeg ./pkg/magic ./pkg/mp4 ./pkg/mpegts ./pkg/y4m ./internal/exec ./internal/mjpeg ./internal/streams ./internal/roborock
CGO_ENABLED=0 go build -trimpath -ldflags="-s -w" -o /out/go2rtc .
go test -race -v -count=3 -timeout=120s ./internal/exec ./internal/mjpeg ./internal/streams ./pkg/roborock/... ./pkg/webrtc ./internal/roborock -run '^(TestLifecycle|TestAcquisition)'
```

No race reports. Exec startup/reaping, Roborock no-reopen, other-producer reconnect, and acquisition diagnostic regressions also passed unchanged. `git diff --check -- deploy/patch-go2rtc.go deploy/go2rtc-mjpeg-test.go` returned RC 0.

Verified image IDs:

- transport-green: `sha256:99a83c03d0510fddd3d5c40757322c16700eeaa1cd87fd0ce2817ecb0d9c97d8`
- final race: `sha256:d909a6e2653438b17a2adba4897a22a1a3ab25d485560147b491c41b04efb2c3`
- full regression/compiler/race: `sha256:a6745660ad7d8d84a64fa4a78c858012e122e393d3417812d289725f7624b468`

Final repository file SHA-256:

- patch-go2rtc.go: `8e6b14371ee15ef69273224daa4bf93387c52e79f67a1a5cf6a40406ab6a1908`
- go2rtc-mjpeg-test.go: `6cb06ccf5edf2063074f63fc8e881f866e0891b777fce6547ee7bd83eadc101c`

## Limits / remaining issues

No remaining blocker was found within these two defects. This does not establish live camera delivery or production deployment. A non-closable writer whose in-flight Write/Flush never returns cannot be forcibly interrupted by WriteBuffer; cancellation waits rather than returning a handler while output is still active. A closer that itself never returns is similarly outside this wrapper's control. No watchdog was added. The closable blocked-transport regression exercises the shared core used by RTMP/FLV, but is not a live RTMP interoperability test. Some cross-format packages have no upstream tests; they compiled, and their available tests passed. The pinned source image retains the previously documented exclusion of its two non-compiling legacy stream_test.go tests; those were not edited here.
