> **Historical / superseded guidance — 2026-10-18 curation.** Original body below is preserved verbatim. Measurements, failed fits, cleanup uncertainty, test counts and commands describe only the dated experiment/source revision, not current runtime state or reusable authorization. Current guidance: [odometry guide](../../../src/z70_odometry/README.md), [camera guidance](../../../research/cameras/FINDINGS.md).

# Retained acquisition diagnostics — offline build

## Outcome

Implemented a second fail-closed patcher for pinned go2rtc `b5948cfb25404cc5cb37b166ecaa2dca20b11d4b` (1.9.14). It retains only the newest Roborock camera attempt, including failures before a producer exists. `GET /api/roborock/diagnostic` returns a flat credential-free snapshot via upstream `internal/api.ResponseJSON`. Other methods return 405 without invoking any acquisition operation.

This is instrumentation, not a recovery change. No robot/MCP/service HTTP calls, production starts/restarts, Docker exec, or private configuration reads were performed by this subagent. The built images have distinct diagnostics-only tags and were not started or deployed. Existing MQTT client-ID, bounded pipe startup, MJPEG cancellation/writebuffer and Roborock no-automatic-reopen patches were left unchanged.

## Evidence context (provided by parent, not newly measured here)

- Mac boot: `2026-10-13T15:24:15Z`. A real frame subsequently succeeded at `receivedAt=1791905671401`; “total failure since reboot” is false. Failure began after idle and manual relocation; causation remains unknown.
- Latest authorized acquisition used the existing production image, not these diagnostics. It returned 502 `no_camera_image`, upstream 200. After cleanup both streams had `producers:[{}]`, `consumers:[]`; robot state 8, battery 100, preview status 0/client none; both services stopped.
- Parent verified the Docker log attach handshake (HTTP 101). Safe matched events: EOF at `1791915364363484000`, timeout at `1791915370059967000`, EOF at `1791915370063720000`; observer start `1791915352184619000`. No matched MQTT-login/PIN/SDP/connect-denial event is not proof those boundaries succeeded. Earlier classifier zeros did not establish an attached observer.
- Those logs and post-cleanup producer objects do not locate the actual failing boundary. This patch cannot reconstruct old attempts; a future explicitly authorized acquisition is needed to obtain its retained evidence.

## Instrumentation boundaries

The attempt begins inside `Client.Dial`, before URL parsing, MQTT/TLS connection or producer installation. Stage advances around the existing calls only: URL parse; TCP; TLS; MQTT admission; MQTT subscription; optional PIN; preview status/old-preview stop; preview start; TURN lookup; WebRTC API/peer creation; offer generation/send; answer polling/application; wait-connected; connected.

The MQTT RPC codec records categorized underlying read EOF/timeout/closed errors before `net/rpc` can hide them behind `ErrShutdown`. An EOF/timeout already observed is not overwritten by a later RPC shutdown. Answer/ICE operations that upstream already ignores retain the first observed safe error category; no call, retry or deadline was added. TURN failure remains nonfatal as in upstream and is visible separately in `turn`.

`NewConn` retains its original behavior and delegates to a diagnostics-aware constructor with a nil observer. Roborock supplies its attempt at construction. The existing Pion ICE, selected-pair, OnTrack, RTP read/unmarshal and PC callbacks are augmented in place, not replaced. DTLS adds its public state callback (upstream Conn has none). The Pion 4.2.3 loopback test confirms ICE/DTLS/PC callbacks and incoming video remain functional.

Selected pair evidence contains candidate **types** and remote transport protocol only. Video counters count video OnTrack callbacks and successfully parsed incoming RTP packets; bytes include the RTP header, not only payload. Audio is excluded. Reader exits distinguish EOF, timeout, closed transport, generic read error, malformed RTP and track setup failure.

Before deliberate stop, peer state is sampled/frozen before closing the MQTT RPC transport and PC. Later PC/ICE/DTLS closed callbacks cannot overwrite the retained transport state. Video reader exit remains writable after Close so cleanup retains its exit category; cleanup-induced MQTT read errors do not become acquisition failures. All retained updates/snapshots use mutexes. Replacing the latest pointer does not let old callbacks mutate the newer attempt.

## Exact API schema

The parent later extended the retained snapshot with a signed numeric vendor error code and a closed reason enum, using a real encrypted MQTT response-codec fixture (RED then GREEN) and three race-detector repetitions. Final live evidence: MQTT/PIN succeeded, `start_camera_preview` returned -10013 before transport setup, and hidden warmup fallback removal reduced one MCP capture to one acquisition. The exact field additions are listed below; original child-build results remain historical evidence, not a claim that robot video recovered.

No fields are omitted, including zero counters. Only the following keys exist:

| Key | Type / allowed values |
| --- | --- |
| `available` | boolean |
| `attempt` | nonnegative integer, process-local monotonically increasing attempt counter; not a source/client/device ID |
| `elapsedMs` | nonnegative integer; live elapsed while Dial/Connect is pending, frozen when Dial fails or Connect returns |
| `stage` | `none`, `unknown`, `parse_url`, `tcp`, `tls`, `mqtt`, `subscribe`, `pin`, `status`, `stop_preview`, `start_preview`, `turn`, `create_api`, `create_peer`, `create_offer`, `send_offer`, `get_answer`, `set_answer`, `wait_connected`, `connected` |
| `failure` | `none`, `invalid_url`, `error`, `eof`, `closed`, `timeout`, `mqtt_rejected`, `pin_rejected`, `rpc_error`, `camera_not_ready`, `answer_timeout`, `connect_failed` |
| `vendorCode` | signed safe integer, zero if no vendor rejection captured; preserve the signed wire code |
| `vendorReason` | `none`, `unknown`, `permission_denied`, `busy`, `invalid_params`, `unsupported`, `not_allowed`, `camera_disabled`; unknown text is never retained |
| `mqtt` | `unknown`, `ok`, `error`, `eof`, `closed`, `timeout`, `rejected`, `rpc_error` |
| `pin` | same outcome enum as `mqtt`, plus `skipped` |
| `turn` | same outcome enum as `mqtt` |
| `pcState` | `unknown`, `new`, `connecting`, `connected`, `disconnected`, `failed`, `closed` |
| `iceState` | `unknown`, `new`, `checking`, `connected`, `completed`, `disconnected`, `failed`, `closed` |
| `dtlsState` | `unknown`, `new`, `connecting`, `connected`, `closed`, `failed` |
| `localCandidateType` | `unknown`, `host`, `srflx`, `prflx`, `relay` |
| `remoteCandidateType` | same candidate-type enum |
| `candidateProtocol` | `unknown`, `udp`, `tcp` |
| `videoTracks` | nonnegative integer |
| `videoPackets` | nonnegative integer |
| `videoBytes` | nonnegative integer |
| `readerExit` | `none`, `unknown`, `eof`, `timeout`, `closed`, `read_error`, `rtp_error`, `unmapped_track`, `track_error` |

Before any attempt: `available=false`; all numeric fields zero; `stage=none`, `failure=none`, `readerExit=none`; other enums `unknown`. The response is an in-memory latest attempt, not a history or persistent journal. It survives stream/preview cleanup but not process restart. An available snapshot is not a camera success signal.

`wrong login` and `wrong pin code` are compared only against fixed upstream literals; raw errors are never stored. Typed `net.Error`, EOF/UnexpectedEOF, closed/ErrShutdown and `rpc.ServerError` map to closed categories. Generic errors remain `error`; a category cannot identify the detailed robot RPC denial. No URLs, SDP, candidate text, addresses/ports, TURN fields, client/device IDs or raw errors enter the snapshot.

## Offline verification

The restricted scratch build context contained only explicitly named deploy Dockerfile/patch/test files. No repo-wide context or private files were copied. Source/module dependency acquisition is a separate prerequisite: the pinned public source and Go modules were fetched/cached before tests. An initial global `docker build --network=none` against the unprimed source stage failed at `go mod download`, not at a test. To exercise the compiler/tests with the **global** network restriction, the source stage was materialized as `roboclaw-go2rtc-diagnostics:source`, then a scratch `offline.Dockerfile` substituted that already populated image for the source stage and retained the exact downstream Dockerfile stages. All patch/test/compiler RUNs also explicitly use `--network=none` in the committed integration.

Commands exercised (context `/Users/mpentax/.hermes/cache/scratch/go2rtc-diagnostics-context`):

```sh
# Dependency/source stage only, no application start:
docker build --target source -t roboclaw-go2rtc-diagnostics:source \
  -f "$CTX/deploy/go2rtc.Dockerfile" "$CTX"
# offline.Dockerfile = FROM populated source + unchanged downstream stages.
docker build --network=none --target diagnostics-baseline \
  -f "$CTX/offline.Dockerfile" "$CTX"
docker build --network=none --target diagnostics-race \
  -t roboclaw-go2rtc-diagnostics:race -f "$CTX/offline.Dockerfile" "$CTX"
docker build --network=none --target lifecycle-race \
  -t roboclaw-go2rtc-diagnostics:lifecycle-race -f "$CTX/offline.Dockerfile" "$CTX"
docker build --network=none -t roboclaw-go2rtc-diagnostics:runtime \
  -f "$CTX/offline.Dockerfile" "$CTX"
```

Measured results:

- **RED:** unchanged `TestAcquisitionDiagnosticRetainsPreProducerFailure` against pinned baseline failed with `retained acquisition diagnostic missing: status=404`. It intentionally supplies a malformed URL and therefore performs no network call. The same test becomes GREEN after both patches.
- **GREEN:** all eight named diagnostic tests passed, plus the relevant existing package tests. In-memory RPC fixtures execute real `Client.Connect` failures at status, old-preview stop, preview start, offer send, answer-read EOF and invalid answer application. PIN fixtures distinguish success, RPC denial and real false-result PIN rejection. Every stage's fixed failure category is also covered by a unit table; TCP/TLS/API/peer/offer creation failure injection is category-unit coverage, not a real robot/full-stack handshake test.
- **Race:** `go test -race -v -count=3 -timeout=90s ./pkg/roborock/... ./pkg/webrtc ./internal/roborock -run '^TestAcquisition'` passed without races. The concurrent latest-snapshot test has multiple writers/readers and checks exact totals and old-attempt isolation.
- **Preservation:** the original pipe, MJPEG and no-reopen lifecycle regressions passed three times under `-race` after the diagnostics patch. The same two obsolete upstream stream tests remain excluded identically in baseline and patched builds; no new exclusion was added.
- **Real Pion:** UDP4 loopback-only peers (no STUN, TURN, DNS or robot) negotiated ICE/DTLS and delivered video RTP under Docker network-none. Final race samples retained `pcState=connected`, `iceState=connected`, `dtlsState=connected`, local/remote type `host`, protocol `udp`, `videoTracks=1`, `videoPackets=1`, `videoBytes=16`, `readerExit=eof` after Close. Measured elapsed samples were 33, 33 and 35 ms. The isolated Pion test bypasses Roborock Dial/Connect, hence its `stage=none` and MQTT/PIN/TURN `unknown` are intentional, not production evidence.
- **Runtime compiler:** `CGO_ENABLED=0 go build -trimpath -ldflags="-s -w" -o /out/go2rtc .` succeeded offline. Runtime image was built and inspected, not started.

Final inspected images:

- runtime: `sha256:37b1018613550590ce2d0b8a57b4014df056807e4d729c4b1449ae6f2acf25de`
- diagnostics race: `sha256:590719017f3c069302897a9ea873e5a5833e365dbd4803a758cb33bf6c3a6962`
- lifecycle race: `sha256:8476de9fb7ea9cb1461aa2c2f75edd572c5e20458777c6517b2accf691c98ec9`

## Files / handoff

Created:

- `deploy/patch-go2rtc-diagnostics.go`
- `deploy/go2rtc-diagnostics-test.go`
- `deploy/go2rtc-diagnostics-state-test.go`
- `deploy/go2rtc-diagnostics-connect-test.go`
- `deploy/go2rtc-diagnostics-pion-test.go`
- `deploy/go2rtc-diagnostics-mqtt-test.go`
- this report

Modified only `deploy/go2rtc.Dockerfile` (second patch/test, baseline and race targets) and `deploy/go2rtc.yaml` (one new allow-path). No commits. Parent owns TypeScript whitelist/plumbing, restricted deployment-context allowlisting, independent review and any separately authorized deployment/acquisition.

Limitations: the snapshot has the current/terminal stage and one elapsed duration, not per-stage history. Ignored answer/ICE failures have categories but no operation-specific subfield; the stage and transport states must be interpreted together. A video reader EOF after intentional Close is expected and must not be described as a spontaneous camera fault. Candidate/PC state cannot prove successful JPEG decoding. Decoding/JPEG delivery remain outside this instrumentation. Upstream timing, retry loops, failure cleanup behavior and pending-Connect waits were not repaired or bounded here.
