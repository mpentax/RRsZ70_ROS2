# syntax=docker/dockerfile:1
FROM golang:1.24-bookworm@sha256:1a6d4452c65dea36aac2e2d606b01b4a029ec90cc1ae53890540ce6173ea77ac AS source
WORKDIR /src
# v1.9.14, the exact commit used by the previous production image.
ADD --checksum=sha256:78aa79bcedec8f155e4060a379613979b0b3ee48ff62ee5164bafc0ac6532386 https://codeload.github.com/AlexxIT/go2rtc/tar.gz/b5948cfb25404cc5cb37b166ecaa2dca20b11d4b /source.tar.gz
RUN tar -xzf /source.tar.gz --strip-components=1 -C /src && rm /source.tar.gz
COPY deploy/patch-go2rtc.go /patch-go2rtc.go
COPY deploy/go2rtc-clientid-test.go /camera_clientid_test.go
# Dependency acquisition only; every patch, test and compiler RUN below is offline.
RUN --network=default go mod download
# Upstream's two legacy stream_test.go tests do not compile against its own
# current two-result API. Keep the original outside the Go test file glob;
# production source, existing API tests and all local regressions stay intact.
RUN mv internal/streams/stream_test.go /upstream-stream_test.go.disabled

COPY deploy/go2rtc-pipe-test.go /camera_pipe_test.go
COPY deploy/go2rtc-mjpeg-test.go /camera_mjpeg_test.go
COPY deploy/go2rtc-streams-test.go /camera_streams_test.go

# Expected RED: exact pinned worker automatically reopens Roborock previews.
FROM source AS streams-baseline
RUN cp /camera_streams_test.go internal/streams/camera_streams_test.go
RUN --network=none go test -v -count=1 -timeout=30s ./internal/streams -run '^TestLifecycle'

# Independent expected-RED lifecycle stage; legacy MQTT regression remains unchanged.
FROM source AS lifecycle-baseline
RUN cp /camera_pipe_test.go internal/exec/camera_pipe_test.go && cp /camera_mjpeg_test.go internal/mjpeg/camera_mjpeg_test.go
RUN --network=none go test -v -count=1 -timeout=30s ./internal/exec ./internal/mjpeg -run '^TestLifecycle'

# Isolate each RED independently so neither masks the other.
FROM source AS pipe-baseline
RUN cp /camera_pipe_test.go internal/exec/camera_pipe_test.go
RUN --network=none go test -v -count=1 -timeout=30s ./internal/exec -run '^TestLifecyclePipe'
FROM source AS mjpeg-baseline
RUN cp /camera_mjpeg_test.go internal/mjpeg/camera_mjpeg_test.go
RUN --network=none go test -v -count=1 -timeout=30s ./internal/mjpeg -run '^TestLifecycleMJPEG'

# Reproduce the regression without contacting a robot or broker.
FROM source AS regression
RUN --network=none go run /patch-go2rtc.go baseline && cp /camera_clientid_test.go pkg/roborock/iot/camera_clientid_test.go && go test ./pkg/roborock/iot

# Expected RED: acquisition failure disappears before the producer is installed.
FROM source AS diagnostics-baseline
COPY deploy/go2rtc-diagnostics-test.go /camera_diagnostics_test.go
RUN cp /camera_diagnostics_test.go internal/roborock/camera_diagnostics_test.go
RUN --network=none go test -v -count=1 -timeout=30s ./internal/roborock -run '^TestAcquisition'

FROM source AS build
COPY deploy/patch-go2rtc-diagnostics.go /patch-go2rtc-diagnostics.go
COPY deploy/go2rtc-diagnostics-test.go /camera_diagnostics_test.go
COPY deploy/go2rtc-diagnostics-state-test.go /camera_diagnostics_state_test.go
COPY deploy/go2rtc-diagnostics-connect-test.go /camera_diagnostics_connect_test.go
COPY deploy/go2rtc-diagnostics-pion-test.go /camera_diagnostics_pion_test.go
COPY deploy/go2rtc-diagnostics-mqtt-test.go /camera_diagnostics_mqtt_test.go
RUN --network=none go run /patch-go2rtc.go && cp /camera_clientid_test.go pkg/roborock/iot/camera_clientid_test.go && cp /camera_pipe_test.go internal/exec/camera_pipe_test.go && cp /camera_mjpeg_test.go internal/mjpeg/camera_mjpeg_test.go && cp /camera_streams_test.go internal/streams/camera_streams_test.go && gofmt -w pkg/roborock/iot/client.go pkg/roborock/iot/camera_clientid_test.go internal/exec/exec.go internal/exec/camera_pipe_test.go pkg/shell/command.go internal/mjpeg/mjpeg.go internal/mjpeg/camera_mjpeg_test.go pkg/core/writebuffer.go internal/streams/producer.go internal/streams/camera_streams_test.go && go test -v -count=1 -timeout=30s ./pkg/mqtt ./pkg/roborock/iot ./pkg/shell ./pkg/core ./internal/exec ./internal/mjpeg ./internal/streams
RUN --network=none go run /patch-go2rtc-diagnostics.go && cp /camera_diagnostics_test.go internal/roborock/camera_diagnostics_test.go && cp /camera_diagnostics_state_test.go pkg/roborock/diagnostic/camera_diagnostics_state_test.go && cp /camera_diagnostics_connect_test.go pkg/roborock/camera_diagnostics_connect_test.go && cp /camera_diagnostics_pion_test.go pkg/webrtc/camera_diagnostics_pion_test.go && cp /camera_diagnostics_mqtt_test.go pkg/roborock/iot/camera_diagnostics_mqtt_test.go && gofmt -w pkg/roborock pkg/webrtc/conn.go pkg/webrtc/camera_diagnostics_pion_test.go internal/roborock && go test -v -count=1 -timeout=30s ./pkg/roborock/... ./pkg/webrtc ./internal/roborock
RUN --network=none CGO_ENABLED=0 go build -trimpath -ldflags="-s -w" -o /out/go2rtc .

# Repetition under the race detector, still entirely offline.
FROM build AS lifecycle-race
RUN --network=none go test -race -v -count=3 -timeout=90s ./internal/exec ./internal/mjpeg ./internal/streams -run '^TestLifecycle'

FROM build AS diagnostics-race
RUN --network=none go test -race -v -count=3 -timeout=90s ./pkg/roborock/... ./pkg/webrtc ./internal/roborock -run '^TestAcquisition'

# Preserve the existing sidecar's FFmpeg, certificates, init and runtime layout.
FROM alexxit/go2rtc:1.9.14@sha256:675c318b23c06fd862a61d262240c9a63436b4050d177ffc68a32710d9e05bae
COPY --from=build /out/go2rtc /usr/local/bin/go2rtc
LABEL org.roboclaw.camera-fix="unique-mqtt-client-id,bounded-exec-pipe,request-canceled-mjpeg,roborock-no-automatic-reopen" org.roboclaw.go2rtc-upstream="b5948cfb25404cc5cb37b166ecaa2dca20b11d4b"
