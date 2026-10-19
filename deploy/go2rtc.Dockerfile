# syntax=docker/dockerfile:1
FROM golang:1.24-bookworm@sha256:1a6d4452c65dea36aac2e2d606b01b4a029ec90cc1ae53890540ce6173ea77ac AS build
WORKDIR /src
ADD --checksum=sha256:78aa79bcedec8f155e4060a379613979b0b3ee48ff62ee5164bafc0ac6532386 https://codeload.github.com/AlexxIT/go2rtc/tar.gz/b5948cfb25404cc5cb37b166ecaa2dca20b11d4b /source.tar.gz
RUN tar -xzf /source.tar.gz --strip-components=1 -C /src && rm /source.tar.gz
RUN --network=default go mod download
COPY deploy/patch-go2rtc.go /patch-go2rtc.go
COPY deploy/patch-go2rtc-diagnostics.go /patch-go2rtc-diagnostics.go
RUN --network=none go run /patch-go2rtc.go && go run /patch-go2rtc-diagnostics.go && CGO_ENABLED=0 go build -trimpath -ldflags="-s -w" -o /out/go2rtc .

FROM alexxit/go2rtc:1.9.14@sha256:675c318b23c06fd862a61d262240c9a63436b4050d177ffc68a32710d9e05bae
COPY --from=build /out/go2rtc /usr/local/bin/go2rtc
LABEL org.roboclaw.camera-fix="unique-mqtt-client-id,bounded-exec-pipe,request-canceled-mjpeg,roborock-no-automatic-reopen" org.roboclaw.go2rtc-upstream="b5948cfb25404cc5cb37b166ecaa2dca20b11d4b"
