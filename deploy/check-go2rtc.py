"""Offline container checks: no robot, credentials, network, or published ports."""
import json
import os
import subprocess
import time
import urllib.error
import urllib.request

base = "http://127.0.0.1:57001"
process = subprocess.Popen(
    ["go2rtc", "-config", "/etc/go2rtc.yaml"],
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL,
)


def request(path, method="GET"):
    return urllib.request.urlopen(
        urllib.request.Request(base + path, method=method), timeout=4
    )


try:
    for attempt in range(50):
        if process.poll() is not None:
            raise RuntimeError("go2rtc exited before becoming healthy")
        try:
            with request("/api") as response:
                json.load(response)  # Health/API JSON, not a release-label assertion on a patched build.
            break
        except urllib.error.URLError:
            if attempt == 49:
                raise
            time.sleep(0.1)

    subprocess.run(
        ["wget", "-q", "-T", "4", "-O", "/dev/null", base + "/api"], check=True
    )
    with request("/api/streams?src=z70") as response:
        assert response.status == 200
    before = open("/etc/go2rtc.yaml", "rb").read()
    with request(
        "/api/streams?name=z70&src=rtsp%3A%2F%2F127.0.0.1%3A1%2Fsynthetic",
        "PATCH",
    ) as response:
        assert response.status == 200
    assert open("/etc/go2rtc.yaml", "rb").read() == before

    for path in (
        "/", "/api/config", "/api/log", "/api/exit", "/api/restart",
        "/api/roborock", "/api/ws", "/api/exec",
    ):
        try:
            request(path).close()
        except urllib.error.HTTPError as error:
            assert error.code == 404, (path, error.code)
        else:
            raise AssertionError("Unexpected enabled endpoint: " + path)

    for path, method in (
        ("/api/webrtc?src=missing", "POST"),
        ("/api/frame.jpeg?src=missing", "GET"),
    ):
        try:
            request(path, method).close()
        except urllib.error.HTTPError as error:
            assert error.read() != b"404 page not found\n", path
        else:
            raise AssertionError("Missing stream unexpectedly succeeded")

    subprocess.run([
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "lavfi",
        "-i", "color=c=black:s=1600x1200", "-frames:v", "1",
        "-f", "image2", "-update", "1", "/tmp/validation.jpeg",
    ], check=True)
    dimensions = subprocess.check_output([
        "ffprobe", "-v", "error", "-select_streams", "v:0",
        "-show_entries", "stream=width,height", "-of", "csv=p=0",
        "/tmp/validation.jpeg",
    ], text=True).strip()
    assert dimensions == "1600,1200", dimensions
    assert os.getuid() == 1000
    print("Pinned-source go2rtc: nonroot/read-only, health, PATCH, API allowlist, FFmpeg JPEG 1600x1200 OK")
finally:
    process.terminate()
    process.wait(timeout=5)
