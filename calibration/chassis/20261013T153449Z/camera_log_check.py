"""Bounded Docker log classifier. Never retains raw credential-bearing logs."""
import subprocess, selectors, time, json, os
from pathlib import Path
p = subprocess.Popen(['docker', 'attach', '--no-stdin', '--sig-proxy=false', 'roboclaw-go2rtc-1'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
s = selectors.DefaultSelector(); s.register(p.stdout, selectors.EVENT_READ)
counts = {k: 0 for k in ['wrong login', 'wrong pin', "can't get device sdp", "can't get device ice", 'timeout', 'connection refused', 'ffmpeg', 'error', 'eof', 'exec', 'streams', 'codecs', 'read tcp', 'write tcp', 'context canceled', 'signal', 'exit status', 'not found', 'broken pipe', 'closed pipe', 'no route', 'unreachable', 'dial', 'webrtc', 'roborock', 'other', 'daemon', 'logging', 'attach', 'permission', 'operation', 'panic', 'invalid', 'exit', 'jpeg', 'mp4', 'video', 'codec', 'http', 'input', 'output', 'sdp', 'reader', 'producer', 'consumer', 'reset', 'stopped', 'device', 'disconnected', "can't", 'get', 'first', 'frame', 'busy', 'bind', 'address', 'network', 'refused', 'eof', 'dts', 'pts', 'packet', 'decod', 'rtp', 'ice', 'mqtt', 'start', 'unsupported', 'application', 'not', 'connect', 'stream']}
end = time.monotonic() + 75
try:
    while time.monotonic() < end:
        for key, _ in s.select(timeout=min(1, max(0, end-time.monotonic()))):
            line = os.read(key.fd, 65536)
            if not line: break
            text = line.decode('utf8', 'replace').lower()
            matched = False
            for phrase in counts:
                if phrase != 'other' and phrase.lower() in text:
                    counts[phrase] += 1; matched = True
            if not matched: counts['other'] += 1
finally:
    p.kill(); p.wait(); s.close()
Path(__file__).with_name('camera-log-categories.json').write_text(json.dumps(counts, indent=2)+'\n')
print(json.dumps(counts))
