"""Verified Docker attach handshake; retain only closed error categories."""
import socket, os, time, json, re
from pathlib import Path
root = Path(__file__).parent
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.settimeout(10)
paths = [Path.home()/'.docker/run/docker.sock', Path('/var/run/docker.sock')]
path = next(p for p in paths if p.exists())
sock.connect(str(path))
sock.sendall(b'POST /v1.47/containers/roboclaw-go2rtc-1/attach?stream=1&stdout=1&stderr=1 HTTP/1.1\r\nHost: docker\r\nConnection: Upgrade\r\nUpgrade: tcp\r\nContent-Length: 0\r\n\r\n')
data = b''
while b'\r\n\r\n' not in data:
    data += sock.recv(4096)
    if len(data) > 32768: raise RuntimeError('Oversize attach handshake')
headers, data = data.split(b'\r\n\r\n', 1)
status = int(headers.split(b'\r\n', 1)[0].split()[1])
if status not in [101, 200]:
    sock.close()
    raise RuntimeError('Docker attach refused HTTP '+str(status))
ready = {'attached': True, 'httpStatus': status, 'startedAtNs': time.time_ns()}
(root/'camera-observer-ready.json').write_text(json.dumps(ready)+'\n')
print(json.dumps(ready), flush=True)
phrases = ['wrong login', 'wrong pin', "can't get device sdp", "can't connect", 'camera not ready', 'permission denied', 'startup timeout', 'exec/pipe', 'connection refused', 'timeout', 'invalid', 'eof', 'ffmpeg']
counts = {p:0 for p in phrases}
events=[]; linebuf=b''; end=time.monotonic()+80
sock.settimeout(1)
try:
    while time.monotonic() < end:
        while len(data)>=8:
            length=int.from_bytes(data[4:8], 'big')
            if length > 1024*1024: raise RuntimeError('Oversize attach frame')
            if len(data)<8+length: break
            chunk=data[8:8+length]; data=data[8+length:]
            linebuf+=chunk
            if len(linebuf)>65536: linebuf=linebuf[-65536:]
            while b'\n' in linebuf:
                line,linebuf=linebuf.split(b'\n',1)
                text=line.decode('utf8','replace').lower()
                found=[p for p in phrases if p in text]
                for p in found: counts[p]+=1
                caller=re.search(r'(client|producer|mjpeg|exec|jpeg|command|webrtc)\.go:(\d+)', text)
                severity='error' if 'err' in text else 'other'
                if found or caller:
                    events.append({'receivedAtNs':time.time_ns(),'categories':found,'severity':severity,
                        **({'callerKind':caller[1],'callerLine':int(caller[2])} if caller else {})})
                    events=events[-40:]
        try:
            chunk=sock.recv(65536)
            if not chunk: break
            data+=chunk
        except socket.timeout: pass
finally:
    sock.close()
    out={**ready,'finishedAtNs':time.time_ns(),'categories':counts,'events':events}
    (root/'camera-observer-result.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps(out),flush=True)
