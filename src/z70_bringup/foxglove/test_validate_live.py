"""Offline validator regressions: never start ROS, vendor services or cameras."""
import os
import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

import validate_live as validator


def test_prepare_launch_isolates_all_user_write_locations(tmp_path, monkeypatch):
    protected = tmp_path / 'protected'
    protected.mkdir()
    state = protected / 'RRsZ70_ROS2'
    (state / 'calibration/chassis').mkdir(parents=True)
    estimate = state / 'arm-estimate.json'
    calibration = state / 'calibration/chassis/online.json'
    estimate.write_bytes(b'protected arm estimate\x00')
    calibration.write_bytes(b'protected chassis calibration\x00')
    before = {p: p.read_bytes() for p in (estimate, calibration)}
    inherited = dict(os.environ, HOME=str(protected), XDG_DATA_HOME=str(protected),
                     XDG_CACHE_HOME=str(protected), XDG_CONFIG_HOME=str(protected),
                     XDG_STATE_HOME=str(protected), ROS_HOME=str(protected),
                     ROS_LOG_DIR=str(protected), TMPDIR=str(protected),
                     Z70_REPO=str(protected), ROS_DOMAIN_ID='0')
    original = inherited.copy()
    monkeypatch.chdir(tmp_path)

    args, env = validator.prepare_launch(Path('disposable'), inherited)

    scratch = (tmp_path / 'disposable').resolve()
    for key in ('HOME', 'XDG_DATA_HOME', 'XDG_CACHE_HOME', 'XDG_CONFIG_HOME',
                'XDG_STATE_HOME', 'ROS_HOME', 'ROS_LOG_DIR', 'TMPDIR'):
        path = Path(env[key])
        assert path.is_absolute()
        assert path.is_relative_to(scratch)
        assert path.is_dir()
    assert env['ROS_DOMAIN_ID'] == '96'
    assert env['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
    assert 'Z70_REPO' not in env
    assert inherited == original
    assert {p: p.read_bytes() for p in before} == before


def test_launch_uses_installed_calibrations_and_explicit_original_depth_python(tmp_path):
    home = tmp_path / 'original home'
    args, env = validator.prepare_launch(tmp_path / 'scratch', {'HOME': str(home)})

    assert args[:3] == ['bash', '-c',
        'source install/setup.bash && exec ros2 launch z70_bringup bringup.launch.py "$@"']
    assert args[3] == '--'
    launch_args = dict(arg.split(':=', 1) for arg in args[4:])
    assert launch_args == {
        'foxglove_port': '8876',
        'token_file': str(validator.ROOT / '.local/service-token'),
        'depth_python': str(home / '.venvs/roboclaw-depth/bin/python'),
    }
    assert 'Z70_REPO' not in env


@pytest.mark.parametrize('inherited_xdg', [None, '', 'relative', 'absolute'])
def test_main_passes_private_state_to_child_without_touching_protected_bytes(
        tmp_path, monkeypatch, inherited_xdg):
    protected = tmp_path / 'protected'
    home = protected / 'home'
    xdg = protected / 'data'
    protected_files = []
    for base in (xdg, home / '.local/share'):
        for relative in ('arm-estimate.json', 'calibration/chassis/online.json'):
            path = base / 'RRsZ70_ROS2' / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b'untouched disposable state\x00')
            protected_files.append(path)
    before = {p: p.read_bytes() for p in protected_files}
    monkeypatch.setenv('HOME', str(home))
    if inherited_xdg is None:
        monkeypatch.delenv('XDG_DATA_HOME', raising=False)
    else:
        monkeypatch.setenv('XDG_DATA_HOME', str(xdg) if inherited_xdg == 'absolute' else inherited_xdg)
    monkeypatch.setenv('TMPDIR', str(tmp_path))
    monkeypatch.setattr(validator, 'ROOT', tmp_path)
    monkeypatch.setattr(validator, 'HERE', tmp_path)
    (tmp_path / 'z70-dashboard.json').write_text('{}')
    calls = []
    real_popen = subprocess.Popen

    def launch(args, **kwargs):
        env = kwargs['env']
        scratch = Path(env['XDG_DATA_HOME']).parent
        assert kwargs['cwd'] == tmp_path
        assert kwargs['start_new_session'] is True
        assert kwargs['stderr'] == subprocess.STDOUT
        assert 'Z70_REPO' not in env
        assert args == validator.prepare_launch(scratch, dict(os.environ))[0]
        # A real, harmless Python subprocess consumes the exact child environment.
        # Use the production odometry path resolver; no node or service is started.
        probe = '''
import json, os
from pathlib import Path
from z70_odometry.resources import runtime_calibration_path
arm = Path(os.environ['XDG_DATA_HOME']) / 'RRsZ70_ROS2/arm-estimate.json'
calibration = runtime_calibration_path()
for path in (arm, calibration):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b'private validator state')
print(json.dumps([str(arm), str(calibration)]))
'''
        with real_popen([sys.executable, '-c', probe], env=env, cwd=scratch,
                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True) as child:
            output, error = child.communicate(timeout=15)
            assert child.returncode == 0, error
        paths = [Path(p) for p in json.loads(output)]
        assert all(p.is_absolute() and p.is_relative_to(scratch) for p in paths)
        assert all(p.read_bytes() == b'private validator state' for p in paths)
        calls.append(scratch)
        return SimpleNamespace(pid=987654, poll=lambda: 0, returncode=0, wait=lambda timeout: 0)

    def absent_group(pid, sig):
        assert pid == 987654
        raise ProcessLookupError

    monkeypatch.setattr(validator.os, 'killpg', absent_group)
    monkeypatch.setattr(validator.subprocess, 'Popen', launch)
    monkeypatch.setattr(validator.rclpy, 'init', lambda: None)
    monkeypatch.setattr(validator.rclpy, 'create_node', lambda _: SimpleNamespace(destroy_node=lambda: None))
    monkeypatch.setattr(validator.rclpy, 'try_shutdown', lambda: None)
    class Socket:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def bind(self, address): assert address == ('127.0.0.1', 8876)
    monkeypatch.setattr(validator.socket, 'socket', Socket)
    with pytest.raises(SystemExit, match='1'):
        validator.main()  # Fake exited bringup has no topic messages.
    assert len(calls) == 1
    assert calls[0].is_dir()  # Failed validation conservatively retains diagnostics.
    assert {p: p.read_bytes() for p in before} == before


def test_only_real_conditional_sources_are_optional():
    assert set(validator.OPTIONAL_TOPICS) == {
        '/camera/front/depth/points', '/camera/gripper/image_raw/compressed',
    }
    assert validator.referenced_topics({
        'map': '/map', 'field': '/z70/state.battery',
        'nested': ['/camera/gripper/image_raw/compressed', '/joint_states.position[0]'],
    }) == {'/map', '/z70/state', '/camera/gripper/image_raw/compressed', '/joint_states'}


@pytest.mark.parametrize('exit_code,exit_phase', [
    (7, 'observation'), (0, 'observation'), (7, 'observer_shutdown'),
    (0, 'observer_shutdown'), (None, 'cleanup'),
])
def test_complete_topics_cannot_hide_unexpected_bringup_exit(
        tmp_path, monkeypatch, exit_code, exit_phase):
    monkeypatch.setenv('TMPDIR', str(tmp_path))
    monkeypatch.setenv('ROS_DOMAIN_ID', '43')
    monkeypatch.delenv('ROS_LOG_DIR', raising=False)
    previous = dict(os.environ)
    monkeypatch.setattr(validator, 'HERE', tmp_path)
    (tmp_path / 'z70-dashboard.json').write_text('{}')
    callbacks = []
    signals = []
    captured = {}

    class Process:
        pid = 987654
        returncode: int | None = None
        def poll(self): return self.returncode
        def wait(self, timeout): return self.returncode
    proc = Process()

    class Node:
        def get_topic_names_and_types(self):
            return [(topic, ['std_msgs/msg/String']) for topic in ('/tf', '/tf_static', '/joint_states')]
        def get_publishers_info_by_topic(self, topic):
            return [SimpleNamespace(qos_profile=SimpleNamespace(durability=validator.DurabilityPolicy.VOLATILE))]
        def create_subscription(self, cls, topic, callback, qos):
            callbacks.append(callback)
            return object()
        def destroy_node(self):
            if exit_phase == 'observer_shutdown': proc.returncode = exit_code

    def spin_once(node, **kwargs):
        for callback in callbacks: callback(SimpleNamespace(data='seen'))
        if exit_phase == 'observation': proc.returncode = exit_code

    def killpg(pid, sig):
        assert pid == proc.pid
        if sig == 0 or proc.returncode is not None: raise ProcessLookupError
        signals.append(sig)
        proc.returncode = -int(sig)

    class Socket:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def bind(self, address): pass

    def launch(args, **kwargs):
        captured.update(kwargs['env'])
        assert os.environ['ROS_DOMAIN_ID'] == '96'
        assert os.environ['ROS_LOG_DIR'] == captured['ROS_LOG_DIR']
        return proc

    clock = iter([0, 0, 66, 66])  # One observation cycle, then its bounded deadline.
    monkeypatch.setattr(validator.time, 'monotonic', lambda: next(clock))
    monkeypatch.setattr(validator.socket, 'socket', Socket)
    monkeypatch.setattr(validator.subprocess, 'Popen', launch)
    monkeypatch.setattr(validator.os, 'killpg', killpg)
    monkeypatch.setattr(validator.rclpy, 'init', lambda: None)
    monkeypatch.setattr(validator.rclpy, 'create_node', lambda _: Node())
    monkeypatch.setattr(validator.rclpy, 'spin_once', spin_once)
    monkeypatch.setattr(validator.rclpy, 'try_shutdown', lambda: None)
    if exit_code is None:
        validator.main()
    else:
        with pytest.raises(SystemExit, match='1'):
            validator.main()
    scratch = Path(captured['XDG_DATA_HOME']).parent
    assert scratch.exists() is (exit_code is not None)
    assert dict(os.environ) == previous
    report = json.loads((tmp_path / 'live-validation.json').read_text())
    assert report['missing_undocumented'] == []
    assert all(topic['messages'] == 1 for topic in report['topics'].values())
    assert report['bringup_exit'] == exit_code
    assert report['cleanup_exit'] == (exit_code if exit_code is not None else -2)
    assert signals == ([] if exit_code is not None else [validator.signal.SIGINT])


def test_exited_launch_leader_does_not_leave_owned_child_running(tmp_path, monkeypatch):
    import signal
    import time

    monkeypatch.setattr(validator, 'HERE', tmp_path)
    (tmp_path / 'z70-dashboard.json').write_text('{}')
    ready = tmp_path / 'child-ready'
    stopped = tmp_path / 'child-stopped'
    child_code = '''
import os, signal, sys, time
from pathlib import Path
signal.signal(signal.SIGINT, signal.SIG_IGN)
def stop(*_):
    Path(sys.argv[2]).write_text('terminated')
    sys.exit(0)
signal.signal(signal.SIGTERM, stop)
Path(sys.argv[1]).write_text(str(os.getpid()))
while True: time.sleep(.01)
'''
    leader_code = 'import subprocess, sys; subprocess.Popen(sys.argv[1:])'
    real_popen = subprocess.Popen
    owned = []
    unrelated = real_popen([sys.executable, '-c', 'import time; time.sleep(90)'], start_new_session=True)

    def launch(args, **kwargs):
        proc = real_popen([sys.executable, '-c', leader_code, sys.executable, '-c',
                           child_code, str(ready), str(stopped)], **kwargs)
        owned.append(proc)
        return proc

    def fail_after_leader_exits():
        owned[0].wait(timeout=5)
        deadline = time.monotonic() + 5
        while not ready.exists() and time.monotonic() < deadline:
            time.sleep(.01)
        assert ready.exists()
        raise RuntimeError('observer failed after leader exited')

    class Socket:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def bind(self, address): pass

    monkeypatch.setattr(validator.socket, 'socket', Socket)
    monkeypatch.setattr(validator.subprocess, 'Popen', launch)
    monkeypatch.setattr(validator.rclpy, 'init', fail_after_leader_exits)
    monkeypatch.setattr(validator.rclpy, 'try_shutdown', lambda: None)
    try:
        with pytest.raises(RuntimeError, match='observer failed after leader exited'):
            validator.validate([], dict(os.environ))
        assert stopped.exists(), 'owned descendant survived cleanup after its leader exited'
        child_pid = int(ready.read_text())
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            try: os.kill(child_pid, 0)
            except ProcessLookupError: break
            time.sleep(.01)
        else: pytest.fail('owned descendant was not reaped')
        assert owned[0].returncode == 0
        assert unrelated.poll() is None
    finally:
        for proc in owned:
            try: os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError: pass
            proc.wait(timeout=5)
        unrelated.terminate()
        unrelated.wait(timeout=5)


@pytest.mark.parametrize('cleanup_error', [KeyboardInterrupt, OSError])
def test_exceptional_cleanup_preserves_scratch_while_owned_child_is_alive(
        tmp_path, monkeypatch, cleanup_error):
    import signal
    import time

    monkeypatch.setenv('TMPDIR', str(tmp_path))
    monkeypatch.setattr(validator, 'HERE', tmp_path)
    monkeypatch.setattr(validator, 'ROOT', tmp_path)
    (tmp_path / 'z70-dashboard.json').write_text('{}')
    previous = dict(os.environ)
    real_popen = subprocess.Popen
    real_killpg = os.killpg
    real_sleep = time.sleep
    owned = []
    signals = []
    captured = {}
    unrelated = real_popen([sys.executable, '-c', 'import time; time.sleep(90)'],
                           start_new_session=True)
    child_code = '''
import signal, sys, time
from pathlib import Path
signal.signal(signal.SIGINT, signal.SIG_IGN)
signal.signal(signal.SIGTERM, signal.SIG_IGN)
Path(sys.argv[1]).write_text('ready')
while True: time.sleep(.01)
'''

    def launch(args, **kwargs):
        captured.update(kwargs['env'])
        ready = Path(captured['HOME']) / 'ready'
        proc = real_popen([sys.executable, '-c', child_code, str(ready)], **kwargs)
        owned.append(proc)
        deadline = time.monotonic() + 5
        while not ready.exists() and time.monotonic() < deadline:
            real_sleep(.01)
        assert ready.exists()
        return proc

    def fail_observation():
        raise RuntimeError('disposable observer failure')

    def interrupted_sleep(_):
        raise cleanup_error('injected during cleanup grace period')

    def killpg(pid, sig):
        assert pid == owned[0].pid, 'must never signal the unrelated session'
        signals.append(sig)
        return real_killpg(pid, sig)

    class Socket:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def bind(self, address): pass

    monkeypatch.setattr(validator.socket, 'socket', Socket)
    monkeypatch.setattr(validator.subprocess, 'Popen', launch)
    monkeypatch.setattr(validator.rclpy, 'init', fail_observation)
    monkeypatch.setattr(validator.rclpy, 'try_shutdown', lambda: None)
    monkeypatch.setattr(validator.time, 'sleep', interrupted_sleep)
    monkeypatch.setattr(validator.os, 'killpg', killpg)
    try:
        with pytest.raises(cleanup_error, match='injected during cleanup grace period'):
            validator.main()
        assert owned[0].poll() is None, 'shutdown is deliberately still unconfirmed'
        assert unrelated.poll() is None
        assert signals == [signal.SIGINT, 0]
        assert Path(captured['XDG_DATA_HOME']).parent.is_dir()
        assert (Path(captured['HOME']) / 'ready').read_text() == 'ready'
        assert not (tmp_path / 'live-validation.json').exists()
        assert dict(os.environ) == previous
    finally:
        monkeypatch.setattr(validator.time, 'sleep', real_sleep)
        for proc in owned:
            try: real_killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError: pass
            proc.wait(timeout=5)
        unrelated.terminate()
        unrelated.wait(timeout=5)


@pytest.mark.parametrize('shutdown_fails', [False, True])
@pytest.mark.parametrize('group_survives', [False, True])
def test_observer_failure_stops_only_owned_process_and_retains_scratch(
        tmp_path, monkeypatch, shutdown_fails, group_survives):
    monkeypatch.setenv('TMPDIR', str(tmp_path))
    monkeypatch.setattr(validator, 'HERE', tmp_path)
    (tmp_path / 'z70-dashboard.json').write_text('{}')
    captured = {}
    signals = []
    waits = []
    previous = dict(os.environ)

    class Process:
        pid = 987654
        returncode: int | None = None
        def poll(self):
            if signals: self.returncode = 0  # Leader exits on INT; its child ignores INT/TERM.
            return self.returncode
        def wait(self, timeout=None):
            waits.append(timeout)
            return self.returncode

    def launch(args, **kwargs):
        captured.update(kwargs['env'])
        assert kwargs['start_new_session'] is True
        return Process()

    def fail():
        raise RuntimeError('observer failed')

    class Socket:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def bind(self, address): pass

    monkeypatch.setattr(validator.socket, 'socket', Socket)
    monkeypatch.setattr(validator.subprocess, 'Popen', launch)
    monkeypatch.setattr(validator.rclpy, 'init', fail)
    monkeypatch.setattr(validator.rclpy, 'try_shutdown', fail if shutdown_fails else lambda: None)
    def killpg(pid, sig):
        assert pid == 987654
        if sig: signals.append((pid, sig))
        elif not group_survives and signals[-1][1] == validator.signal.SIGKILL: raise ProcessLookupError

    clock = iter(range(100))
    monkeypatch.setattr(validator.time, 'monotonic', lambda: next(clock))
    monkeypatch.setattr(validator.time, 'sleep', lambda _: None)
    monkeypatch.setattr(validator.os, 'killpg', killpg)
    error = TimeoutError if group_survives else RuntimeError
    message = 'survived bounded cleanup' if group_survives else 'observer failed'
    with pytest.raises(error, match=message):
        validator.main()
    assert signals == [(987654, validator.signal.SIGINT), (987654, validator.signal.SIGTERM),
                       (987654, validator.signal.SIGKILL)]
    assert waits == ([] if group_survives else [5])
    assert Path(captured['XDG_DATA_HOME']).parent.is_dir()
    assert dict(os.environ) == previous
