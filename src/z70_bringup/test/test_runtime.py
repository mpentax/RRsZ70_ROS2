"""Simulation launch isolation, without consulting the live robot or its files."""
import importlib.util
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import ExecuteProcess, LogInfo
import pytest
import requests
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter_client import AsyncParameterClient

from z70_bringup.runtime import configure_runtime, simulation_mcp_origin
from z70_web_gateway.gateway import read_token


def configured(tmp_path, **overrides):
    context = LaunchContext()
    context.launch_configurations.update({
        'sim': 'true', 'sidecars': 'true', 'origin': '', 'token_file': '',
        'web_gateway_port': '', 'foxglove_port': '', 'sim_domain_id': '98',
        'sim_state_dir': str(tmp_path / 'simulation'), **overrides,
    })
    for action in configure_runtime(context, repo=tmp_path / 'live-repo'):
        action.execute(context)
    return context


def test_simulation_defaults_are_fresh_and_separate_from_live_state(tmp_path):
    context = configured(tmp_path)
    config = context.launch_configurations
    assert config['origin'] == 'http://127.0.0.1:57110'
    assert config['sim_port'] == '57110'
    assert config['web_gateway_port'] == '57080'
    assert config['mcp_origin'] == 'http://127.0.0.1:57080'
    assert config['foxglove_port'] == '8865'
    assert context.environment['ROS_DOMAIN_ID'] == '98'
    assert context.environment['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
    state = tmp_path / 'simulation'
    assert context.environment['XDG_DATA_HOME'] == str(state / 'data')
    assert context.environment['ROS_LOG_DIR'] == str(state / 'ros-log')
    assert config['runtime_repo'] == str(state)
    token = Path(config['token_file'])
    assert token.parent == state and token.stat().st_mode & 0o777 == 0o600
    assert read_token(str(token))
    assert not (tmp_path / 'live-repo').exists()
    with pytest.raises(FileExistsError):
        configured(tmp_path)


def test_live_launch_defaults_remain_compatible(tmp_path):
    context = configured(tmp_path, sim='false')
    config = context.launch_configurations
    assert config['origin'] == 'http://127.0.0.1:57010'
    assert config['mcp_origin'] == 'http://127.0.0.1:57010'
    assert config['token_file'] == str(tmp_path / 'live-repo/.local/service-token')
    assert config['web_gateway_port'] == '57020' and config['foxglove_port'] == '8765'
    assert config['runtime_repo'] == str(tmp_path / 'live-repo')
    assert not (tmp_path / 'simulation').exists()


@pytest.mark.parametrize('overrides', [
    {'origin': 'http://192.0.2.1:57110'}, {'origin': 'http://127.0.0.1:57010'},
    {'origin': 'http://127.0.0.1:57020'}, {'origin': 'http://127.0.0.1:57110/api'},
    {'origin': 'http://user@127.0.0.1:57110'}, {'origin': 'http://127.0.0.1'},
    {'web_gateway_port': '57020'}, {'web_gateway_port': '57110'},
    {'web_gateway_port': '57120'}, {'foxglove_port': '8765'},
    {'foxglove_port': '57080'}, {'sim_domain_id': '0'},
    {'token_file': '/do-not-read/live-token'},
])
def test_simulation_rejects_live_or_ambiguous_configuration_before_creating_state(tmp_path, overrides):
    with pytest.raises(ValueError):
        configured(tmp_path, **overrides)
    assert not (tmp_path / 'simulation').exists()


@pytest.mark.parametrize('sim', ['true', 'True', '1', ' true '])
def test_simulation_disables_hardware_sidecars_even_when_requested(tmp_path, sim):
    path = Path(get_package_share_directory('z70_bringup')) / 'launch/bringup.launch.py'
    spec = importlib.util.spec_from_file_location('offline_bringup', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    context = configured(tmp_path, sim=sim)
    sidecars = [action for action in module.generate_launch_description().entities
                if type(action) is ExecuteProcess and action.condition is not None]
    # Simulator is enabled; both hardware transports are disabled.
    assert [action.condition.evaluate(context) for action in sidecars] == [True, False, False]


def test_simulation_advertises_gateway_as_isolated_mcp_origin(tmp_path):
    context = LaunchContext()
    context.launch_configurations.update({
        'sim': 'true', 'sidecars': 'true', 'origin': '', 'token_file': '',
        'web_gateway_port': '', 'foxglove_port': '', 'sim_domain_id': '98',
        'sim_state_dir': str(tmp_path / 'simulation'),
    })
    actions = configure_runtime(context, repo=tmp_path / 'live-repo')
    logs = [action for action in actions if isinstance(action, LogInfo)]
    assert len(logs) == 1
    message = ''.join(context.perform_substitution(part) for part in logs[0].msg)
    assert 'transport: http://127.0.0.1:57110' in message
    assert 'mcp: http://127.0.0.1:57080' in message
    assert 'service-token' not in message
    for action in actions:
        action.execute(context)
    assert context.launch_configurations['mcp_origin'] == 'http://127.0.0.1:57080'
    assert context.launch_configurations['origin'] == 'http://127.0.0.1:57110'


def test_custom_simulation_ports_keep_mcp_on_the_gateway_not_the_transport(tmp_path):
    context = configured(tmp_path, origin='http://127.0.0.1:57050', web_gateway_port='57090')
    config = context.launch_configurations
    assert config['origin'] == 'http://127.0.0.1:57050'
    assert config['mcp_origin'] == 'http://127.0.0.1:57090'
    assert config['web_gateway_port'] == '57090'
    assert config['sim_port'] == '57050'


@pytest.mark.parametrize('port', [56999, 57010, 57020, 57110, 57101])
def test_simulation_mcp_origin_rejects_live_and_transport_ports(port):
    with pytest.raises(ValueError):
        simulation_mcp_origin(port)


def test_simulation_mcp_origin_accepts_the_isolated_gateway_band():
    assert simulation_mcp_origin(57080) == 'http://127.0.0.1:57080'
    assert simulation_mcp_origin(57100) == 'http://127.0.0.1:57100'


def test_standard_simulation_launch_routes_driver_and_gateway_to_owned_endpoints(tmp_path):
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0)); sim_port = sock.getsockname()[1]
    gateway_port = None
    for candidate in range(57070, 57100):
        with socket.socket() as sock:
            try: sock.bind(('127.0.0.1', candidate))
            except OSError: continue
            gateway_port = candidate; break
    assert gateway_port is not None
    state = tmp_path / 'launch-state'
    log = tmp_path / 'launch.log'
    env = dict(os.environ, ROS_DOMAIN_ID='98', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               ROS_LOG_DIR=str(tmp_path / 'launcher-logs'))
    with log.open('w') as output:
        process = subprocess.Popen([
            'ros2', 'launch', 'z70_bringup', 'bringup.launch.py',
            'sim:=true', 'sidecars:=true', 'web_gateway:=true', 'manipulation:=true',
            'depth:=false', 'sim_domain_id:=98', f'sim_state_dir:={state}',
            f'origin:=http://127.0.0.1:{sim_port}', f'web_gateway_port:={gateway_port}',
            'foxglove_port:=0',
        ], cwd=tmp_path, env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 35
            while time.monotonic() < deadline:
                assert process.poll() is None, log.read_text()
                try:
                    response = requests.get(f'http://127.0.0.1:{sim_port}/api/ros/state', timeout=2)
                    snapshot = response.json() if response.status_code == 200 else {}
                    if (snapshot.get('available') and snapshot.get('state')
                            and snapshot['state']['evidence']['valid'] and snapshot.get('camera')):
                        break
                except requests.RequestException:
                    pass
                time.sleep(.1)
            else:
                raise AssertionError(log.read_text())
            token = read_token(str(state / 'service-token'))
            direct = requests.get(f'http://127.0.0.1:{gateway_port}/api/ros/state',
                                  headers={'Authorization': 'Bearer ' + token}, timeout=3)
            assert direct.status_code == 200 and direct.json()['available']
            assert requests.get(f'http://127.0.0.1:{sim_port}/healthz', timeout=2).json()['simulator'] == 'mujoco'
            assert requests.get(f'http://127.0.0.1:{gateway_port}/api/ros/state', timeout=2).status_code == 401
            assert 'ROS domain: 98' in log.read_text()
            assert 'go2rtc' not in log.read_text() and 'service-main.ts' not in log.read_text()
            # Parameter services respond only once node construction has finished.
            context = Context(); rclpy.init(context=context, domain_id=98)
            probe = rclpy.create_node('isolated_simulation_parameters', context=context)
            executor = SingleThreadedExecutor(context=context); executor.add_node(probe)
            try:
                for node_name, names, expected in (
                    ('z70_driver', ['origin', 'token_file', 'arm_estimate_path'],
                     [f'http://127.0.0.1:{sim_port}', str(state / 'service-token'),
                      str(state / 'data/RRsZ70_ROS2/arm-estimate.json')]),
                    ('z70_pick_server', ['repo', 'patrol_ledger_directory'],
                     [str(state), str(state / '.local/patrol')]),
                ):
                    client = AsyncParameterClient(probe, node_name)
                    assert client.wait_for_services(timeout_sec=10), log.read_text()
                    future = client.get_parameters(names)
                    executor.spin_until_future_complete(future, timeout_sec=10)
                    assert future.done(), log.read_text()
                    assert [value.string_value for value in future.result().values] == expected
            finally:
                executor.shutdown(); probe.destroy_node(); context.shutdown()
        finally:
            if process.poll() is None: process.send_signal(signal.SIGINT)
            try: process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=5)
                raise AssertionError('Simulation launch failed bounded shutdown: ' + log.read_text())
    assert process.returncode == 0, log.read_text()
    assert 'process has died' not in log.read_text(), log.read_text()
