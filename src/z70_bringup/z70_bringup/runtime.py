"""Resolve live-compatible defaults or a fresh, loopback-only simulation runtime."""
from pathlib import Path
import os
import secrets
import tempfile
from urllib.parse import urlsplit

from launch.actions import LogInfo, SetEnvironmentVariable, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def configure_runtime(context, repo):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    sim = IfCondition(LaunchConfiguration('sim')).evaluate(context)
    origin = value('origin') or ('http://127.0.0.1:57110' if sim else 'http://127.0.0.1:57010')
    gateway_port = int(value('web_gateway_port') or ('57080' if sim else '57020'))
    foxglove_port = int(value('foxglove_port') or ('8865' if sim else '8765'))
    token_file = value('token_file')
    actions = []
    runtime_repo = str(repo)
    if sim:
        endpoint = urlsplit(origin)
        if (endpoint.scheme != 'http' or endpoint.hostname != '127.0.0.1'
                or endpoint.username or endpoint.password or endpoint.path not in ('', '/')
                or endpoint.query or endpoint.fragment or not endpoint.port
                or endpoint.port in (57010, 57020)):
            raise ValueError('Simulation origin must be an explicit 127.0.0.1 HTTP port, separate from live services')
        if not 57000 <= gateway_port <= 57100 or gateway_port in (57010, 57020, endpoint.port):
            raise ValueError('Simulation gateway requires a separate non-live loopback port')
        if not 0 <= foxglove_port <= 65535 or foxglove_port in (8765, 57010, 57020, endpoint.port, gateway_port):
            raise ValueError('Simulation Foxglove requires a separate non-live port (or 0)')
        domain = int(value('sim_domain_id'))
        if not 91 <= domain <= 99:
            raise ValueError('Simulation ROS domain must be in the offline range 91–99')
        if token_file:
            raise ValueError('Simulation creates its own token; do not supply token_file')
        requested_state = value('sim_state_dir')
        if requested_state:
            state = Path(requested_state).expanduser().resolve()
            state.mkdir(mode=0o700, parents=True, exist_ok=False)
        else:
            state = Path(tempfile.mkdtemp(prefix='z70-sim-'))
        token = state / 'service-token'
        with open(token, 'x', opener=lambda path, flags: os.open(path, flags, 0o600)) as stream:
            stream.write(secrets.token_urlsafe(32))
        token_file = str(token)
        runtime_repo = str(state)
        actions.extend([
            SetEnvironmentVariable('ROS_DOMAIN_ID', str(domain)),
            SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
            SetEnvironmentVariable('XDG_DATA_HOME', str(state / 'data')),
            SetEnvironmentVariable('ROS_LOG_DIR', str(state / 'ros-log')),
            LogInfo(msg=f'Simulation state: {state}; ROS domain: {domain}; transport: {origin}'),
        ])
    else:
        token_file = token_file or str(Path(repo) / '.local/service-token')
    actions.extend(SetLaunchConfiguration(name, str(setting)) for name, setting in {
        'sim': 'true' if sim else 'false',
        'origin': origin.rstrip('/'), 'token_file': token_file,
        'web_gateway_port': gateway_port, 'foxglove_port': foxglove_port,
        'runtime_repo': runtime_repo,
        'sim_port': str(urlsplit(origin).port) if sim else '57110',
    }.items())
    return actions
