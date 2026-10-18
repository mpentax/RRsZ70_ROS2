"""Immutable package seeds and durable user-owned runtime calibration paths."""
import os
from pathlib import Path
from z70_description.resources import config_path


def front_calibration_path():
    return config_path('front_camera.json')


def runtime_calibration_path():
    """Compute, but do not create, the durable mutable state file."""
    base = Path(os.environ.get('XDG_DATA_HOME') or Path.home() / '.local/share').expanduser()
    if not base.is_absolute():
        # XDG requires absolute paths; relative values are treated as unset.
        base = Path.home() / '.local/share'
    return base / 'RRsZ70_ROS2/calibration/chassis/online.json'


def initialize_runtime_state(path, initial_state_file=''):
    """Explicit one-time archive import; never overwrite existing durable state.

    The source bytes, update time and IDs remain exact. No checkout is searched.
    Publishing historical state still marks it ESTIMATED, not a fresh measurement.
    """
    import json
    import tempfile
    target = Path(path)
    if target.exists() or not initial_state_file:
        return False
    content = Path(initial_state_file).read_bytes()
    value = json.loads(content)
    if not isinstance(value, dict) or value.get('version') != 1 or not isinstance(value.get('samples'), list):
        raise ValueError('initial state must be a version-1 calibration sample file')
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=target.parent, prefix=target.name + '.', suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(content); stream.flush(); os.fsync(stream.fileno())
        try:
            os.link(temporary, target)  # Atomic no-clobber commit, unlike replace.
        except FileExistsError:
            return False
        directory = os.open(target.parent, os.O_RDONLY)
        try: os.fsync(directory)
        finally: os.close(directory)
    finally:
        if temporary is not None: temporary.unlink(missing_ok=True)
    return True


def _package_root():
    try:
        from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    except ImportError:
        return Path(__file__).resolve().parent.parent
    try:
        return Path(get_package_share_directory('z70_odometry'))
    except PackageNotFoundError:
        return Path(__file__).resolve().parent.parent


def initial_camera_x_m():
    """Retained historical initial guess, not the model preview mount offset."""
    import yaml
    return float(yaml.safe_load((_package_root() / 'config/odometry.yaml').read_text())['camera_x_m'])


def seed_directory():
    path = _package_root() / 'config/chassis'
    if not path.is_dir():
        raise FileNotFoundError(path)
    return path
