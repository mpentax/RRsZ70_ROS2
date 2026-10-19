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
