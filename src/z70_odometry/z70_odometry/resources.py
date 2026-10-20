"""Package seeds and runtime calibration paths for z70_odometry."""
import os
from pathlib import Path
from z70_description.resources import config_path, package_root

_FALLBACK_ROOT = Path(__file__).resolve().parent.parent


def front_calibration_path():
    return config_path('front_camera.json')


def runtime_calibration_path():
    base = Path(os.environ.get('XDG_DATA_HOME') or Path.home() / '.local/share').expanduser()
    if not base.is_absolute():
        base = Path.home() / '.local/share'
    return base / 'RRsZ70_ROS2/calibration/chassis/online.json'


def _package_root():
    return package_root('z70_odometry', _FALLBACK_ROOT)


def initial_camera_x_m():
    import yaml
    return float(yaml.safe_load((_package_root() / 'config/odometry.yaml').read_text())['camera_x_m'])


def seed_directory():
    path = _package_root() / 'config/chassis'
    if not path.is_dir():
        raise FileNotFoundError(path)
    return path
