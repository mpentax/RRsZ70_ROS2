"""Resource paths and configuration loaders for z70_manipulation."""
from pathlib import Path
import yaml
from z70_description.resources import config_path, package_root

_FALLBACK_ROOT = Path(__file__).resolve().parent.parent


def front_calibration_path():
    return config_path('front_camera.json')


def description_config(name):
    return yaml.safe_load(config_path(name).read_text())


def colors_path():
    path = package_root('z70_manipulation', _FALLBACK_ROOT) / 'config/colors.yaml'
    if not path.is_file():
        raise FileNotFoundError(path)
    return path
