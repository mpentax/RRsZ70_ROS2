"""Resource API for installed perception; explicit archives are offline overrides."""
from pathlib import Path
import yaml
from z70_description.resources import config_path


def front_calibration_path(repo=None):
    """The default never depends on cwd, Z70_REPO or a repository ancestor."""
    path = Path(repo) / 'calibration/front-camera/front_camera.json' if repo else config_path('front_camera.json')
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def description_config(name, repo=None):
    # Optional repo is only an explicit archive/test override. Never import code
    # from the archive and never silently fall back when an override is missing.
    path = Path(repo) / 'src/z70_description/config' / name if repo else config_path(name)
    return yaml.safe_load(path.read_text())


def colors_path():
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory('z70_manipulation')) / 'config/colors.yaml'
    if not path.is_file():
        raise FileNotFoundError(path)
    return path
