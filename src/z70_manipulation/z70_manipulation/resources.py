"""Resource API for installed perception."""
from pathlib import Path
import yaml
from z70_description.resources import config_path


def front_calibration_path():
    """Installed front camera calibration resource."""
    path = config_path('front_camera.json')
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def description_config(name):
    return yaml.safe_load(config_path(name).read_text())


def colors_path():
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory('z70_manipulation')) / 'config/colors.yaml'
    if not path.is_file():
        raise FileNotFoundError(path)
    return path
