"""Resolve installed package and calibration resource paths."""
from pathlib import Path


def package_root(package_name: str, fallback: Path) -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    except ImportError:
        return fallback
    try:
        return Path(get_package_share_directory(package_name))
    except PackageNotFoundError:
        return fallback


def config_path(name: str) -> Path:
    """Resolve a configuration basename under z70_description/config."""
    if Path(name).name != name:
        raise ValueError('configuration name must be a basename')
    path = package_root('z70_description', Path(__file__).resolve().parent.parent) / 'config' / name
    if not path.is_file():
        raise FileNotFoundError(path)
    return path
