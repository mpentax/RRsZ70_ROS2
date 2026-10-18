"""Installed, versioned model/calibration resources (never a checkout search)."""
from pathlib import Path


def config_path(name):
    """Resolve one config basename via ament, with package-local source use for tests.

    A missing installed file is an error, not permission to search a checkout or
    use a different camera model. The source fallback only applies before this
    package is registered with ament; it never walks repository ancestors.
    """
    if Path(name).name != name:
        raise ValueError('configuration name must be a basename')
    try:
        from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    except ImportError:
        root = Path(__file__).resolve().parent.parent
    else:
        try:
            root = Path(get_package_share_directory('z70_description'))
        except PackageNotFoundError:
            root = Path(__file__).resolve().parent.parent
    path = root / 'config' / name
    if not path.is_file():
        raise FileNotFoundError(path)
    return path
