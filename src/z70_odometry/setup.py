from setuptools import setup
from glob import glob
from pathlib import Path

package_name = "z70_odometry"
setup(
    name=package_name, version="0.1.0", packages=[package_name],
    data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                ("share/" + package_name, ["package.xml"]),
                ("share/" + package_name + '/config', glob('config/*.yaml'))] +
               [(str(Path('share') / package_name / Path(path).parent), [path])
                for path in sorted(glob('config/chassis/*/fit.json'))],
    install_requires=["setuptools", "numpy", "PyYAML"], zip_safe=True,
    entry_points={"console_scripts": ["floor_odometry = z70_odometry.node:main",
                                       "chassis_calibrator = z70_odometry.node:calibrator_main",
                                       "floor_mosaic = z70_odometry.node:mosaic_main"]},
)
