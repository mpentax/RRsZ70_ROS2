from glob import glob
from setuptools import setup

package_name = "z70_description"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/urdf", glob("urdf/*")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/meshes", glob("meshes/*.stl")),
        ("share/" + package_name + "/meshes/cad", glob("meshes/cad/*.stl")),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True,
    entry_points={"console_scripts": []},
)
