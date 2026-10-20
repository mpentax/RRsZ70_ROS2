from setuptools import setup
from glob import glob
setup(name='z70_manipulation',version='0.1.0',packages=['z70_manipulation'],
 data_files=[('share/ament_index/resource_index/packages',['resource/z70_manipulation']),
 ('share/z70_manipulation',['package.xml','README.md']),('share/z70_manipulation/config',glob('config/*'))],
 install_requires=['setuptools'],entry_points={'console_scripts':[
 'garment_perception = z70_manipulation.garment_node:main',
 'perception = z70_manipulation.node:perception_main','pick_server = z70_manipulation.node:server_main','pick = z70_manipulation.node:main','stack = z70_manipulation.node:stack_main']})
