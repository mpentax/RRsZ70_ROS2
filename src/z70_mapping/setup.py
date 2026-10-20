from setuptools import setup
setup(name='z70_mapping', version='0.1.0', packages=['z70_mapping'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/z70_mapping']),
                  ('share/z70_mapping', ['package.xml', 'README.md'])],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts':['vendor_map = z70_mapping.node:main',
                                       'depth_cloud = z70_mapping.node:depth_main']})
