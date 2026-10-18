from setuptools import setup
setup(name='z70_web_gateway', version='0.1.0', packages=['z70_web_gateway'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/z70_web_gateway']),
                  ('share/z70_web_gateway', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['gateway = z70_web_gateway.gateway:main']})
