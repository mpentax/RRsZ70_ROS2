from setuptools import setup

setup(name='z70_driver', version='0.1.0', packages=['z70_driver'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/z70_driver']),
                  ('share/z70_driver', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['driver = z70_driver.driver:main']})
