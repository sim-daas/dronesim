from setuptools import setup
import os
from glob import glob

package_name = 'skysentry'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],  # This package contains scripts, not python modules
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@example.com',
    description='SkySentry Drone Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # This makes the scripts in the 'scripts' folder executable
    scripts=['scripts/roskeypub.py', 'scripts/mavlinkros.py'],
)
