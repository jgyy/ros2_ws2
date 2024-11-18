from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sensor_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jegoh',
    maintainer_email='jeffrey.goh@sit.singaporetech.edu.sg',
    description='Sensor system package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_simulator = sensor_system.sensor_simulator:main',
            'data_filter = sensor_system.data_filter:main',
            'data_analyzer = sensor_system.data_analyzer:main',
        ],
    },
)
