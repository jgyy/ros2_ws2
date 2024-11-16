from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f'{package_name}.nodes'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jegoh',
    maintainer_email='jeffrey.goh@sit.singaporetech.edu.sg',
    description='Test Implementation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_state_publisher = robot_control.nodes.robot_state_publisher:main',
            'robot_monitor = robot_control.nodes.robot_monitor:main',
            'state_controller = robot_control.nodes.state_controller:main',
            'robot_mover = robot_control.nodes.robot_mover:main',
            'move_commander = robot_control.nodes.move_commander:main'
        ],
    },
)
