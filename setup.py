from setuptools import setup
import os
from glob import glob
package_name = 'px4_ros_offboard'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Atar Babgei',
    maintainer_email='atarbabgei@gmail.com',
    description='Offboard control package for PX4',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'altitude_hold_control = px4_ros_offboard.altitude_hold_control:main',
            'velocity_control = px4_ros_offboard.velocity_control:main',
            'manual_control = px4_ros_offboard.manual_control:main',
            'position_control = px4_ros_offboard.position_control:main',
        ],
    },
)
