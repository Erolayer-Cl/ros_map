import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'uav_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ignacio',
    maintainer_email='ibanezignacio2005@gmail.com',
    description='Misión PX4 offboard: waypoints con evasión de paredes y misión de mapeo por fronteras.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'mission_node = uav_mission.mission_node:main',
        ],
    },
)
