import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'uav_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'scenarios'), glob('scenarios/*.yaml')),
        (os.path.join('share', package_name, 'airsim'), glob('airsim/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ignacio',
    maintainer_email='ibanezignacio2005@gmail.com',
    description='Escenarios de prueba (Unreal y offline), PX4+lidar falso y simulación de lazo cerrado.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'fake_px4_world = uav_sim.fake_px4_world:main',
            'sim_offline = uav_sim.closed_loop:main',
            'spawn_scenario = uav_sim.spawn_unreal:main',
        ],
    },
)
