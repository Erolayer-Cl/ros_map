import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'uav_wall_mapping'

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
    description='Mapa 2D/3D, detección y clasificación de paredes (planas/irregulares) y sectores de obstáculos.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'mapper_node = uav_wall_mapping.mapper_node:main',
        ],
    },
)
