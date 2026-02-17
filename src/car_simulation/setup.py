from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'car_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arihant',
    maintainer_email='01fe22bei048@kletech.ac.in',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'car_simulation_node = car_simulation.car_simulation_node:main',
            'waypoint_publisher_node = car_simulation.waypoint_publisher_node:main',
            'color_detector_node = car_simulation.color_detector_node:main'
        ],
    },
)
