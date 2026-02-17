import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'farm_sim'

data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yeml]'))),
    (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
    (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
]

def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        if os.path.exists(directory):
            for (path, directories, filenames) in os.walk(directory):
                for filename in filenames:
                    file_path = os.path.join(path, filename)
                    install_path = os.path.join('share', package_name, path)
                    if install_path in paths_dict:
                        paths_dict[install_path].append(file_path)
                    else:
                        paths_dict[install_path] = [file_path]
    
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

package_files(data_files, ['models'])

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arihant',
    maintainer_email='01fe22bei048@kletech.ac.in',
    description='Farm simulation package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'navigation_node = farm_sim.navigation_node:main',
        ],
    },
)
