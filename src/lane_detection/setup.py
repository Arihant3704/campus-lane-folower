from setuptools import setup

package_name = 'lane_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arihant',
    maintainer_email='arihant@todo.todo',
    description='Lane detection node using OpenCV Canny and Hough Transform',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lane_detector_node = lane_detection.lane_detector_node:main',
            'lane_keeper = lane_detection.lane_keeper:main'
        ],
    },
)
