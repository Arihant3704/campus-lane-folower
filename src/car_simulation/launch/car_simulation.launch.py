import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_car_simulation = get_package_share_directory('car_simulation')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': os.path.join(pkg_car_simulation, 'worlds', 'car_world.world')}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(os.path.join(pkg_car_simulation, 'urdf', 'car.urdf')).read()}]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'car'],
            output='screen'
        ),
        Node(
            package='car_simulation',
            executable='car_simulation_node',
            name='car_simulation_node',
            output='screen'
        ),
        Node(
            package='car_simulation',
            executable='waypoint_publisher_node',
            name='waypoint_publisher_node',
            output='screen'
        ),
        Node(
            package='car_simulation',
            executable='color_detector_node',
            name='color_detector_node',
            output='screen'
        )
    ])
