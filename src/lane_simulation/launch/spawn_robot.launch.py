import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('lane_simulation')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    urdf_file = os.path.join(pkg_share, 'urdf', 'ebot.xacro')
    doc = xacro.process_file(urdf_file)
    robot_desc = doc.toprettyxml(indent='  ')
    
    params = {'robot_description': robot_desc, 'use_sim_time': use_sim_time}
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'ebot', '-topic', 'robot_description'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        robot_state_publisher_node,
        spawn_entity
    ])
