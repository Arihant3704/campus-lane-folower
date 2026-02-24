import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_lane_simulation = get_package_share_directory('lane_simulation')

    models_path = os.path.join(pkg_lane_simulation, 'models')

    # ── Arguments ──────────────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_lane_simulation, 'world', 'course.world'),
        description='Full path to world model file to load'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to launch Gazebo GUI'
    )

    # ── Environment Variables ───────────────────────────────────────────────────
    set_libgl       = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')
    set_mesa        = SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3')
    set_model_path  = SetEnvironmentVariable('GAZEBO_MODEL_PATH', models_path)

    # ── Gazebo Server ──────────────────────────────────────────────────────────
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world':   LaunchConfiguration('world'),
            'verbose': 'true',
        }.items()
    )

    # ── Gazebo GUI (optional) ──────────────────────────────────────────────────
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # ── Lane Detection Node ────────────────────────────────────────────────────
    # Delayed by 5 s to give Gazebo time to fully start up before connecting
    lane_detector = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='lane_detection',
                executable='lane_detector_node',
                name='lane_detector_node',
                output='screen',
                parameters=[{
                    'canny_low':           50,
                    'canny_high':          150,
                    'hough_threshold':     50,
                    'hough_min_line_len':  100,
                    'hough_max_line_gap':  50,
                    'curvature_threshold': 0.2,
                }]
            )
        ]
    )

    return LaunchDescription([
        # env vars first so child processes inherit them
        set_libgl,
        set_mesa,
        set_model_path,
        # args
        world_arg,
        gui_arg,
        # processes
        gzserver,
        gzclient,
        lane_detector,
    ])
