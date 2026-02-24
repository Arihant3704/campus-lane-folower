import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_lane_simulation = get_package_share_directory('lane_simulation')

    models_path = os.path.join(pkg_lane_simulation, 'models')

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

    # Force software rendering to avoid GPU driver crashes
    set_libgl = SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1')
    set_mesa = SetEnvironmentVariable(name='MESA_GL_VERSION_OVERRIDE', value='3.3')

    # Gazebo model path for custom models  ← was in LaunchDescription before but never added!
    set_gazebo_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=models_path)

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'true',
        }.items()
    )

    # Only launch gzclient when gui:=true
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    return LaunchDescription([
        # Environment variables MUST come first so child processes inherit them
        set_libgl,
        set_mesa,
        set_gazebo_path,
        # Args
        world_arg,
        gui_arg,
        # Processes
        gzserver,
        gzclient,
    ])
