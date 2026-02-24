import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, TimerAction, OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_lane_simulation = get_package_share_directory('lane_simulation')

    # Path to the xacro file
    xacro_file = os.path.join(pkg_lane_simulation, 'urdf', 'ebot.xacro')
    
    # Process xacro to get URDF XML
    # We use Command substitution to ensure it's evaluated at launch time
    robot_description_config = Command(['xacro ', xacro_file])
    
    # ── Arguments ──────────────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_lane_simulation, 'world', 'osm_pure.world'),
        description='Full path to world model file to load'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to launch Gazebo GUI'
    )

    x_arg = DeclareLaunchArgument('x', default_value='-9.2', description='Robot spawn x position')
    y_arg = DeclareLaunchArgument('y', default_value='-126.3', description='Robot spawn y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.5', description='Robot spawn z position')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='-1.18', description='Robot spawn yaw orientation')

    # ── Environment Variables ───────────────────────────────────────────────────
    set_libgl       = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')
    set_mesa        = SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3')
    
    # Gazebo needs to know where to find package:// meshes and model:// assets.
    pkg_share_parent = os.path.join(pkg_lane_simulation, '..')
    
    # Standard Gazebo 11 paths (may vary, but these are common defaults)
    gazebo_base_paths = [
        '/usr/share/gazebo-11',
        '/usr/share/gazebo'
    ]
    gazebo_model_paths = [p + '/models' for p in gazebo_base_paths]
    
    # Construct GAZEBO_MODEL_PATH
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    # Important: Include the specific models directory inside the package share
    pkg_models_path = os.path.join(pkg_lane_simulation, 'models')
    all_model_paths = [pkg_models_path, pkg_share_parent] + gazebo_model_paths
    if existing_model_path:
        all_model_paths.append(existing_model_path)
    new_model_path = os.pathsep.join([p for p in all_model_paths if os.path.exists(p)])
    set_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', new_model_path)
    
    # Construct GAZEBO_RESOURCE_PATH (critical for shaders)
    existing_resource_path = os.environ.get('GAZEBO_RESOURCE_PATH', '')
    all_resource_paths = gazebo_base_paths
    if existing_resource_path:
        all_resource_paths.append(existing_resource_path)
    new_resource_path = os.pathsep.join([p for p in all_resource_paths if os.path.exists(p)])
    set_resource_path = SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', new_resource_path)
    
    # Disable online model database to speed up startup and prevent hangs
    set_no_db       = SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', '')

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

    # ── Robot State Publisher ──────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description_config, value_type=str)}]
    )

    # ── Spawn Entity ───────────────────────────────────────────────────────────
    # Delayed to allow Gazebo to load the world and ROS plugins first
    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                # Updated spawn coordinates to match the geojson road start
                arguments=[
                    '-topic', 'robot_description', 
                    '-entity', 'ebot', 
                    '-x', LaunchConfiguration('x'), 
                    '-y', LaunchConfiguration('y'), 
                    '-z', LaunchConfiguration('z'),
                    '-Y', LaunchConfiguration('yaw')
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        set_libgl,
        set_mesa,
        set_model_path,
        set_resource_path,
        set_no_db,
        world_arg,
        gui_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
    ])
