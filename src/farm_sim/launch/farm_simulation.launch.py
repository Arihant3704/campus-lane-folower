import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node 

def generate_launch_description():
    # Get the path to the farm_sim package
    farm_sim_pkg_dir = get_package_share_directory('farm_sim')
    virtual_maize_field_pkg_dir = get_package_share_directory('virtual_maize_field')
    gazebo_ros_pkg_dir = get_package_share_directory('gazebo_ros')

    # Construct the full path to the world file
    world_path = os.path.join(farm_sim_pkg_dir, 'worlds', 'custom_maize.world')

    model_path = os.pathsep.join([
            os.path.join(farm_sim_pkg_dir, 'models'),
            os.path.join(virtual_maize_field_pkg_dir, 'models'),
            os.environ.get('GAZEBO_MODEL_PATH', '') 
        ])
    
    # Set Gazebo resource path to include virtual_maize_field models and our farm_robot models
    gazebo_resource_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        model_path
    )

    # Launch Gazebo Classic
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg_dir, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(os.path.join(get_package_share_directory('car_simulation'), 'urdf', 'car.urdf')).read()}]
    )

    # Spawn the farm_robot
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_farm_robot',
        output='screen',
        arguments=[
            '-entity', 'farm_robot',
            '-topic', 'robot_description',
            '-x', '12.0',
            '-y', '4.0',
            '-z', '0.3',
            '-Y', '3.14' # Face Left (West)
        ]
    )

    # Run the serpentine_navigator node
    navigator_node = Node(
        package='farm_sim',
        executable='navigation_node',
        name='serpentine_navigator',
        output='screen',
        namespace='/demo',
        parameters=[
            {'farm_center_lat': 15.3478}, # Will be overridden by GPS fix
            {'farm_center_lon': 75.1338},
            {'farm_size_meters': 24.0}, # Covers approx -12 to +12 (Crops are -10 to +10)
            {'grid_spacing_meters': 2.0}, # Matches crop row spacing
            {'vehicle_speed': 0.5},
            {'angular_speed': 0.5},
            {'angular_speed': 0.5},
            {'crop_csv_path': os.path.join(farm_sim_pkg_dir, 'config', 'crop_coordinates.csv')}
        ]
    )

    return LaunchDescription([
        gazebo_resource_path,
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_robot_node,
        navigator_node,
    ])
