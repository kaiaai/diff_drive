import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get package directories
    pkg_description = get_package_share_directory('tortoisebot_description')
    pkg_gazebo = get_package_share_directory('tortoisebot_gazebo')
    
    # Path to URDF file
    urdf_file_name = 'tortoisebot_complete.urdf.xacro'
    urdf_path = os.path.join(pkg_description, 'urdf', urdf_file_name)
    
    # World file path
    world_file_name = 'empty_world.world'
    world_path = os.path.join(pkg_gazebo, 'worlds', world_file_name)
    
    # Process the URDF file
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    # Path Tracker Node
    path_tracker_node = Node(
        package='tortoisebot_description',
        executable='path_tracker.py',
        name='path_tracker',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'path_update_rate': 5.0,
            'max_path_length': 2000
        }],
        output='screen'
    )
    
    # Emergency Stop Node
    emergency_stop_node = Node(
        package='tortoisebot_description',
        executable='emergency_stop.py',
        name='emergency_stop',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'auto_stop_on_bumper': True
        }],
        output='screen'
    )
    
    # Simple Controller Node
    simple_controller_node = Node(
        package='tortoisebot_description',
        executable='simple_robot_controller.py',
        name='simple_robot_controller',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'world',
            default_value=world_path,
            description='Full path to world model file to load'),

        # Start Gazebo first
        ExecuteProcess(
            cmd=['gz', 'sim', LaunchConfiguration('world'), '-v', '4'],
            output='screen',
            name='gazebo'
        ),

        # Robot State Publisher (starts earlier)
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'robot_description': robot_description
                    }],
                ),
            ]
        ),
        
        # Spawn robot
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_tortoisebot',
                    output='screen',
                    arguments=[
                        '-topic', '/robot_description',
                        '-name', 'tortoisebot',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.1'
                    ],
                ),
            ]
        ),

        # Core bridges - essential for robot movement
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='cmd_vel_bridge',
                    arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
                    output='screen'
                ),
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='odom_bridge',
                    arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
                    output='screen'
                ),
            ]
        ),

        # Sensor bridges
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='imu_bridge',
                    arguments=['/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'],
                    output='screen'
                ),
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='lidar_bridge',
                    arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
                    output='screen'
                ),
                # Added odom to base_footprint transform publisher
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='odom_to_base_footprint',
                    arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
                    output='screen'
                ),
            ]
        ),

        # Fixed Bumper bridges - using proper message type
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='front_bumper_bridge',
                    arguments=['/bumper/front_contact@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts'],
                    output='screen'
                ),
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='left_bumper_bridge',
                    arguments=['/bumper/left_contact@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts'],
                    output='screen'
                ),
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='right_bumper_bridge',
                    arguments=['/bumper/right_contact@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts'],
                    output='screen'
                ),
            ]
        ),

        # Start custom nodes after all bridges are ready (FIXED - removed duplicate)
        TimerAction(
            period=12.0,
            actions=[path_tracker_node, emergency_stop_node, simple_controller_node]
        ),
    ])