import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get the urdf file
    urdf_file_name = 'tortoisebot_complete.urdf.xacro'
    urdf = os.path.join(
        get_package_share_directory('tortoisebot_description'),
        'urdf',
        urdf_file_name)
    
    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'x_pose', default_value='0.0',
            description='x position of robot spawn'),
        DeclareLaunchArgument(
            'y_pose', default_value='0.0',
            description='y position of robot spawn'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'robot_description': Command(['xacro ', urdf, ' gazebo:=true'])}],
        ),
        
        # Updated to use new Gazebo spawn entity
        Node(
            package='ros_gz_sim',
            executable='create',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "/robot_description",
                      "-name", "tortoisebot",
                      "-x", x_pose,
                      "-y", y_pose,
                      "-z", "0.01"],
        ),
    ])