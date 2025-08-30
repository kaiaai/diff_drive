import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    # Launch configuration variables
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Get the launch directory
    gazebo_launch_dir = os.path.join(get_package_share_directory('tortoisebot_gazebo'), 'launch')

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_dir, '/gazebo_world.launch.py']),
    )

    # Include the spawn launch file with a delay
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([gazebo_launch_dir, '/spawn_tortoisebot.launch.py']),
                launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'x_pose', default_value='0.0',
            description='x position of robot spawn'),
        DeclareLaunchArgument(
            'y_pose', default_value='0.0',
            description='y position of robot spawn'),

        gazebo,
        spawn_robot,
    ])