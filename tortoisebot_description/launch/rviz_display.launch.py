import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Get the rviz config file
    rviz_config_dir = os.path.join(
        get_package_share_directory('tortoisebot_description'),
        'rviz',
        'tortoisebot.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir],
        ),
    ])