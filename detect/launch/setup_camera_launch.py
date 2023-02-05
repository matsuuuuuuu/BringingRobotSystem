import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import launch_ros.actions
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        
        Node(
            package='detect',
            executable='setup_stereocamera',
            name='object_detect_system',
            output='screen',
        ),
        Node(
            package='detect',
            executable='camera_server',
            name='camera_server',
            output='screen',
        ),

    ])