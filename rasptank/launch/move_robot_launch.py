import os


from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='rasptank',
            executable='velocity_publisher',
            #parameters=[{'use_sim_time':True}],
        ),
        Node(
            package='rasptank',
            executable='sim_relayer',
            #parameters=[{'use_sim_time':True}],
        ),


    ])