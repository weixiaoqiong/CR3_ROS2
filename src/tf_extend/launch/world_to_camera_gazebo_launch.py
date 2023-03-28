import os
from struct import pack

from rospkg import get_package_name
import launch
from ament_index_python import get_package_share_directory
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription(
        [        
            Node(
                package='tf_extend',
                executable='world_to_camera_pub',
                name='world2camera_pub_node',
                output='screen',
                emulate_tty = True
            )
        ]
    )