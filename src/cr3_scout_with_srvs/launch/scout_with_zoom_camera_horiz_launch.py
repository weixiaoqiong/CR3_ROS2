import os
from struct import pack

from rospkg import get_package_name
import launch
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # para_dir = os.path.join(get_package_share_directory('cr3_scout_with_srvs'), 'config', 'cam_info.yaml')
    return LaunchDescription(
        [
            Node(
                package='cr3_scout_with_srvs',
                # namespace='cr3_ns',
                executable='scout_with_zoom_camera_horiz',
                name='scout_node',
                # parameters=[para_dir], 
                output='screen',
                emulate_tty = True
            )
        ]
    )