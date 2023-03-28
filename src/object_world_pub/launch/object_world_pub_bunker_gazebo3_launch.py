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
    # launch_node_dir = os.path.join(
    #                 get_package_share_directory('cr3_description'),
    #                 'launch/cr3_display_launch.py'
    #             )
    # print(launch_node_dir)
    para_dir = os.path.join(get_package_share_directory('object_world_pub'), 'config', 'cam_info.yaml')    
    return LaunchDescription(
        [           
            # Node(
            #     package='object_world_pub',
            #     executable='pixel_tf_fake_pub',
            #     name='pixel_fake_pub_node',
            #     output='screen',
            #     emulate_tty = True
            # ),
            Node(
                package='object_world_pub',
                executable='object_world_pub_bunker_gazebo3',
                name='object_world_pub_bunker_gazebo3_node',
                parameters=[para_dir], 
                output='screen',
                emulate_tty = True
            )          
        ]
    )