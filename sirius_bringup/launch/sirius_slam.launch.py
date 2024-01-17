import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sirius_launch = get_package_share_directory('sirius_bringup')
    sirius_launch_path = os.path.join(sirius_launch, 'launch')
    
    slam_launch = get_package_share_directory('slam_toolbox')
    slam_launch_path = os.path.join(slam_launch, 'launch')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [sirius_launch_path, '/sirius_bringup.launch.py']
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [slam_launch_path, '/online_async_launch.py']
            )
        ),

    ])
