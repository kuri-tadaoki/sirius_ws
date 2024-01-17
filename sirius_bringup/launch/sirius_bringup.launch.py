import os
import math
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sirius_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sirius_base'),
                'launch/sirius_base.launch.py'
            )
        )
    )
    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=['0', '0', '0','0', '0', '0','base_footprint','base_link'],
                    )
    #qz = str(math.pi/2) = 1.57079632
    tf2_node2 = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=['0', '0', '0','0', '0', '0','base_footprint','velodyne'],
                    )

    # urg_node_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('urg_node'),
    #             'launch/urg_node_launch.py'
    #         )
    #     )
    # )
    # urg_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0','0','1.0','0','0','0','1','base_link','laser']
    #     )
    # rviz2_config = os.path.join(get_package_share_directory('sirius_base'), 'rviz2/sirius.rviz')
    # rviz2_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=[
    #         '-d', rviz2_config
    #     ]
    # )

    return LaunchDescription([
        sirius_launch,
        tf2_node,
        tf2_node2
        #urg_node_launch,
        #urg_tf
        #rviz2_node,
    ])

#playsound("UC3000.mp3")
