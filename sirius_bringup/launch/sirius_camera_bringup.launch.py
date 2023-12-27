import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from playsound import playsound

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
                    node_executable='static_transform_publisher',
                    node_name='static_tf_pub_laser',
                    arguments=['0', '0', '0.2','0', '0', '0', '1','base_footprint','base_link'],
                    )
                    
    tf2_node_baselink = Node(package='tf2_ros',
                    node_executable='static_transform_publisher',
                    node_name='static_tf_pub_laser',
                    arguments=['0', '0', '0.8','0', '0', '0.7071067811865475', '0.7071067811865475','base_link','velodyne'],
                    )

    tf2_node_camera1 = Node(package='tf2_ros',
                    node_executable='static_transform_publisher',
                    node_name='static_tf_pub_laser',
                    arguments=['0.26', '0', '0.33','0', '0', '0', '1','base_link','center_camera'],
                    )

    tf2_node_camera2 = Node(package='tf2_ros',
                    node_executable='static_transform_publisher',
                    node_name='static_tf_pub_laser',
                    arguments=['0.243', '0.062', '0.33','0', '0', '-0.5', '-0.866666','base_link','left_camera'],
                    ) #left

    tf2_node_camera3 = Node(package='tf2_ros',
                    node_executable='static_transform_publisher',
                    node_name='static_tf_pub_laser',
                    arguments=['0.243', '-0.062', '0.33','0', '0', '0.5', '-0.866666','base_link','right_camera'],
                    ) #right
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
        tf2_node_baselink,
        tf2_node_camera1,
        tf2_node_camera2,
        tf2_node_camera3,
        #urg_node_launch,
        #urg_tf
        #rviz2_node,
    ])

#playsound("UC3000.mp3")
