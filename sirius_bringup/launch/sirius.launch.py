import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # laser_frame_config_arg = DeclareLaunchArgument(
    #     'tf2_config', default_value='laser_frame_config.yaml'
    # )

    # get_point_flg_arg = DeclareLaunchArgument(
    #     'get_point_flg', default_value=False
    # )


    config_pkg = get_package_share_directory('sirius_bringup')
    config_path1 = os.path.join(config_pkg, 'config', 'laser_frame_config.yaml')
    config_path2 = os.path.join(config_pkg, 'config', 'base_config.yaml')
    
    kobuki_node_pkg = get_package_share_directory('kobuki_node')
    kobuki_node_path = os.path.join(kobuki_node_pkg, 'launch')

    velodyne_pkg = get_package_share_directory('velodyne')
    velodyne_path = os.path.join(velodyne_pkg, 'launch')

    urg_node_pkg = get_package_share_directory('urg_node')
    urg_node_path = os.path.join(urg_node_pkg, 'launch')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [kobuki_node_path, '/kobuki_node-launch.py']
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [velodyne_path, '/velodyne-all-nodes-VLP16-composed-launch.py']
            )
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [urg_node_path, '/urg_node_launch.py']
        #     )
        # ),

        # Node(
        #     package = 'sirius_bringup',
        #     namespace = 'sirius',
        #     executable = 'static_tf2_broadcaster',
        #     name = 'static_tf2_broadcaster',
        #     parameters = [config_path2]
        #),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.5', '0', '0', '0', 'base_footprint', 'velodyne']
        ),

        # Node(
        #     package = 'sirius_base',
        #     executable = 'sirius_base_node',
        #     name = 'sirius_base_node'
        # ),

        # Node(
        #     package = 'rviz2',
        #     executable = 'rviz2',
        #     name = 'rviz2',
        #     arguments = ['-d', [os.path.join(config_pkg, 'rviz2', 'sirius.rviz')]]
        # )

        Node(
            package = 'tf2_broadcaster',
            namespace = 'tf2_broadcaster',
            executable = 'tf2_broadcaster_node',
            name = 'tf2_broadcaster_node'
        ),

    ])
