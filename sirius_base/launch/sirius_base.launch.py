import os
from ament_index_python import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
def generate_launch_description():
    kobuki_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('kobuki_node'),
                'launch/kobuki_node-launch.py'
            )
        )
    )
    velodyne_launch = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(
             os.path.join(
                 get_package_share_directory('velodyne'),
                 'launch/velodyne-all-nodes-VLP16-composed-launch.py'
             )
         )
     )

    #velodyne_launch = IncludeLaunchDescription(
     #   PythonLaunchDescriptionSource(
      #      os.path.join(
       #         get_package_share_directory('urg_node'),
        #        'launch/urg_node_launch.py'
         #   )
        #)
    #)

    #lidarslam_launch = IncludeLaunchDescription(
       # PythonLaunchDescriptionSource(
        #    os.path.join(
         #       get_package_share_directory('lidarslam'),
           #     'launch/lidarslam.launch.py'
           # )
       # )
   # ) #Localization

    return LaunchDescription([
        kobuki_launch,
        velodyne_launch,
        #tf2_node,
        #lidarslam_launch
    ])
