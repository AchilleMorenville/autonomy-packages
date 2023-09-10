import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    lidar_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('aut_lidar_odometry'), 'launch'),
            '/lidar_odometry_launch.py'])
        )
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('aut_localization'), 'launch'),
            '/localization_launch.py'])
        )

    return LaunchDescription([
        localization_launch,
        lidar_odometry_launch
    ])