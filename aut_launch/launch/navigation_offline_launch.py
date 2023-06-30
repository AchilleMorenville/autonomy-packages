import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('aut_localization'), 'launch'),
            '/localization_launch.py'])
        )
    lidar_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('aut_lidar_odometry'), 'launch'),
            '/lidar_odometry_launch.py'])
        )
    global_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('aut_global_planner'), 'launch'),
            '/global_planner_launch.py'])
        )
    local_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('aut_local_planner'), 'launch'),
            '/local_planner_launch.py'])
        )


    return LaunchDescription([
        localization_launch,
        lidar_odometry_launch,
        global_planner_launch,
        local_planner_launch
    ])