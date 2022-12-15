import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('spot_driver'),
        'config',
        'spot_driver.yaml'
    )

    return LaunchDescription([
        Node(
            package='spot_driver',
            executable='spot_odometry_pub',
            name='spot_odometry_pub',
            parameters=[
                config
            ]
        )
    ])