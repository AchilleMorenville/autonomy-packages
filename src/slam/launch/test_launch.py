import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('slam'),
        'config',
        'slam_params.yaml'
    )

    with open(config, 'r') as f:
        feature_extraction_params = yaml.safe_load(f)['feature_extraction']['ros__parameters']
    with open(config, 'r') as f:
        graph_optimization_params = yaml.safe_load(f)['graph_optimization']['ros__parameters']

    return LaunchDescription([
        Node(
            package='slam',
            executable='feature_extraction',
            name='feature_extraction',
            parameters=[
                feature_extraction_params
            ]
        ),
        Node(
            package='slam',
            executable='lidar_odometry',
            name='lidar_odometry'
        )
    ])