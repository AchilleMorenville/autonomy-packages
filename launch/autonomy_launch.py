import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
	slam_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([os.path.join(
			get_package_share_directory('slam'), 'launch'),
			'/slam_launch.py'])
		)
	spot_driver_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([os.path.join(
			get_package_share_directory('spot_driver'), 'launch'),
			'/spot_driver_launch.py'])
		)
	velodyne_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([
			'/ros2_ws/src/velodyne_launch',
			'/velodyne_launch.py']),
		)

	return LaunchDescription([
		slam_launch,
		spot_driver_launch,
		velodyne_launch
	])