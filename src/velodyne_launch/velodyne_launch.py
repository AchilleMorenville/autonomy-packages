import os
import yaml

import ament_index_python.packages
import launch
import launch_ros.actions


def generate_launch_description():

    with open("velodyne_params.yaml", 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_convert_node']['ros__parameters']
    with open("velodyne_params.yaml", 'r') as f:
        driver_params = yaml.safe_load(f)['velodyne_driver_node']['ros__parameters']

    driver_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_driver')
    # driver_params_file = os.path.join(driver_share_dir, 'config', 'VLP16-velodyne_driver_node-params.yaml')
    velodyne_driver_node = launch_ros.actions.Node(package='velodyne_driver',
                                                   executable='velodyne_driver_node',
                                                   output='both',
                                                   parameters=[driver_params])

    convert_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    # convert_params_file = os.path.join(convert_share_dir, 'config', 'VLP16-velodyne_transform_node-params.yaml')
    convert_params['calibration'] = os.path.join(convert_share_dir, 'params', 'VLP16db.yaml')
    velodyne_convert_node = launch_ros.actions.Node(package='velodyne_pointcloud',
                                                    executable='velodyne_convert_node',
                                                    output='both',
                                                    parameters=[convert_params])

    laserscan_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_laserscan')
    laserscan_params_file = os.path.join(laserscan_share_dir, 'config', 'default-velodyne_laserscan_node-params.yaml')
    velodyne_laserscan_node = launch_ros.actions.Node(package='velodyne_laserscan',
                                                      executable='velodyne_laserscan_node',
                                                      output='both',
                                                      parameters=[laserscan_params_file])


    return launch.LaunchDescription([velodyne_driver_node,
                                     velodyne_convert_node,
                                     velodyne_laserscan_node,

                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=velodyne_driver_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])