import numpy as np
import time

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import std_msgs.msg as std_msgs
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time

from bosdyn.client import create_standard_sdk
from bosdyn.client.frame_helpers import get_a_tform_b, VISION_FRAME_NAME, ODOM_FRAME_NAME, BODY_FRAME_NAME
from bosdyn.client.robot_state import RobotStateClient

from bosdyn.api import robot_state_pb2

class SpotOdometryPublisher(Node):

    def __init__(self, hostname, password):
        super().__init__('spot_odometry_publisher')

        self.declare_parameter('spot_odometry_frequency', 100)
        spot_odometry_frequency = self.get_parameter('spot_odometry_frequency').get_parameter_value().integer_value

        self.get_logger().info(f"spot_odometry_frequency param value : {spot_odometry_frequency}")


        self.get_logger().info("Connecting...")

        # Create and connect robot
        self.sdk = create_standard_sdk('OdometryClient')
        self.robot = self.sdk.create_robot(hostname)
        self.robot.authenticate('user', password)
        self.robot.sync_with_directory()
        self.robot.time_sync.wait_for_sync()

        self.get_logger().info(f"Time synced : {self.robot.time_sync.has_established_time_sync}")

        self.get_logger().info(f"Clock skew : {self.robot.time_sync.get_robot_clock_skew()}")

        self.get_logger().info(f"RobotTimeConverter : {self.robot.time_sync.get_robot_time_converter()}")

        self.get_logger().info("Connected...")

        # Ensure clients
        self.point_cloud_client = self.robot.ensure_client('velodyne-point-cloud')
        self.robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)

        # Initialize the transform broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Publish velodyne pose
        self.publish_body_tform_velodyne()

        # Create a timed callback
        timer_period = 1.0 / spot_odometry_frequency  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def publish_body_tform_velodyne(self):

        # Get a point cloud
        point_clouds = self.point_cloud_client.get_point_cloud_from_sources(["velodyne-point-cloud"])

        point_cloud_transforms_snapshot = point_clouds[0].point_cloud.source.transforms_snapshot
        body_tform_velodyne = get_a_tform_b(
            point_cloud_transforms_snapshot,
            BODY_FRAME_NAME,
            "sensor"
        )

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'body'
        t.child_frame_id = 'velodyne'

        t.transform.translation.x = body_tform_velodyne.x
        t.transform.translation.y = body_tform_velodyne.y
        t.transform.translation.z = body_tform_velodyne.z

        t.transform.rotation.x = body_tform_velodyne.rot.x
        t.transform.rotation.y = body_tform_velodyne.rot.y
        t.transform.rotation.z = body_tform_velodyne.rot.z
        t.transform.rotation.w = body_tform_velodyne.rot.w

        self.tf_static_broadcaster.sendTransform(t)

    def timer_callback(self):

        # Request robot state proto from Spot
        robot_state = self.robot_state_client.get_robot_state()

        stamp = self.get_clock().now().to_msg()  # Save ros timestamp as close as possible from the request TODO maybe take the one from the proto directly

        timestamp = robot_state.kinematic_state.acquisition_timestamp
        timestamp.seconds = timestamp.seconds - self.robot.time_sync.get_robot_clock_skew().seconds
        timestamp.nanos = timestamp.nanos - self.robot.time_sync.get_robot_clock_skew().nanos

        if timestamp.nanos < 0:
            timestamp.nanos = timestamp.nanos + 1000000000
            timestamp.seconds = timestamp.seconds - 1

        stamp.sec = timestamp.seconds
        stamp.nanosec = timestamp.nanos

        kinematic_state = robot_state.kinematic_state
        transforms_snapshot = kinematic_state.transforms_snapshot

        # Kinematic odometry
        body_tform_odom = get_a_tform_b(
            transforms_snapshot,
            BODY_FRAME_NAME,
            ODOM_FRAME_NAME
        )

        # Vision odometry
        body_tform_vision = get_a_tform_b(
            transforms_snapshot,
            BODY_FRAME_NAME,
            VISION_FRAME_NAME
        )

        ko = TransformStamped() # Kinematic Odometry
        vo = TransformStamped() # Visual Odometry

        stamp = self.get_clock().now().to_msg()

        ko.header.stamp = stamp
        ko.header.frame_id = 'body'
        ko.child_frame_id = 'odom'

        ko.transform.translation.x = body_tform_odom.x
        ko.transform.translation.y = body_tform_odom.y
        ko.transform.translation.z = body_tform_odom.z

        ko.transform.rotation.x = body_tform_odom.rot.x
        ko.transform.rotation.y = body_tform_odom.rot.y
        ko.transform.rotation.z = body_tform_odom.rot.z
        ko.transform.rotation.w = body_tform_odom.rot.w

        vo.header.stamp = stamp
        vo.header.frame_id = 'body'
        vo.child_frame_id = 'vision'

        vo.transform.translation.x = body_tform_vision.x
        vo.transform.translation.y = body_tform_vision.y
        vo.transform.translation.z = body_tform_vision.z

        vo.transform.rotation.x = body_tform_vision.rot.x
        vo.transform.rotation.y = body_tform_vision.rot.y
        vo.transform.rotation.z = body_tform_vision.rot.z
        vo.transform.rotation.w = body_tform_vision.rot.w

        # Send the transformation
        self.tf_broadcaster.sendTransform(ko)
        self.tf_broadcaster.sendTransform(vo)


def main(args=None):
    rclpy.init(args=args)

    spot_odometry_publisher = SpotOdometryPublisher('192.168.80.3', 'upsa43jm7vnf')

    rclpy.spin(spot_odometry_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    spot_odometry_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()