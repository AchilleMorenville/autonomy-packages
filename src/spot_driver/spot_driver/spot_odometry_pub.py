import numpy as np
import time

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import std_msgs.msg as std_msgs
from std_srvs.srv import Trigger
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time

from nav_msgs.msg import Odometry

from bosdyn.client import create_standard_sdk
from bosdyn.client.frame_helpers import get_a_tform_b, VISION_FRAME_NAME, ODOM_FRAME_NAME, BODY_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME
from bosdyn.client.robot_state import RobotStateClient

from bosdyn.api import robot_state_pb2

class SpotOdometryPublisher(Node):

    def __init__(self):
        super().__init__('spot_odometry_publisher')

        self.declare_parameter('spot_odometry_frequency', 50)
        self.spot_odometry_frequency = self.get_parameter('spot_odometry_frequency').get_parameter_value().integer_value

        self.declare_parameter('spot_ip', '192.168.80.3')
        self.spot_ip = self.get_parameter('spot_ip').get_parameter_value().string_value
        
        self.declare_parameter('spot_username', 'user')
        self.spot_username = self.get_parameter('spot_username').get_parameter_value().string_value
        
        self.declare_parameter('spot_password', 'upsa43jm7vnf')
        self.spot_password = self.get_parameter('spot_password').get_parameter_value().string_value

        # # Initialize the transform broadcasters
        # self.tf_broadcaster = TransformBroadcaster(self)
        # self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Create services
        self.srv_Connect = self.create_service(Trigger, "spot_driver/odometry/connect", self.connect_call_back)
        self.connected_to_spot = False

        # Create publishers

        self.ko_publisher_ = self.create_publisher(Odometry, "spot_driver/odometry/ko_odom", 1000)
        self.vo_publisher_ = self.create_publisher(Odometry, "spot_driver/odometry/vo_odom", 1000)
        self.gravity_publisher_ = self.create_publisher(TransformStamped, "spot_driver/state/gravity", 1000)

        self.sdk = None
        self.robot = None
        self.timer = None

    def connect_call_back(self, request, response):

        if self.connected_to_spot:
            response.success = True
            response.message = "Already connected to spot"
            return response

        # Connect to Spot
        try:
            self.get_logger().info("Conntecting to Spot...")
            self.sdk = create_standard_sdk('OdometryClient')
            self.robot = self.sdk.create_robot(self.spot_ip)
            self.robot.authenticate(self.spot_username, self.spot_password)
            self.robot.sync_with_directory()
            self.robot.time_sync.wait_for_sync()
            self.get_logger().info("Connected to Spot")

        except Exception as e:
            response.success = False 
            response.message = "Failed to connect to Spot. Verify spot ip, username and password and the network connection."
            return response

        # Ensure clients
        self.robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)

        timer_period = 1.0 / self.spot_odometry_frequency  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.connected_to_spot = True

        response.success = True
        response.message = "Connected to Spot and publisher started"
        return response

    def timer_callback(self):

        # Request robot state proto from Spot
        robot_state = self.robot_state_client.get_robot_state()

        stamp = self.get_clock().now().to_msg()

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

        # Gravity
        body_tform_gravity = get_a_tform_b(
            transforms_snapshot,
            BODY_FRAME_NAME,
            GRAV_ALIGNED_BODY_FRAME_NAME
        )

        ko = Odometry()  # Kinematic Odometry
        vo = Odometry()  # Visual Odometry
        grav = TransformStamped()  # Gravity

        # stamp = self.get_clock().now().to_msg()

        ko.header.stamp = stamp
        ko.header.frame_id = 'body'
        ko.child_frame_id = 'odom'

        ko.pose.pose.position.x = body_tform_odom.x
        ko.pose.pose.position.y = body_tform_odom.y
        ko.pose.pose.position.z = body_tform_odom.z

        ko.pose.pose.orientation.x = body_tform_odom.rot.x
        ko.pose.pose.orientation.y = body_tform_odom.rot.y
        ko.pose.pose.orientation.z = body_tform_odom.rot.z
        ko.pose.pose.orientation.w = body_tform_odom.rot.w

        vo.header.stamp = stamp
        vo.header.frame_id = 'body'
        vo.child_frame_id = 'vision'

        vo.pose.pose.position.x = body_tform_vision.x
        vo.pose.pose.position.y = body_tform_vision.y
        vo.pose.pose.position.z = body_tform_vision.z

        vo.pose.pose.orientation.x = body_tform_vision.rot.x
        vo.pose.pose.orientation.y = body_tform_vision.rot.y
        vo.pose.pose.orientation.z = body_tform_vision.rot.z
        vo.pose.pose.orientation.w = body_tform_vision.rot.w

        grav.header.stamp = stamp
        grav.header.frame_id = 'body'
        grav.child_frame_id = 'gravity'

        grav.transform.translation.x = body_tform_gravity.x
        grav.transform.translation.y = body_tform_gravity.y
        grav.transform.translation.z = body_tform_gravity.z

        grav.transform.rotation.x = body_tform_gravity.rot.x
        grav.transform.rotation.y = body_tform_gravity.rot.y
        grav.transform.rotation.z = body_tform_gravity.rot.z
        grav.transform.rotation.w = body_tform_gravity.rot.w

        # Send the transformation
        # self.tf_broadcaster.sendTransform(ko)
        # self.tf_broadcaster.sendTransform(vo)
        # self.tf_broadcaster.sendTransform(grav)

        self.ko_publisher_.publish(ko)
        self.vo_publisher_.publish(vo)
        self.gravity_publisher_.publish(grav)


def main(args=None):
    rclpy.init(args=args)

    spot_odometry_publisher = SpotOdometryPublisher()

    rclpy.spin(spot_odometry_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    spot_odometry_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()