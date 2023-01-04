import numpy as np
import time

import rclpy
from rclpy.node import Node

# from tf2_ros import TransformBroadcaster
# from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import std_msgs.msg as std_msgs
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Time

from autonomous_interfaces.msg import Fiducial, Fiducials

from bosdyn.client import create_standard_sdk
from bosdyn.client.frame_helpers import get_a_tform_b, VISION_FRAME_NAME, ODOM_FRAME_NAME, BODY_FRAME_NAME
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.api import world_object_pb2

from bosdyn.api import robot_state_pb2

class SpotFiducialPublisher(Node):

    def __init__(self):
        super().__init__('spot_fiducial_publisher')

        self.declare_parameter('spot_fiducial_frequency', 10.0)
        self.spot_fiducial_frequency = self.get_parameter('spot_fiducial_frequency').get_parameter_value().double_value

        self.declare_parameter('spot_ip', '192.168.80.3')
        self.spot_ip = self.get_parameter('spot_ip').get_parameter_value().string_value
        
        self.declare_parameter('spot_username', 'user')
        self.spot_username = self.get_parameter('spot_username').get_parameter_value().string_value
        
        self.declare_parameter('spot_password', 'upsa43jm7vnf')
        self.spot_password = self.get_parameter('spot_password').get_parameter_value().string_value

        # Initialize the transform broadcasters
        # self.tf_broadcaster = TransformBroadcaster(self)

        # Create services
        self.srv_Connect = self.create_service(Trigger, "spot_driver/fiducial/connect", self.connect_call_back)

        self.connected_to_spot = False

        self.last_detection = {}

    def connect_call_back(self, request, response):

        if self.connected_to_spot:
            response.success = True
            response.message = ""
            return response

        # Connect to Spot
        try:

            self.get_logger().info("Conntecting to Spot...")

            self.sdk = create_standard_sdk('FiducialClient')
            self.robot = self.sdk.create_robot(self.spot_ip)
            self.robot.authenticate(self.spot_username, self.spot_password)
            self.robot.sync_with_directory()
            self.robot.time_sync.wait_for_sync()

            self.get_logger().info("Connected to Spot")

        except Exception as e:
            response.success = False 
            response.message = "Failed to connect to Spot. Verify spot ip, username, password and the network connection."
            return response

        # Ensure clients
        self.robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        self.world_object_client = self.robot.ensure_client(WorldObjectClient.default_service_name)

        timer_period = 1.0 / self.spot_fiducial_frequency  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.fiducials_publisher = self.create_publisher(Fiducials, "spot_driver/fiducials", 10)

        self.connected_to_spot = True

        response.success = True
        response.message = ""
        return response

    def timer_callback(self):

        # Get fiducials
        world_objects = self.world_object_client.list_world_objects(
            object_type=[world_object_pb2.WORLD_OBJECT_APRILTAG]
        ).world_objects

        stamp = self.get_clock().now().to_msg()

        list_fiducials = []

        for world_object in world_objects:

            if world_object.apriltag_properties.fiducial_pose_status != world_object_pb2.AprilTagProperties.AprilTagPoseStatus.STATUS_OK:
                continue

            timestamp = world_object.acquisition_time

            tag_id = world_object.apriltag_properties.tag_id

            # Sends only when new fiducial as been detected
            if tag_id in self.last_detection:
                if timestamp.seconds - self.last_detection[tag_id].seconds == 0 and timestamp.nanos - self.last_detection[tag_id].nanos < 100:
                    continue
                else:
                    self.last_detection[tag_id] = timestamp
            else:
                self.last_detection[tag_id] = timestamp

            stamp.sec = timestamp.seconds - self.robot.time_sync.get_robot_clock_skew().seconds
            stamp.nanosec = timestamp.nanos - self.robot.time_sync.get_robot_clock_skew().nanos

            if stamp.nanosec < 0:
                stamp.nanosec = stamp.nanosec + 1000000000
                stamp.sec = stamp.sec - 1

            self.get_logger().info(f"Fiducial {tag_id} detected at {stamp.sec}.{stamp.nanosec} s")

            transforms_snapshot = world_object.transforms_snapshot

            body_tform_fiducial = get_a_tform_b(
                transforms_snapshot,
                BODY_FRAME_NAME,
                world_object.apriltag_properties.frame_name_fiducial_filtered
            )

            fiducial = Fiducial()

            pose_fiducial = Pose()

            pose_fiducial.position.x = body_tform_fiducial.x
            pose_fiducial.position.y = body_tform_fiducial.y
            pose_fiducial.position.z = body_tform_fiducial.z

            pose_fiducial.orientation.x = body_tform_fiducial.rot.x
            pose_fiducial.orientation.y = body_tform_fiducial.rot.y
            pose_fiducial.orientation.z = body_tform_fiducial.rot.z
            pose_fiducial.orientation.w = body_tform_fiducial.rot.w

            fiducial.tag_id = tag_id
            fiducial.stamp = stamp
            fiducial.pose = pose_fiducial

            list_fiducials.append(fiducial)

        if len(list_fiducials) > 0:
            msg = Fiducials()
            msg.nbr = len(list_fiducials)
            msg.fiducials = list_fiducials
            self.fiducials_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    spot_fiducial_publisher = SpotFiducialPublisher()

    rclpy.spin(spot_fiducial_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    spot_fiducial_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()