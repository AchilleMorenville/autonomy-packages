import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from bosdyn.client import create_standard_sdk
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.api import world_object_pb2
from bosdyn.client.frame_helpers import get_a_tform_b, VISION_FRAME_NAME, ODOM_FRAME_NAME, BODY_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME

from aut_msgs.msg import Fiducial

def create_TransformStamped(bosdyn_transform, frame_id, child_frame_id, stamp):
	trans = TransformStamped()
	trans.header.stamp = stamp
	trans.header.frame_id = frame_id
	trans.child_frame_id = child_frame_id
	trans.transform.translation.x = bosdyn_transform.x
	trans.transform.translation.y = bosdyn_transform.y
	trans.transform.translation.z = bosdyn_transform.z
	trans.transform.rotation.x = bosdyn_transform.rot.x
	trans.transform.rotation.y = bosdyn_transform.rot.y
	trans.transform.rotation.z = bosdyn_transform.rot.z
	trans.transform.rotation.w = bosdyn_transform.rot.w
	return trans

class SpotData(Node):

	def __init__(self):
		super().__init__("spot_data")

		self.connected_to_spot = False
		self.publishing_odom = False

		self.robot_state_client = None
		self.timer_odom = None
		self.timer_vision = None

		self.last_detection = {}

		# Paramerters

		# Connection
		self.declare_parameter("spot_ip", "ip")
		self.spot_ip = self.get_parameter("spot_ip").get_parameter_value().string_value

		self.declare_parameter("spot_username", "username")
		self.spot_username = self.get_parameter("spot_username").get_parameter_value().string_value

		self.declare_parameter("spot_password", "password")
		self.spot_password = self.get_parameter("spot_password").get_parameter_value().string_value

		# State
		self.declare_parameter("spot_state_frequency", 50)
		self.spot_state_frequency = self.get_parameter("spot_state_frequency").get_parameter_value().integer_value

		# Fiducials
		self.declare_parameter("spot_fiducials_frequency", 2)
		self.spot_fiducials_frequency = self.get_parameter("spot_fiducials_frequency").get_parameter_value().integer_value

		# LocalMap
		self.declare_parameter("spot_local_map_frequency", 10)
		self.spot_local_map_frequency = self.get_parameter("spot_local_map_frequency").get_parameter_value().integer_value

		# # Publishers
		# self.ko_publisher = self.create_publisher(Odometry, "aut_spot/odometry/k_odom", 10)
		# self.vo_publisher = self.create_publisher(Odometry, "aut_spot/odometry/v_odom", 10)

		self.fiducials_publisher = self.create_publisher(Fiducial, "aut_spot/fiducial", 10)

		# TF2 Broadcaster
		self.tf_broadcaster = TransformBroadcaster(self)

		self.get_logger().info('Connecting to Spot...')
		# Connect to Spot
		self.sdk = create_standard_sdk('DataClient')
		self.robot = self.sdk.create_robot(self.spot_ip)
		self.robot.authenticate(self.spot_username, self.spot_password)
		self.robot.sync_with_directory()
		self.robot.time_sync.wait_for_sync()
		self.get_logger().info('Connected')

		# Launch odometries
		self.robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
		
		timer_period_state = 1.0 / self.spot_state_frequency
		self.timer_state = self.create_timer(timer_period_state, self.state_call_back)

		# Launch fiducials
		self.world_object_client = self.robot.ensure_client(WorldObjectClient.default_service_name)

		timer_period_fiducials = 1.0 / self.spot_fiducials_frequency
		self.timer_fiducials = self.create_timer(timer_period_fiducials, self.fiducials_call_back)

	def state_call_back(self):
		robot_state = self.robot_state_client.get_robot_state()

		stamp_robot = robot_state.kinematic_state.acquisition_timestamp
		seconds_real = stamp_robot.seconds - self.robot.time_sync.get_robot_clock_skew().seconds
		nanos_real = stamp_robot.nanos - self.robot.time_sync.get_robot_clock_skew().nanos

		if nanos_real < 0:
			nanos_real = nanos_real + 1_000_000_000
			seconds_real = seconds_real - 1

		stamp_real = Time()
		stamp_real.sec = seconds_real
		stamp_real.nanosec = nanos_real

		transforms_snapshot = robot_state.kinematic_state.transforms_snapshot

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
			"flat_body"
		)

		ko = create_TransformStamped(body_tform_odom, "base_link", "k_odom", stamp_real)
		vo = create_TransformStamped(body_tform_vision, "base_link", "v_odom", stamp_real)
		grav = create_TransformStamped(body_tform_gravity, "base_link", "gravity", stamp_real)

		self.tf_broadcaster.sendTransform(ko)
		self.tf_broadcaster.sendTransform(vo)
		self.tf_broadcaster.sendTransform(grav)

		self.get_logger().info('Published odometries')

	def fiducials_call_back(self):

		# Get fiducials
		world_objects = self.world_object_client.list_world_objects(
			object_type=[world_object_pb2.WORLD_OBJECT_APRILTAG]
		).world_objects

		stamp = self.get_clock().now().to_msg()

		for world_object in world_objects:

			if world_object.apriltag_properties.fiducial_pose_status != world_object_pb2.AprilTagProperties.AprilTagPoseStatus.STATUS_OK:
				continue

			timestamp = world_object.acquisition_time

			tag_id = world_object.apriltag_properties.tag_id

			# Sends only when new fiducial as been detected because fiducials are stored and reported during 15s after last appearance.
			# if tag_id in self.last_detection:
			# 	if timestamp.seconds - self.last_detection[tag_id].seconds == 0 and timestamp.nanos - self.last_detection[tag_id].nanos < 100:
			# 		continue
			# 	else:
			# 		self.last_detection[tag_id] = timestamp
			# else:
			# 	self.last_detection[tag_id] = timestamp

			seconds_real = timestamp.seconds - self.robot.time_sync.get_robot_clock_skew().seconds
			nanos_real = timestamp.nanos - self.robot.time_sync.get_robot_clock_skew().nanos

			if nanos_real < 0:
				nanos_real = nanos_real + 1_000_000_000
				seconds_real = seconds_real - 1

			stamp_real = Time()
			stamp_real.sec = seconds_real
			stamp_real.nanosec = nanos_real

			transforms_snapshot = world_object.transforms_snapshot

			body_tform_fiducial = get_a_tform_b(
				transforms_snapshot,
				BODY_FRAME_NAME,
				world_object.apriltag_properties.frame_name_fiducial_filtered
			)

			fiducial = Fiducial()

			fiducial.pose.translation.x = body_tform_fiducial.x
			fiducial.pose.translation.y = body_tform_fiducial.y
			fiducial.pose.translation.z = body_tform_fiducial.z

			fiducial.pose.rotation.x = body_tform_fiducial.rot.x
			fiducial.pose.rotation.y = body_tform_fiducial.rot.y
			fiducial.pose.rotation.z = body_tform_fiducial.rot.z
			fiducial.pose.rotation.w = body_tform_fiducial.rot.w

			fiducial.tag_id = tag_id
			fiducial.header.stamp = stamp_real
			fiducial.header.frame_id = "base_link"

			self.fiducials_publisher.publish(fiducial)

			fid = create_TransformStamped(body_tform_fiducial, "base_link", f"fiducial_{tag_id}", stamp)
			
			self.tf_broadcaster.sendTransform(fid)

			self.get_logger().info('Published fiducial')


def main(args=None):
	rclpy.init(args=args)

	spot_data = SpotData()

	rclpy.spin(spot_data)

	spot_driver.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
