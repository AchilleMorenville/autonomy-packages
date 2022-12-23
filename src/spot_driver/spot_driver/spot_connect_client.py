from std_srvs.srv import Trigger

import rclpy
from rclpy.node import Node

class SpotConnectClient(Node):

	def __init__(self):
		super().__init__('spot_connect_client')

		self.client_spot_driver_odometry_connect = self.create_client(Trigger, "spot_driver/odometry/connect")
		self.client_spot_driver_fiducial_connect = self.create_client(Trigger, "spot_driver/fiducial/connect")
		while not self.client_spot_driver_fiducial_connect.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('fiducial service not available')
		while not self.client_spot_driver_odometry_connect.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('odometry service not available')

	def send_request_spot_driver_connect(self):
		req = Trigger.Request()
		future = self.client_spot_driver_odometry_connect.call_async(req)
		rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
		if future.result() is not None and future.result().success == True:
			self.get_logger().info("Odometry connected to Spot")
		else:
			self.get_logger().info("Can't connect Odometry to Spot")

		req = Trigger.Request()
		future = self.client_spot_driver_fiducial_connect.call_async(req)
		rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
		if future.result() is not None and future.result().success == True:
			self.get_logger().info("Fiducial connected to Spot")
		else:
			self.get_logger().info("Can't connect Fiducial to Spot")

def main():
	rclpy.init()

	connect_client = SpotConnectClient()

	response = connect_client.send_request_spot_driver_connect();

	connect_client.destroy_node()

	rclpy.shutdown()

if __name__ == "__main__":
	main()