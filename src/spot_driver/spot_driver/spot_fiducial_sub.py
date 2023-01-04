import rclpy
from rclpy.node import Node

from autonomous_interfaces.msg import Fiducial, Fiducials


class SpotFiducialListener(Node):

	def __init__(self):
		super().__init__('spot_fiducial_client')

		self.subscription = self.create_subscription(
			Fiducials, 
			"spot_driver/fiducials",
			self.listener_callback,
			10
		)

	def listener_callback(self, msg):
		self.get_logger().info(f"{msg.nbr} fiducials detected")
		self.get_logger().info(f"Fiducials : {[f.tag_id for f in msg.fiducials]}")

def main():
	rclpy.init()
	node = SpotFiducialListener()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass

	rclpy.shutdown()

if __name__=="__main__":
	main()


