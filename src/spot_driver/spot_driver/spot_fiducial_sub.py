import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class SpotFiducialListener(Node):

	def __init__(self):
		super().__init__('spot_fiducial_client')

		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)

		self.timer = self.create_timer(1.0, self.on_timer)

	def on_timer(self):

		try:
			when = self.get_clock().now() - rclpy.time.Duration(seconds=20.0)
			t = self.tf_buffer.lookup_transform(
				'body',
				'fiducial_475',
				when)
			self.get_logger().info(f'Transformation relative to body : {t.transform.translation.x}, {t.transform.translation.y}, {t.transform.translation.z}')

		except TransformException as ex:
			self.get_logger().info(f'Could not transform body to fiducial: {ex}')
			return

		return

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


