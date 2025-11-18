import rclpy
import math
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class ControllerMotionModel(Node):
	def __init__(self):
		super().__init__('controller_motion_model')
		self.get_logger().info("Controller Motion Model starting.")
		self.left_publisher_ = self.create_publisher(Float64, '/left_wheel_rpm', 10)
		self.right_publisher_ = self.create_publisher(Float64, '/right_wheel_rpm', 10)
		self.subscription_ = self.create_subscription(
			Twist, '/cmd_vel', self.listener_callback, 10
		)
		self.get_logger().info("Controller Motion Model started. Listening to /cmd_vel...")
	
	def listener_callback(self, msg: Twist):
		left_msg = Float64()
		left_msg.data = (msg.linear.x - msg.angular.z * 0.6 / 2) / 0.1 * 60 / (2 * math.pi)
		right_msg = Float64()
		right_msg.data = (msg.linear.x + msg.angular.z * 0.6 / 2) / 0.1 * 60 / (2 * math.pi)
		self.left_publisher_.publish(left_msg)
		self.right_publisher_.publish(right_msg)

def main(args=None):
	rclpy.init(args=args)
	controller_motion_model = ControllerMotionModel()
	rclpy.spin(controller_motion_model)
	controller_motion_model.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
		
		
