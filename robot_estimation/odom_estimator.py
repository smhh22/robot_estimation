import rclpy
import math
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64

def get_quaternion_from_euler(roll, pitch, yaw): # I used Gemini to write this function
    """
    Convert an Euler angle to a quaternion.
    Input: roll, pitch, yaw
    Output: geometry_msgs/Quaternion
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return qx, qy, qz, qw

class OdomEstimator(Node):
	def __init__(self):
		super().__init__('odom_estimator')
		self.get_logger().info("Odom Estimator starting.")

		self.last_left_rpm = 0
		self.last_right_rpm = 0

		self.x = self.y = self.z = self.theta = 0

		self.left_subscription_ = self.create_subscription(
			Float64, '/left_wheel_rpm', self.left_callback, 10
		)
		self.right_subscription_ = self.create_subscription(
			Float64, '/right_wheel_rpm', self.right_callback, 10
		)
		self.get_logger().info("Odom Estimator Model started. Listening to /left_wheel_rpm and /right_wheel_rpm...")
		timer_period = 0.02
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.get_logger().info("Timer Created!")

		self.publisher_ = self.create_publisher(Odometry, '/odom', 10)

		self.tf_broadcaster = TransformBroadcaster(self)
	
	def left_callback(self, msg: Float64):
		self.last_left_rpm = msg.data

	def right_callback(self, msg: Float64):
		self.last_right_rpm = msg.data

	def timer_callback(self):
		if self.last_left_rpm == None or self.last_right_rpm == None:
			return
		odom = Odometry()
		odom.header.frame_id = "odom"
		odom.child_frame_id = "base_link"

		w_l = self.last_left_rpm / 60
		w_r = self.last_right_rpm / 60
		v = 0.1 * (w_l + w_r) / 2
		omega = 0.1 * (w_r - w_l) / 1.2

		self.x = self.x + v * math.cos(self.theta) * 0.02
		self.y = self.y + v * math.sin(self.theta) * 0.02
		self.theta += omega * 0.02

		q = get_quaternion_from_euler(0, 0, self.theta)

		odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
		odom.pose.pose.orientation.x = q[0]
		odom.pose.pose.orientation.y = q[1]
		odom.pose.pose.orientation.z = q[2]
		odom.pose.pose.orientation.w = q[3]
		odom.twist.twist.linear.x = v
		odom.twist.twist.angular.z = omega

		self.publisher_.publish(odom)

		t = TransformStamped()
		t.header.stamp = self.get_clock().now().to_msg()
		t.header.frame_id = 'odom'
		t.child_frame_id = 'base_link'

		t.transform.translation.x = self.x
		t.transform.translation.y = self.y
		t.transform.translation.z = 0.0

		t.transform.rotation.x = q[0]
		t.transform.rotation.y = q[1]
		t.transform.rotation.z = q[2]
		t.transform.rotation.w = q[3]
		self.tf_broadcaster.sendTransform(t)


def main(args=None):
	rclpy.init(args=args)
	odom_estimator = OdomEstimator()
	rclpy.spin(odom_estimator)
	odom_estimator.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
		
		
