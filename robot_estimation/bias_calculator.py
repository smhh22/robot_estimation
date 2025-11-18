import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import math

from sensor_msgs.msg import Imu

class BiasCalculator(Node):
    def __init__(self):
        super().__init__('bias_calculator')
        self.get_logger().info('Bias Calculator Starting.')

        self.gyro_x_sum = 0
        self.gyro_y_sum = 0
        self.gyro_z_sum = 0

        self.accl_x_sum = 0
        self.accl_y_sum = 0
        self.accl_z_sum = 0

        self.count = 0

        self.subscription_ = self.create_subscription(
			Imu, '/zed/zed_node/imu/data_raw', self.listener_callback, qos_profile_sensor_data
		)
        self.get_logger().info("Bias Calculator started. Listening to /zed/zed_node/imu/data_raw...")

        self.i = 0
    
    def listener_callback(self, msg: Imu):
        self.i += 1
        self.count += 1

        self.gyro_x_sum += msg.angular_velocity.x
        self.gyro_y_sum += msg.angular_velocity.y
        self.gyro_z_sum += msg.angular_velocity.z

        self.accl_x_sum += msg.linear_acceleration.x
        self.accl_y_sum += msg.linear_acceleration.y
        self.accl_z_sum += msg.linear_acceleration.z - 9.81

        if self.i % 6000 == 0:
            self.get_logger().info(f'gyro_x_mean: {self.gyro_x_sum / self.count}')
            self.get_logger().info(f'gyro_y_mean: {self.gyro_y_sum / self.count}')
            self.get_logger().info(f'gyro_z_mean: {self.gyro_z_sum / self.count}')
            self.get_logger().info(f'accl_x_mean: {self.accl_x_sum / self.count}')
            self.get_logger().info(f'accl_y_mean: {self.accl_y_sum / self.count}')
            self.get_logger().info(f'accl_z_mean: {self.accl_z_sum / self.count}')

def main(args=None):
	rclpy.init(args=args)
	bias_calculator = BiasCalculator()
	rclpy.spin(bias_calculator)
	bias_calculator.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()