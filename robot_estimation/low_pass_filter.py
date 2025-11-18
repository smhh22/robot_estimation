import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import math

from sensor_msgs.msg import Imu

class LowPassFilter(Node):
    def __init__(self):
        super().__init__('low_pass_filter')
        self.get_logger().info('Low-Pass Filter Starting.')

        self.fc = 5
        self.fs = 100 # defined in URDF
        self.alpha = 2*math.pi*self.fc / (2*math.pi*self.fc + self.fs)
        self.prev_msg = None

        self.gyro_x_bias = 1.6036364689146685e-16
        self.gyro_y_bias = -7.590507806713289e-16
        self.gyro_z_bias = 2.7638274001806276e-16
        self.accl_x_bias = -1.6568403043370306
        self.accl_y_bias = -2.469041862536855e-07
        self.accl_z_bias = -0.151072512647818

        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
        self.subscription_ = self.create_subscription(
			Imu, '/zed/zed_node/imu/data_raw', self.listener_callback, qos_profile_sensor_data
		)
        self.get_logger().info("Low-Pass Filter started. Listening to /zed/zed_node/imu/data_raw...")
    
    def listener_callback(self, msg: Imu):
        msg.angular_velocity.x -= self.gyro_x_bias
        msg.angular_velocity.y -= self.gyro_y_bias
        msg.angular_velocity.z -= self.gyro_z_bias
        msg.linear_acceleration.x -= self.accl_x_bias
        msg.linear_acceleration.y -= self.accl_y_bias
        msg.linear_acceleration.z -= self.accl_z_bias
        if self.prev_msg == None:
            self.publisher_.publish(msg)
        else:
            msg.angular_velocity.x = self.alpha*msg.angular_velocity.x + (1 - self.alpha)*self.prev_msg.angular_velocity.x
            msg.angular_velocity.y = self.alpha*msg.angular_velocity.y + (1 - self.alpha)*self.prev_msg.angular_velocity.y
            msg.angular_velocity.z = self.alpha*msg.angular_velocity.z + (1 - self.alpha)*self.prev_msg.angular_velocity.z
            
            msg.linear_acceleration.x = self.alpha*msg.linear_acceleration.x + (1 - self.alpha)*self.prev_msg.linear_acceleration.x
            msg.linear_acceleration.y = self.alpha*msg.linear_acceleration.y + (1 - self.alpha)*self.prev_msg.linear_acceleration.y
            msg.linear_acceleration.z = self.alpha*msg.linear_acceleration.z + (1 - self.alpha)*self.prev_msg.linear_acceleration.z
            self.publisher_.publish(msg)
        self.prev_msg = msg

def main(args=None):
	rclpy.init(args=args)
	low_pass_filter = LowPassFilter()
	rclpy.spin(low_pass_filter)
	low_pass_filter.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()