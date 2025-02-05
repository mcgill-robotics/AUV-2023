import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import Imu

class Covariance(Node):
    def __init__(self):
        super().__init__('sensor_covariance')
        self.ang_vel = []
        self.accel = []

        self.imu_sub = self.create_subscription(
            Imu,
            '/sensors/imu/data',
            self.imu_cb,
            rclpy.qos.QoSProfile()  # Use the default
        )
        self.timer = self.create_timer(30.0, self.finish)

    def imu_cb(self, msg):
        self.ang_vel.append()
        self.accel.append()

    def finish(self, event):
        self.imu_sub.unregister()
        self.ang_vel = np.array(self.ang_vel)
        self.accel = np.array(self.accel)
        
        # Use the tranpose matrix to calculate the covariance matrix
        self.ang_vel_cov = np.cov(self.ang_vel.T)
        self.accel_cov = np.cov(self.accel.T)

        self.get_logger().info('Angular Velocity Covariance:')
        self.get_logger().info(self.ang_vel_cov)

        self.get_logger().info('Linear Acceleration Covariance')
        self.get_logger().info(self.accel_cov)


def main(args=None):
    rclpy.init(args=args)
    sensor_covariance = Covariance()

    # TODO: Determine if the node needs to be rate fixed and use rclpy.ok()
    # Create node spin
    rclpy.spin(sensor_covariance)

    sensor_covariance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()