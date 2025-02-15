#! /usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Imu

class SensorCovariance(Node):
    def __init__(self):
        super().__init__('sensor_covariance')
        self.ang_vel = []
        self.accel = []
        self.imu_sub = self.create_subscription(Imu, '/sensors/imu/raw', self.imu_cb)
        self.timer = self.create_timer(30, self.finish)

    def imu_cb(self, msg):
        self.ang_vel.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self.accel.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

    def finish(self):
        self.destroy_subscription(self.imu_sub)
        ang_vel = np.array(self.ang_vel)
        accel = np.array(self.accel)
        self.get_logger().info(str(ang_vel))
        self.get_logger().info(str(accel))
        ang_vel_cov = np.cov(ang_vel.T)
        accel_cov = np.cov(accel.T)
        self.get_logger().info('Angular Velocity Covariance:')
        self.get_logger().info(str(ang_vel_cov))
        self.get_logger().info('Linear Acceleration Covariance:')
        self.get_logger().info(str(accel_cov))

        self.get_logger().info('Finished')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    sensor_covariance = SensorCovariance()

    try:
        rclpy.spin(sensor_covariance)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_covariance.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

        

