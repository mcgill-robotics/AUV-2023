#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from tf import euler_from_quaternion, quaternion_from_euler
import numpy as np

RAD_PER_DEG = np.pi / 180.0

class DVLConvariance(Node):
    def __init__(self):
        super().__init__('waterlinked_driver')
        self.pub_vr = self.create_publisher(TwistWithCovarianceStamped, '/sensors/dvl/twist', 1)
        self.pub_dr = self.create_publisher(PoseWithCovarianceStamped, '/sensors/dvl/pose', 1)

        # Declare parameters
        self.declare_parameter("port")
        self.declare_parameter("baudrate")
        self.declare_parameter("quat_variance")

        # Get parameters
        port = self.get_parameter('port').get_parameter_value()
        baudrate = self.get_parameter('baudrate').get_parameter_value()
        quat_variance = self.get_parameter('quat_variance').get_parameter_value()

        # dvl's baud has been set to 115200 but its default is 9600.
        # There is a way to set the baudrate of the dvl through a command.
        try:
            self.conn = serial.Serial(self.port, self.baudrate, timeout=10)
        except serial.serialutil.SerialException:
            self.get_logger().error("ERR: /dev/dvl directory does not exist")
            self.sleep(5)
            exit()

        # dvl's baud has been set to 115200 but its default is 9600.
        # There is a way to set the baudrate of the dvl through a command.
        self.conn.baudrate = baudrate

        if not self.conn.isOpen():
            self.conn.open()

        self.conn.send_break()
        self.conn.flush()

        self.conn.write("wcr\r\n".encode("utf-8"))
        self.conn.flush()

        self.get_logger().info("Reset dead reckoning.")

        
    def loop(self):
        while self.conn.is_open and rclpy.ok():
            try:
                line = self.conn.readline().decode("utf-8")
                if line.startswith("wra"):
                    self.get_logger().info("INFO: DVL dead reckoning reset successful.")
                    break
                elif line.startswith("wrn"):
                    self.get_logger().warn("WARN: DVL dead reckoning reset failed.")
                    break
            except Exception as e:
                self.get_logger().error(str(e))
                break

        # Only grabbing data we care about but this dvl can be used for more.
        # Refer to work horse manual for more info
        start = self.get_clock().now()
        eulers = []
        while self.conn.is_open and rclpy.ok() and self.get_clock().now() - start < rclpy.Duration(30):
            try:
                line = self.conn.readline().decode("utf-8")
                # print(line)
                if line.startswith("wrp"):
                    eulers.append(self.parse_dead_reckon_report(line, self.quat_variance))
            except Exception as e:
                self.get_logger().error(str(e))
                self.conn.close()
                exit()
        eulers = np.array(eulers)
        eulers_cov = np.cov(eulers.T)
        self.get_logger().info("Eulers Covariance: " + str(eulers_cov))

    def parse_dead_reckon_report(self, line, quat_variance):
        tokens = line.split(",")
        time_stamp = float(tokens[1])
        x = float(tokens[2])
        y = float(tokens[3])
        z = float(tokens[4])
        std = float(tokens[5])
        roll = float(tokens[6])
        pitch = float(tokens[7])
        yaw = float(tokens[8])
        status = bool(tokens[9])
        return [roll, pitch, yaw]



def main(args=None):
    rclpy.init(args=args)
    dvl_convariance = DVLConvariance()
    dvl_convariance.loop()
    
    try:
        rclpy.spin(dvl_convariance)
    except KeyboardInterrupt:
        exit()
    finally:
        dvl_convariance.destroy_node()
        rclpy.shutdown()