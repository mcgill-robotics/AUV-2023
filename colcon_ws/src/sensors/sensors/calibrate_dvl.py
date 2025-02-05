#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import serial

class CalibrateDVL (Node):
    def __init__(self):
        super().__init__("calibrate_dvl")

        self.declare_parameter("port",)
        self.declare_parameter("baudrate",)

        port = self.get_parameter("port").value().string_value()
        baudrate = self.get_parameter("baudrate").value().integer_value()

        self.conn = serial.Serial(port)
        self.conn.timeout = 10

        self.conn.baudrate = baudrate

        if not self.conn.isOpen():
            self.conn.open()

        self.calibrate()

    def calibrate(self):
        self.conn.send_break()
        self.conn.flush()
        self.get_logger().info("Calibrating DVL...")

        self.conn.write("wcg\r\n".encode("utf-8"))
        self.conn.flush()

        while self.conn.is_open and rclpy.ok():
            try:
                line = self.conn.readline().decode("utf-8")
                if line.startswith("wra"):
                    self.get_logger().info("INFO: DVL gyro calibration successful.")
                    self.conn.close()
                    break
                elif line.startswith("wrn"):
                    self.get_logger().warn("WARN: DVL gyro calibration failed.")
                    self.conn.close()
                    break
            except Exception as e:
                self.get_logger().error(str(e))
                self.conn.close()
                break

def main(args=None):
    rclpy.init(args=args)
    calibrate_dvl = CalibrateDVL()

    try:
        rclpy.spin(calibrate_dvl)

    except KeyboardInterrupt:
        pass

    finally:
        calibrate_dvl.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
    
