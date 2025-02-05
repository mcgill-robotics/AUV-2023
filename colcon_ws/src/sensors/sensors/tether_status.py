#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
from std_msgs.msg import Int32

class TetherStatus(Node):
    def __init__(self):
        super().__init__('tether_status')
        # Publisher for tether status
        self.pub_tether_status = self.create_publisher(Int32, "/tether/status", 1)

        # Declare parameters
        self.declare_parameter("ip_address_tether")
        self.declare_parameter("ping_interval")

        # Get parameters
        self.ip_address = self.get_parameter("ip_address_tether").get_parameter_value().string_value
        self.ping_interval = self.get_parameter("ping_interval").get_parameter_value().double_value

        # Create a timer for periodic ping checks
        self.timer = self.create_timer(self.ping_interval, self.is_tether_active)

    def is_tether_active(self):
        # Just for linux. If you want Windows, change
        # -c to -n
        command = f"ping -c 1 {self.ip_address} > /dev/null"
        response = os.system(command)

        # "response == 0" = successful 
        if response == 0:
            self.pub_tether_status.publish(1)
        else:
            self.pub_tether_status.publish(0)

def main(args=None):
    rclpy.init(args=args)
    tether_status_node = TetherStatus()

    try:
        rclpy.spin(tether_status_node)
    except KeyboardInterrupt:
        exit()
    finally:
        tether_status_node.destroy_node()
        rclpy.shutdown()
