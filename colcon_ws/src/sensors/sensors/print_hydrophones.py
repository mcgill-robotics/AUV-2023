#!/usr/bin/env python3

from auv_msgs.msg import PingerTimeDifference

import rclpy
from rclpy.node import Node

class HydrophonePrinter(Node):
    def __init__(self):
        self.sub = self.create_subscription(
            PingerTimeDifference,
            '/sensors/hydrophones/pinger_time_difference',
            self.cb_hydrophones_time_difference
        )
        
    def cb_hydrophones_time_difference(self, msg):
        if msg.frequency != 0:
            print(msg)
        
def main():
    rclpy.init_node('hydrophone_printer')
    hydrophone_printer = HydrophonePrinter()
    rclpy.spin(hydrophone_printer)

    hydrophone_printer.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()