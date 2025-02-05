#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy import Duration
from rclpy.clock import Clock
from substates.utility.functions import countdown
from substates.utility.controller import *
from substates.utility.state import *
from substates.utility.functions import *
import time


class FacePinger(Node):
    def __init__(self):
        super().__init__('face_pinger')
        self.pub_mission_display = self.create_publisher(String, "/mission_display", 1)
        self.state = StateTracker()
        self.control = Controller(Clock().now().seconds_nanoseconds()[0])

    def update_display(self, status):
        self.pub_mission_display.publish(status)
        self.get_logger().info(status)


def main(args=None):
    rclpy.init(args=args)

    node = FacePinger()
    node.get_logger().info("_______SLEEPING__________")

    while rclpy.ok():
        node.get_clock().sleep_for(Duration(seconds=1))
        pinger_bearings = node.state.pingers_bearings
        if len(pinger_bearings.keys()) > 0:
            frequency, bearing = list(pinger_bearings.items())[0]
            node.update_display(str(frequency))
            node.control.rotateEuler(
                (
                    0,
                    0,
                    180 + vectorToYawDegrees(bearing.x, bearing.y),
                )
            )
        else:
            node.update_display("NONE")


if __name__ == '__main__':
    main()
    