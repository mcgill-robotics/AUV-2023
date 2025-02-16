#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from servers.effort_server import EffortServer
from servers.state_quaternion_pid_server import StateQuaternionServer


def main():
    rclpy.init()

    effort_server = EffortServer()
    quaternion_server = StateQuaternionServer()

    effort_server.server._cancel_callback = lambda goal_handle: effort_server.cancel()
    quaternion_server.server._cancel_callback = lambda goal_handle: quaternion_server.cancel()

    rclpy.spin()


if __name__ == "__main__":
    main()

