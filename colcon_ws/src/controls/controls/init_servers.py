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

    # need to use multithreaded executor for multiple nodes
    executor = MultiThreadedExecutor()
    executor.add_node(effort_server)
    executor.add_node(quaternion_server)

    executor.spin()
    
    # the nodes need to be destroyed on shutdown



if __name__ == "__main__":
    main()

