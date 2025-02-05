#!/usr/bin/env python3
import rclpy
from rclpy.time import Time
from rclpy import Duration
from rclpy.node import Node
from substates.utility.controller import Controller
from substates.utility.functions import countdown

def main(args=None):
    rclpy.init(args=args)

    nodeQuali = Node("quali")

    nodeQuali.get_logger().info("_______SLEEPING__________")
    countdown(30)
    controls = Controller(Time(seconds=0))
    nodeQuali.get_logger().info("STARTING")
    controls.rotateDeltaEuler([0,0,0])
    # controls.freeze_pose()
    # controls.freeze_position()
    # controls.freeze_rotation()
    nodeQuali.get_logger().info("ROTATE")
    controls.moveDelta([None, None, -0.75])
    nodeQuali.get_logger().info("MOVE DELTA")
    controls.forceLocal([20,0])
    nodeQuali.get_logger().info("FORCE")
    nodeQuali.get_clock().sleep_for(Duration(seconds=10))

    controls.kill()

if __name__ == '__name__':
    main()
