#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from substates.utility.controller import Controller
from substates.utility.functions import countdown

def main(args=None):
    rclpy.init(args=args)
    node = Node("pooltest")

    controls = Controller(Time(0))

    #controls.flatten()
    #controls.moveDelta([0,0,-0.3])
    #node.get_logger().info("FLATTENING")
    # controls.freeze_pose()
    # controls.freeze_position()
    # controls.freeze_rotation()

    #controls.move([0, 0, -0.75])
    #controls.moveDelta([1, 0, 0])
    #controls.moveDeltaLocal([0, 0, -1])
    # controls.rotate([1, 0, 0, 0])
    # controls.rotateDelta([1, 0, 0, 0])
    #controls.rotateEuler([0, 0, 0])
    controls.rotateEuler([0, 0, 0])
    controls.moveDelta([0,0,-0.5])

    controls.moveDeltaLocal([0.5,0,0])
    controls.moveDeltaLocal([0,0.5,0])
    controls.moveDeltaLocal([-0.5,0,0])
    controls.moveDeltaLocal([0,-0.5,0])
    while rclpy.ok():
        continue
    controls.kill()

if __name__ == "__name__":
    main()
